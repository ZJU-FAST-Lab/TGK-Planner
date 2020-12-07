#include "state_machine/fsm.h"
#include <ros/console.h>

namespace tgk_planner
{
  FSM::FSM()
  {
  }

  FSM::~FSM()
  {
  }

  void FSM::init(const ros::NodeHandle &nh)
  {
    env_ptr_.reset(new OccMap);
    env_ptr_->init(nh);

    vis_ptr_.reset(new VisualRviz(nh));

    pos_checker_ptr_.reset(new PosChecker);
    pos_checker_ptr_->init(nh);
    pos_checker_ptr_->setMap(env_ptr_);

    front_end_planner_ptr2_.reset(new KRRTPlanner(nh));
    front_end_planner_ptr2_->init(nh);
    front_end_planner_ptr2_->setPosChecker(pos_checker_ptr_);
    front_end_planner_ptr2_->setVisualizer(vis_ptr_);

    optimizer_ptr_.reset(new TrajOptimizer(nh));
    optimizer_ptr_->setPosChecker(pos_checker_ptr_);
    optimizer_ptr_->setVisualizer(vis_ptr_);

    qrcode_pose_sub_ = nh_.subscribe("/qrcode_detector/qrcode_position", 1, &FSM::qrcodeCallback, this);
    goal_sub_ = nh_.subscribe("/goal", 1, &FSM::goalCallback, this);
    traj_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>("planning/poly_traj", 10);
    execution_timer_ = nh_.createTimer(ros::Duration(0.01), &FSM::executionCallback, this); // 100Hz
    track_err_trig_sub_ = nh_.subscribe("/trig/tracking_err", 1, &FSM::trackErrCallback, this);
    rcv_glb_obs_client_ = nh_.serviceClient<self_msgs_and_srvs::GlbObsRcv>("/pub_glb_obs");

    nh.param("fsm/use_optimization", use_optimization_, false);
    nh.param("fsm/replan", replan_, false);
    nh.param("fsm/replan_time", replan_time_, 0.02);
    nh.param("fsm/allow_track_err_replan", allow_track_err_replan_, false);
    nh.param("fsm/e_stop_time_margin", e_stop_time_margin_, 1.0);
    nh.param("fsm/replan_check_duration", replan_check_duration_, 1.0);
    nh.param("fsm/bidirection", bidirection_, false);
    ROS_WARN_STREAM("[fsm] param: use_optimization: " << use_optimization_);
    ROS_WARN_STREAM("[fsm] param: replan: " << replan_);
    ROS_WARN_STREAM("[fsm] param: replan_time: " << replan_time_);
    ROS_WARN_STREAM("[fsm] param: allow_track_err_replan: " << allow_track_err_replan_);
    ROS_WARN_STREAM("[fsm] param: e_stop_time_margin: " << e_stop_time_margin_);
    ROS_WARN_STREAM("[fsm] param: replan_check_duration: " << replan_check_duration_);
    ROS_WARN_STREAM("[fsm] param: bidirection: " << bidirection_);

    track_err_replan_ = false;
    new_goal_ = false;
    started_ = false;
    last_goal_pos_ << 0.0, 0.0, 0.0;
    machine_state_ = INIT;
    cuur_traj_start_time_ = ros::Time::now();
    pos_about_to_collide_ << 0.0, 0.0, 0.0;
    remain_safe_time_ = 0.0;
  }

  void FSM::trackErrCallback(const std_msgs::Empty &msg)
  {
    if (allow_track_err_replan_)
      track_err_replan_ = true;
  }

  void FSM::qrcodeCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
  {
    Vector3d pos;
    pos << msg->point.x,
        msg->point.y,
        msg->point.z;
    if ((last_goal_pos_ - pos).norm() >= 2.0)
    {
      end_pos_ = pos;
      end_vel_.setZero();
      end_acc_.setZero();
      started_ = true;
      new_goal_ = true;
      last_goal_pos_ = end_pos_;
    }
  }

  void FSM::goalCallback(const quadrotor_msgs::PositionCommand::ConstPtr &goal_msg)
  {
    end_pos_ << goal_msg->position.x,
                goal_msg->position.y,
                goal_msg->position.z;
    end_vel_ << goal_msg->velocity.x,
                goal_msg->velocity.y,
                goal_msg->velocity.z;
    end_acc_ << goal_msg->acceleration.x,
                goal_msg->acceleration.y,
                goal_msg->acceleration.z;
    started_ = true;
    new_goal_ = true;
    last_goal_pos_ = end_pos_;
  }

  void FSM::executionCallback(const ros::TimerEvent &event)
  {
    static ros::Time start_follow_time, collision_detect_time, last_replan_start_time;
    static int replan_state = 1;
    static int fsm_num = 0;
    static StatePVA last_replan_start_state;
    fsm_num++;
    if (fsm_num == 100)
    {
      // printState();
      if (!env_ptr_->odomValid())
      {
        ROS_INFO("no odom.");
      }
      if (!env_ptr_->mapValid())
      {
        ROS_INFO("no map.");
        self_msgs_and_srvs::GlbObsRcv srv;
        if (!rcv_glb_obs_client_.call(srv))
          ROS_WARN("Failed to call service /pub_glb_obs");
      }
      if (!started_)
      {
        ROS_INFO("wait for goal in %lf but actual in %lf", event.current_expected.toSec(), event.current_real.toSec());
      }
      fsm_num = 0;
    }

    switch (machine_state_)
    {
    case INIT:
    {
      if (!env_ptr_->odomValid())
      {
        return;
      }
      if (!env_ptr_->mapValid())
      {
        return;
      }
      if (!started_)
      {
        return;
      }
      changeState(WAIT_GOAL);
      break;
    }

    case WAIT_GOAL:
    {
      if (!new_goal_)
      {
        return;
      }
      else
      {
        new_goal_ = false;
        changeState(GENERATE_TRAJ);
      }
      break;
    }

    case GENERATE_TRAJ:
    {
      start_pos_ = env_ptr_->get_curr_posi();
      start_vel_ = env_ptr_->get_curr_twist();
      //numerical problem
      if (start_vel_.norm() < 0.05)
      {
        start_vel_(0) = 0.0;
        start_vel_(1) = 0.0;
        start_vel_(2) = 0.0;
      }
      start_acc_.setZero();

      Vector3d normal, dire; //unused
      bool success = searchForTraj(start_pos_, start_vel_, start_acc_, end_pos_, end_vel_, end_acc_, replan_time_, normal, dire, false); //TODO what if it can not finish in 10ms?
      if (success)
      {
        front_end_planner_ptr2_->getTraj(traj_);
        if (use_optimization_)
        {
          bool optimize_succ = optimize();
          if (optimize_succ)
          {
            optimizer_ptr_->getTraj(traj_);
          }
        }
        vector<StatePVA> vis_x;
        traj_.sampleWholeTrajectory(&vis_x);
        vis_ptr_->visualizeStates(vis_x, OptimizedTraj, pos_checker_ptr_->getLocalTime());
        replan_state = 1;
        sendTrajToServer(traj_);
        cuur_traj_start_time_ = ros::Time::now();
        new_goal_ = false;
        start_follow_time = ros::Time::now();
        changeState(FOLLOW_TRAJ);
      }
      //else
      //{
      //new_goal_ = false;
      //changeState(WAIT_GOAL);
      //}
      break;
    }

    case FOLLOW_TRAJ:
    {
      double t_during_traj = (ros::Time::now() - cuur_traj_start_time_).toSec();
      if (reachGoal(0.5))
      {
        changeState(WAIT_GOAL);
      }
      else if (new_goal_)
      {
        ROS_WARN("Replan because of new goal received");
        new_goal_ = false;
        remain_safe_time_ = traj_.getTotalDuration() - t_during_traj;
        // ROS_INFO("t_during_traj: %lf", t_during_traj);
        // ROS_INFO("remain_safe_time: %lf", remain_safe_time_);
        collision_detect_time = ros::Time::now();
        changeState(REPLAN_TRAJ);
      }
      else if (replan_)
      {
        //replan because remaining traj may collide
        if (needReplan())
        {
          ROS_WARN("REPLAN because of future collision");
          collision_detect_time = ros::Time::now();
          changeState(REPLAN_TRAJ);
        }
        else if (track_err_replan_)
        {
          track_err_replan_ = false;
          ROS_WARN("REPLAN because of not tracking closely");
          remain_safe_time_ = traj_.getTotalDuration() - t_during_traj;
          replan_state = 4;
          collision_detect_time = ros::Time::now();
          changeState(REPLAN_TRAJ);
        }
        else if (close_goal_traj_ && (traj_.getTotalDuration() - t_during_traj) < 2)
        {
          close_goal_traj_ = false;
          remain_safe_time_ = traj_.getTotalDuration() - t_during_traj;
          collision_detect_time = ros::Time::now();
          ROS_WARN("REPLAN cause t_remain is run out");
          changeState(REPLAN_TRAJ);
        }
      }
      //       else if ((ros::Time::now() - start_follow_time).toSec() > 2)
      //       {//replan because
      //         changeState(REFINE_REMAINING_TRAJ);
      //       }
      break;
    }

    case REPLAN_TRAJ:
    {
      ros::Time t_replan_start = ros::Time::now();
      //replan once
      double curr_remain_safe_time = remain_safe_time_ - (ros::Time::now() - collision_detect_time).toSec();
      double dt = max(0.0, min(replan_time_, curr_remain_safe_time));
      //start state is in dt (replan_front_time + optimization_time) second from current traj state
      double t_during_traj = (ros::Time::now() - cuur_traj_start_time_).toSec();
      Eigen::VectorXd start_state;
      bool need_consistancy(false);
      Eigen::Vector3d normal, replan_direction;
      if (replan_state == 1)
      {
        // ROS_INFO_STREAM("state 1, replan from tracking traj");
        start_state = getReplanStateFromPath(t_during_traj + dt, traj_); //start in future state
        // for replan consistancy consideration, not used currently
        // if ((start_state.head(3) - last_replan_start_state.head(3)).norm() <= 1)
        // {
        //   Eigen::Vector3d curr_dire = start_state.segment(3, 3);
        //   Eigen::Vector3d last_dire = last_replan_start_state.segment(3, 3);
        //   normal = (curr_dire.cross(last_dire)).normalized();
        //   replan_direction = (curr_dire.cross(normal)).normalized();
        //   need_consistancy = true;
        // }
      }
      else if (replan_state == 4)
      {
        // ROS_INFO_STREAM("state 4, replan from curr state");
        start_state = getReplanStateFromPath(-1.0, traj_); //start in curr state
      }
      ROS_WARN_STREAM("need_consistancy: " << need_consistancy);
      Eigen::Vector3d start_pos, start_vel, start_acc;
      start_pos = start_state.segment(0, 3);
      start_vel = start_state.segment(3, 3);
      start_acc = start_state.segment(6, 3);
      if (start_vel.norm() < 1e-4)
        start_vel = Vector3d(0.0, 0.0, 0.0);
      if (start_acc.norm() < 1e-4)
        start_acc = Vector3d(0.0, 0.0, 0.0);
      double front_time = dt;
      if (dt <= 0.005)
        front_time = replan_time_;
      last_replan_start_state = start_state;
      bool success = searchForTraj(start_pos, start_vel, start_acc, end_pos_, end_vel_, end_acc_, front_time, normal, replan_direction, need_consistancy);
      //found a traj towards goal
      if (success)
      {
        front_end_planner_ptr2_->getTraj(traj_);
        ROS_WARN("Replan front-end success");
        if (use_optimization_)
        {
          bool optimize_succ = optimize();
          if (optimize_succ)
          {
            ROS_WARN("Replan back-end success");
            optimizer_ptr_->getTraj(traj_);
          }
        }
        vector<StatePVA> vis_x;
        traj_.sampleWholeTrajectory(&vis_x);
        vis_ptr_->visualizeStates(vis_x, OptimizedTraj, pos_checker_ptr_->getLocalTime());
        double replan_duration = (ros::Time::now() - t_replan_start).toSec();
        if (replan_duration < dt)
        {
          ROS_WARN("wait for it: %lf", dt - replan_duration);
          ros::Duration(dt - replan_duration).sleep();
        }
        replan_state = 1;
        sendTrajToServer(traj_);
        cuur_traj_start_time_ = ros::Time::now();
        new_goal_ = false;
        start_follow_time = ros::Time::now();
        changeState(FOLLOW_TRAJ);
      }
      else
      {
        double curr_remain_safe_time = remain_safe_time_ - (ros::Time::now() - collision_detect_time).toSec();
        ROS_ERROR("Replan fail, %lf seconds to collide", curr_remain_safe_time);
        if (curr_remain_safe_time < e_stop_time_margin_)
        {
          sendEStopToServer();
          ROS_ERROR("ABOUT TO CRASH!! SERVER EMERGENCY STOP!!");
          changeState(GENERATE_TRAJ);
        }
        else
        {
          ROS_WARN("keep replanning");
        }
        break;
      }
      if (needReplan())
      {
        ROS_WARN("update future collision info");
        collision_detect_time = ros::Time::now();
      }
      else
      {
        ROS_WARN("future collision unchanged");
      }
      break;
    }

    case EMERGENCY_TRAJ:
    {

      break;
    }

    default:
      break;
    }
  }

  bool FSM::searchForTraj(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                          Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                          double search_time, const Vector3d &normal, const Vector3d &dire, bool need_consistancy)
  {
    int result(false);
    result = front_end_planner_ptr2_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time, normal, dire, need_consistancy);
    if (result == SUCCESS)
    {
      close_goal_traj_ = false;
      return true;
    }
    else if (result == SUCCESS_CLOSE_GOAL)
    {
      close_goal_traj_ = true;
      return true;
    }
    else
      return false;
  }

  bool FSM::optimize()
  {
    ros::Time optimize_start_time = ros::Time::now();
    if (!optimizer_ptr_->setFrontEndTraj(traj_))
      return false;
    bool res = optimizer_ptr_->solve();
    ros::Time optimize_end_time = ros::Time::now();
    ROS_INFO_STREAM("optimize time: " << (optimize_end_time - optimize_start_time).toSec());
    return res;
  }

  void FSM::sendTrajToServer(const Trajectory &poly_traj)
  {
    static int traj_id = 0;
    int path_seg_num = poly_traj.getPieceNum();
    if (path_seg_num < 1)
      return;
    quadrotor_msgs::PolynomialTrajectory traj;
    traj.trajectory_id = ++traj_id;
    traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
    traj.num_segment = path_seg_num;
    traj.time = poly_traj.getDurations();

    for (int i = 0; i < path_seg_num; ++i)
    {
      traj.order.push_back(5);
      for (size_t j = 0; j <= traj.order[i]; ++j)
      {
        CoefficientMat posCoeffsMat = poly_traj[i].getCoeffMat();
        traj.coef_x.push_back(posCoeffsMat(0, j));
        traj.coef_y.push_back(posCoeffsMat(1, j));
        traj.coef_z.push_back(posCoeffsMat(2, j));
      }
    }

    traj.header.frame_id = "map";
    traj.header.stamp = ros::Time::now();
    traj_pub_.publish(traj);
  }

  void FSM::sendEStopToServer()
  {
    quadrotor_msgs::PolynomialTrajectory traj;
    traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT;

    traj.header.frame_id = "map";
    traj.header.stamp = ros::Time::now();
    traj_pub_.publish(traj);
  }

  bool FSM::reachGoal(double radius)
  {
    Eigen::Vector3d pos_now = env_ptr_->get_curr_posi();
    if ((end_pos_ - pos_now).norm() < radius)
      return true;
    else
      return false;
  }

  inline bool FSM::needReplan()
  {
    double t_during_traj = (ros::Time::now() - cuur_traj_start_time_).toSec();
    double t_check_until_traj = std::min(traj_.getTotalDuration(), t_during_traj + replan_check_duration_);
    if (!pos_checker_ptr_->checkPolyTraj(traj_, t_during_traj, t_check_until_traj, pos_about_to_collide_, remain_safe_time_))
    {
      ROS_INFO_STREAM("about to collide pos: " << pos_about_to_collide_.transpose() << ", remain safe time: " << remain_safe_time_);
      vis_ptr_->visualizeCollision(pos_about_to_collide_, pos_checker_ptr_->getLocalTime());
      return true;
    }
    return false;
  }

  Eigen::VectorXd FSM::getReplanStateFromPath(double t, const Trajectory &poly_traj)
  {
    Eigen::Vector3d pos(0.0, 0.0, 0.0), vel(0.0, 0.0, 0.0), acc(0.0, 0.0, 0.0);
    Eigen::VectorXd start_state(9);
    if (t < 0)
    {
      ROS_ERROR("not tracking well! use curr state as start state");
      pos = env_ptr_->get_curr_posi();
      vel = env_ptr_->get_curr_twist();
      acc = env_ptr_->get_curr_acc();
    }
    else
    {
      t = std::min(t, poly_traj.getTotalDuration());
      pos = poly_traj.getPos(t);
      vel = poly_traj.getVel(t);
      acc = poly_traj.getAcc(t);
    }
    //TODO what if pos is not free??
    start_state << pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], acc[0], acc[1], acc[2];
    return start_state;
  }

  void FSM::printState()
  {
    string state_str[6] = {"INIT", "WAIT_GOAL", "GENERATE_TRAJ", "FOLLOW_TRAJ", "REPLAN_TRAJ", "EMERGENCY_TRAJ"};
    ROS_INFO_STREAM("[FSM]: state: " << state_str[int(machine_state_)]);
  }

  void FSM::changeState(FSM::MACHINE_STATE new_state)
  {
    string state_str[6] = {"INIT", "WAIT_GOAL", "GENERATE_TRAJ", "FOLLOW_TRAJ", "REPLAN_TRAJ", "EMERGENCY_TRAJ"};
    ROS_INFO_STREAM("[FSM]: change from " << state_str[int(machine_state_)] << " to " << state_str[int(new_state)]);
    machine_state_ = new_state;
  }

} // namespace tgk_planner
