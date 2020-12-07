#ifndef _FSM_H_
#define _FSM_H_

#include "self_msgs_and_srvs/GlbObsRcv.h"
#include "occ_grid/occ_map.h"
#include "occ_grid/pos_checker.h"
#include "kino_plan/krrtplanner.h"
#include "visualization_utils/visualization_utils.h"
#include "poly_opt/traj_optimizer.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "quadrotor_msgs/PositionCommand.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>

namespace tgk_planner
{
class FSM
{
public:
  FSM();
  ~FSM();
  void init(const ros::NodeHandle& nh);
    
private:
  bool searchForTraj(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,  
                     Vector3d end_pos, Vector3d end_vel, Vector3d end_acc, 
                     double search_time, const Vector3d &normal, const Vector3d &dire, bool need_consistancy);
  void sendTrajToServer(const Trajectory& poly_traj);
  void sendEStopToServer();
  bool reachGoal(double radius);
  bool needReplan();
  Eigen::VectorXd getReplanStateFromPath(double t, const Trajectory& poly_traj);
  /*
   * replan in t second from current state
   */
  bool replanOnce(double t);
  bool optimize();

  // map, checker, planner 
  OccMap::Ptr env_ptr_;
  PosChecker::Ptr pos_checker_ptr_;
  KRRTPlanner::KRRTPlannerPtr front_end_planner_ptr2_;
  TrajOptimizer::Ptr optimizer_ptr_;
  VisualRviz::Ptr vis_ptr_;
  
  // ros 
  ros::NodeHandle nh_;
  ros::Subscriber goal_sub_;
  ros::Subscriber qrcode_pose_sub_;
  ros::Subscriber track_err_trig_sub_;
  ros::Publisher traj_pub_;
  ros::Timer execution_timer_;
  ros::ServiceClient rcv_glb_obs_client_;
  void qrcodeCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  void goalCallback(const quadrotor_msgs::PositionCommand::ConstPtr& goal_msg);
  void executionCallback(const ros::TimerEvent& event);
  void trackErrCallback(const std_msgs::Empty& msg);

  // execution states 
  enum MACHINE_STATE{
    INIT, 
    WAIT_GOAL, 
    GENERATE_TRAJ, 
    FOLLOW_TRAJ,
    REPLAN_TRAJ, 
    EMERGENCY_TRAJ,
    REFINE_REMAINING_TRAJ
  };
  MACHINE_STATE machine_state_;
  void changeState(MACHINE_STATE new_state);
  void printState();
  
  // params 
  bool track_err_replan_, allow_track_err_replan_, close_goal_traj_;
  bool new_goal_, started_, use_optimization_, replan_, bidirection_;
  Eigen::Vector3d last_goal_pos_;
  double replan_time_;
  Eigen::Vector3d start_pos_, start_vel_, start_acc_, end_pos_, end_vel_, end_acc_;
  ros::Time cuur_traj_start_time_;
  Trajectory front_end_traj_, back_end_traj_, traj_;
  Eigen::Vector3d pos_about_to_collide_;
  double remain_safe_time_, e_stop_time_margin_, replan_check_duration_;
};
    
    
}


#endif //_FSM_H_
