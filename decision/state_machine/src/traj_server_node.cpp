#include <ros/ros.h>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Empty.h>
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"

using std::vector;

const int  _DIM_x = 0;
const int  _DIM_y = 1;
const int  _DIM_z = 2;

enum ServerState{INIT, TRAJ, HOVER} _state = INIT;
ros::Publisher _cmd_pub, _cmd_vis_pub, _track_err_trig_pub;
Eigen::Vector3d _curr_posi;
Eigen::Vector3d _goal_point;
quadrotor_msgs::PositionCommand _cmd, _last_cmd;
Eigen::Vector3d _initial_pos;
Eigen::Quaterniond _initial_q;
bool _first_odom = true;

// configuration for polynomial trajectory
int _n_segment = 0;
int _traj_id = 0;
uint32_t _traj_flag = 0;
Eigen::VectorXd _time;
Eigen::MatrixXd _coef[3];
vector<int> _order;
double _mag_coeff;
ros::Time _final_time = ros::TIME_MIN;
ros::Time _start_time = ros::TIME_MAX;
double _start_yaw = 0.0, _final_yaw = 0.0;
bool _receive_traj = false;
double _real_yaw;
Eigen::Quaterniond _q;
Eigen::Vector3d _euler;

//yaw control
double _yaw_error, _last_yaw_error, _cum_yaw_error, _slop_yaw_error;
geometry_msgs::Point _yaw;

//so3 ctrl
double pos_gain[3] = { 5.7, 5.7, 6.2 };
double vel_gain[3] = { 3.4, 3.4, 4.0 };

/*
  * use Qingjiushao's method to calculate the value of f(t) = p0 + p1*t^1 + ... + pn*t^n
  * coeff[i]: pn, pn-1, pn-2, ..., p1, p0
  * order: n
  */
inline double calPosFromCoeff(double t, const Eigen::VectorXd &coeff, const int& order)
{
  double f = coeff[0];
  for (int i=0; i<order; ++i) 
  {
    f = t * f + coeff[i+1];
  }
  return f;
}

inline double calVelFromCoeff(double t, const Eigen::VectorXd &coeff, const int& order)
{
  double f = order * coeff[0];
  for (int i=0; i<order-1; ++i) 
  {
    f = t * f + (order - i - 1) * coeff[i+1];
  }
  return f;
}

inline double calAccFromCoeff(double t, const Eigen::VectorXd &coeff, const int& order)
{
  double f = order * (order-1) * coeff[0];
  for (int i=0; i<order - 2; ++i) 
  {
    f = t * f + (order - i - 1) * (order - i - 2) * coeff[i+1];
  }
  return f;
}

inline void calPVAFromCoeff(Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Vector3d& acc, 
                            const Eigen::VectorXd &x_coeff, const Eigen::VectorXd &y_coeff, const Eigen::VectorXd &z_coeff, 
                            double t, const int& order)
{
  pos[0] = calPosFromCoeff(t, x_coeff, order);
  pos[1] = calPosFromCoeff(t, y_coeff, order);
  pos[2] = calPosFromCoeff(t, z_coeff, order);
  vel[0] = calVelFromCoeff(t, x_coeff, order);
  vel[1] = calVelFromCoeff(t, y_coeff, order);
  vel[2] = calVelFromCoeff(t, z_coeff, order);
  acc[0] = calAccFromCoeff(t, x_coeff, order);
  acc[1] = calAccFromCoeff(t, y_coeff, order);
  acc[2] = calAccFromCoeff(t, z_coeff, order);
}

Eigen::Vector3d getRPY(const Eigen::Quaterniond &quat)
{
    double rotMat[3][3] = {(1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z())), 2 * (quat.x() * quat.y() - quat.w() * quat.z()), 2 * (quat.x() * quat.z() + quat.w() * quat.y()),
                           2 * (quat.x() * quat.y() + quat.w() * quat.z()), (1 - 2 * (quat.x() * quat.x() + quat.z() * quat.z())), 2 * (quat.y() * quat.z() - quat.w() * quat.x()),
                           2 * (quat.x() * quat.z() - quat.w() * quat.y()), 2 * (quat.y() * quat.z() + quat.w() * quat.x()), (1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y()))};

    double yaw, pitch, roll;
    if (rotMat[2][0] != 1 && rotMat[2][0] != -1)
    {
        double yaw1, yaw2, pitch1, pitch2, roll1, roll2;
        pitch1 = -asin(rotMat[2][0]);
        pitch2 = M_PI - pitch1;
        double cos_pitch1 = cos(pitch1);
        double cos_pitch2 = cos(pitch2);
        roll1 = atan2(rotMat[2][1] / cos_pitch1, rotMat[2][2] / cos_pitch1);
        roll2 = atan2(rotMat[2][1] / cos_pitch2, rotMat[2][2] / cos_pitch2);
        yaw1 = atan2(rotMat[1][0] / cos_pitch1, rotMat[0][0] / cos_pitch1);
        yaw2 = atan2(rotMat[1][0] / cos_pitch2, rotMat[0][0] / cos_pitch2);
        if (fabs(pitch1) <= fabs(pitch2))
        {
            yaw = yaw1;
            pitch = pitch1;
            roll = roll1;
        }
        else
        {
            yaw = yaw2;
            pitch = pitch2;
            roll = roll2;
        }
    }
    else if (rotMat[2][0] == 1)
    {
        yaw = 0;
        pitch = M_PI / 2;
        roll = yaw + atan2(rotMat[0][1], rotMat[0][2]);
    }
    else
    {
        yaw = 0;
        pitch = -M_PI / 2;
        roll = -yaw + atan2(-rotMat[0][1], -rotMat[0][2]);
    }

    return Eigen::Vector3d(roll, pitch, yaw);
}

void polyTrajCallback(const quadrotor_msgs::PolynomialTrajectory& traj)
{
  if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ADD)
  {   
    ROS_WARN("[SERVER] Loading the trajectory.");
    if ((int)traj.trajectory_id < _traj_id) return ;

    _state = TRAJ;
    _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    _traj_id = traj.trajectory_id;
    _n_segment = traj.num_segment;
    _final_time = _start_time = traj.header.stamp;
    _time.resize(_n_segment);

    _order.clear();
    for (int idx = 0; idx < _n_segment; ++idx)
    {
      _final_time += ros::Duration(traj.time[idx]);
      _time(idx) = traj.time[idx];
      _order.push_back(traj.order[idx]);
    }
    _start_yaw = traj.start_yaw;
    _final_yaw = traj.final_yaw;
    _mag_coeff = traj.mag_coeff;

    int max_order = *max_element( begin( _order ), end( _order ) ); 
    _coef[_DIM_x] = Eigen::MatrixXd::Zero(max_order + 1, _n_segment);
    _coef[_DIM_y] = Eigen::MatrixXd::Zero(max_order + 1, _n_segment);
    _coef[_DIM_z] = Eigen::MatrixXd::Zero(max_order + 1, _n_segment);
    
    //ROS_WARN("stack the coefficients");
    int shift = 0;
    for (int idx = 0; idx < _n_segment; ++idx)
    {     
      int order = traj.order[idx];
      for (int j = 0; j < (order + 1); ++j)
      {
        _coef[_DIM_x](j, idx) = traj.coef_x[shift + j];
        _coef[_DIM_y](j, idx) = traj.coef_y[shift + j];
        _coef[_DIM_z](j, idx) = traj.coef_z[shift + j];
      }
      shift += (order + 1);
    }

    //compute the goal
    _goal_point[0] = calPosFromCoeff(_time(_n_segment-1), _coef[_DIM_x].col(_n_segment-1), _order[_n_segment-1]);
    _goal_point[1] = calPosFromCoeff(_time(_n_segment-1), _coef[_DIM_y].col(_n_segment-1), _order[_n_segment-1]);
    _goal_point[2] = calPosFromCoeff(_time(_n_segment-1), _coef[_DIM_z].col(_n_segment-1), _order[_n_segment-1]);
  
    _receive_traj = true;
  }
  else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT) 
  {
    ROS_WARN("[SERVER] Aborting the trajectory! EMERGENCY STOP!!");
    _state = HOVER;
    _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
  }
  else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE)
  {
    _state = HOVER;
    _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
  }
}

void odomCallbck(const nav_msgs::OdometryConstPtr odom)
{
  _curr_posi[0] = odom->pose.pose.position.x;
  _curr_posi[1] = odom->pose.pose.position.y;
  _curr_posi[2] = odom->pose.pose.position.z;
  if (_first_odom)
  {
    _first_odom = false;
    _initial_pos[0] = odom->pose.pose.position.x;
    _initial_pos[1] = odom->pose.pose.position.y;
    _initial_pos[2] = odom->pose.pose.position.z;
    _initial_q.w() = odom->pose.pose.orientation.w;
    _initial_q.x() = odom->pose.pose.orientation.x;
    _initial_q.y() = odom->pose.pose.orientation.y;
    _initial_q.z() = odom->pose.pose.orientation.z;
  }

  /* no publishing before receive traj */
//  if (!_receive_traj)
//    return;
  
  // #1. check if it is right state
  if (_state == INIT) 
  {
    _cmd.header = odom->header;
		
    _cmd.position.x = _initial_pos[0];
    _cmd.position.y = _initial_pos[1];
    _cmd.position.z = _initial_pos[2];
		
    _cmd.velocity.x = 0.0;
    _cmd.velocity.y = 0.0;
    _cmd.velocity.z = 0.0;
    
    _cmd.acceleration.x = 0.0;
    _cmd.acceleration.y = 0.0;
    _cmd.acceleration.z = 0.0;

  }
  if (_state == HOVER)
  {
    _cmd.header = odom  ->header;
    _cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;

    _cmd.position = _last_cmd.position;
    
    _cmd.velocity.x = 0.0;
    _cmd.velocity.y = 0.0;
    _cmd.velocity.z = 0.0;
    
    _cmd.acceleration.x = 0.0;
    _cmd.acceleration.y = 0.0;
    _cmd.acceleration.z = 0.0;
    
    _cmd.yaw_dot = 0.0;
    _cmd.yaw = _last_cmd.yaw;
  }
  // #2. locate the trajectory segment
  if (_state == TRAJ)
  {   
    //(odom freq >= cmd timer freq) has to be satisfied or it will cause several diffferent cmd in the same time stamp
    
    double t = std::max(0.0, (ros::Time::now() - _start_time).toSec());
    if (t >= (_final_time - _start_time).toSec() - 0.02)
      _state = HOVER;
    
    // #3. calculate the desired states
    //ROS_WARN("[SERVER] the time : %.3lf\n, n = %d, m = %d", t, _n_order, _n_segment);
    for (int idx = 0; idx < _n_segment; ++idx)
    {
      if (t > _time[idx] && idx + 1 < _n_segment)
      {
        t -= _time[idx];
      }
      else
      { 
        _cmd.header = odom->header;
		
        int cur_order = _order[idx];
        Eigen::Vector3d pos(0.0, 0.0, 0.0), vel(0.0, 0.0, 0.0), acc(0.0, 0.0, 0.0);
        calPVAFromCoeff(pos, vel, acc, _coef[_DIM_x].col(idx), _coef[_DIM_y].col(idx), _coef[_DIM_z].col(idx), t, cur_order);
        _cmd.position.x = pos[0];
        _cmd.position.y = pos[1];
        _cmd.position.z = pos[2];
        _cmd.velocity.x = vel[0];
        _cmd.velocity.y = vel[1];
        _cmd.velocity.z = vel[2];
        _cmd.acceleration.x = acc[0];
        _cmd.acceleration.y = acc[1];
        _cmd.acceleration.z = acc[2];

		//use look_forward yaw planning
        double look_forward_time = std::min(1/vel.norm(), std::max(0.0, (_final_time - _start_time).toSec() - t - 0.02));
        if (look_forward_time == 0)
        {
          _cmd.yaw = _last_cmd.yaw;
        }
        else
        {
          Eigen::Vector3d pos_lf(0.0, 0.0, 0.0), vel_lf(0.0, 0.0, 0.0), acc_lf(0.0, 0.0, 0.0);
          calPVAFromCoeff(pos_lf, vel_lf, acc_lf, _coef[_DIM_x].col(idx), _coef[_DIM_y].col(idx), _coef[_DIM_z].col(idx), t + look_forward_time, cur_order);
          //use look_forward yaw planning
          _cmd.yaw = atan2(pos_lf[1] - pos[1], pos_lf[0] - pos[0]);
          //use tangent direction for yaw planning
          //_cmd.yaw = atan2(_cmd.velocity.y, _cmd.velocity.x); //(-pi, pi]
          double d_yaw = _cmd.yaw - _last_cmd.yaw;
          if (d_yaw >= M_PI)
          {
            d_yaw -= M_PI;
            d_yaw -= M_PI;
          }
          if (d_yaw <= -M_PI)
          {
            d_yaw += M_PI;
            d_yaw += M_PI;
          }
          double d_yaw_abs = fabs(d_yaw);
          if (d_yaw_abs >= 0.02)
            _cmd.yaw = _last_cmd.yaw + d_yaw / d_yaw_abs * 0.02;
        }
        //or use tangent direction for yaw planning

        _last_cmd = _cmd;
        break;
      } 
    }
  }
  // #4. just publish
  _cmd_pub.publish(_cmd);
  Eigen::Vector3d desire_pos(_cmd.position.x, _cmd.position.y, _cmd.position.z);

  static tf2_ros::TransformBroadcaster br_map_ego_desired;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = odom->header.stamp;
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "ego_desired";
  transformStamped.transform.translation.x = _cmd.position.x;
  transformStamped.transform.translation.y = _cmd.position.y;
  transformStamped.transform.translation.z = _cmd.position.z;
  Eigen::AngleAxisd aa(_cmd.yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond d_q(aa);  
  transformStamped.transform.rotation.x = d_q.x();
  transformStamped.transform.rotation.y = d_q.y();
  transformStamped.transform.rotation.z = d_q.z();
  transformStamped.transform.rotation.w = d_q.w();
  br_map_ego_desired.sendTransform(transformStamped);

  if ((_curr_posi - desire_pos).norm() >= 1.0)
  {
    std_msgs::Empty empty_msg;
    _track_err_trig_pub.publish(empty_msg);
  }

  if ((_curr_posi - _goal_point).norm() <= 0.01) 
    _state = HOVER;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;

  ros::Subscriber poly_sub = node.subscribe("planning/poly_traj", 1, polyTrajCallback);
  ros::Subscriber odom_sub = node.subscribe("/curr_state_sub_topic", 50, odomCallbck, ros::TransportHints().tcpNoDelay());
  //ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  _cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  _cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  _track_err_trig_pub = node.advertise<std_msgs::Empty>("/trig/tracking_err", 1);

  /* control parameter for so3 control*/
  _cmd.kx[0] = pos_gain[0];
  _cmd.kx[1] = pos_gain[1];
  _cmd.kx[2] = pos_gain[2];

  _cmd.kv[0] = vel_gain[0];
  _cmd.kv[1] = vel_gain[1];
  _cmd.kv[2] = vel_gain[2];

  ros::Duration(1.0).sleep();
  ROS_WARN("[Traj server]: ready.");
  ros::spin();
  return 0;
}
