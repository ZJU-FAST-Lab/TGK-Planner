#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <math.h>
#include <fstream>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include "quadrotor_msgs/PositionCommand.h"
#include "poly_opt/traj_optimizer.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "krrtstar/utils.h"
#include "vis_utils/planning_visualization.h"

using Eigen::Vector3d;
using namespace std;

Vector3d curr_pos(0.0, 0.0, 0.0);
ros::Publisher traj_pub, goal_pub;
ros::Time latest_time;

ros::Publisher ref_traj_pos_point_pub_;
ros::Publisher ref_traj_vel_vec_pub_;
ros::Publisher ref_traj_acc_vec_pub_;

void sendTraj(const RRTNodeVector& path_nodes)
{
  static int traj_id = 0;
  int path_seg_num = path_nodes.size() - 1;
  if (path_seg_num < 1) 
    return;
  quadrotor_msgs::PolynomialTrajectory traj;
  traj.trajectory_id = ++traj_id;
  traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
  traj.num_segment = path_seg_num;
  for (int i=0; i<path_seg_num; ++i) 
  {
    traj.time.push_back(path_nodes[path_seg_num-i-1].tau_from_parent);
    traj.order.push_back(path_nodes[path_seg_num-i-1].n_order);
    for (size_t j=0; j<traj.order[i]+1 ;++j) 
    {
      traj.coef_x.push_back(path_nodes[path_seg_num-i-1].x_coeff[j]);
      traj.coef_y.push_back(path_nodes[path_seg_num-i-1].y_coeff[j]);
      traj.coef_z.push_back(path_nodes[path_seg_num-i-1].z_coeff[j]);
    }
  }
  traj.header.frame_id = "map";
  traj.header.stamp = ros::Time::now();
  traj_pub.publish(traj);
  ROS_INFO("pubed one ref traj");
}

void odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
  curr_pos(0) = odom->pose.pose.position.x;
  curr_pos(1) = odom->pose.pose.position.y;
  curr_pos(2) = odom->pose.pose.position.z;
  latest_time = odom->header.stamp;
}

double calPosFromCoeff(double t, const double* coeff, const int& order)
{
  double f = coeff[0];
  for (int i=0; i<order; ++i) 
  {
    f = t * f + coeff[i+1];
  }
  return f;
}
double calVelFromCoeff(double t, const double* coeff, const int& order)
{
  double f = order * coeff[0];
  for (int i=0; i<order-1; ++i) 
  {
    f = t * f + (order - i - 1) * coeff[i+1];
  }
  return f;
}
double calAccFromCoeff(double t, const double* coeff, const int& order)
{
  double f = order * (order-1) * coeff[0];
  for (int i=0; i<order - 2; ++i) 
  {
    f = t * f + (order - i - 1) * (order - i - 2) * coeff[i+1];
  }
  return f;
}
void calPVAFromCoeff(Vector3d& pos, Vector3d& vel, Vector3d& acc, 
                    const double* x_coeff, const double* y_coeff, const double* z_coeff, 
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

void visualizeRefTraj(const std::vector<Eigen::Matrix<double,6,1>>& x, const std::vector<Eigen::Matrix<double,3,1>>& u, ros::Time local_time) 
{
    if (x.empty() || u.empty())
        return;
    
    visualization_msgs::Marker pos_point, vel_vec, acc_vec;
    geometry_msgs::Point p, a;
    //x and u are of same size;
    for (int i=0; i<x.size(); ++i) {
        p.x = x[i](0,0);
        p.y = x[i](1,0);
        p.z = x[i](2,0);
        a.x = x[i](0,0);
        a.y = x[i](1,0);
        a.z = x[i](2,0);
        pos_point.points.push_back(p);
                
        vel_vec.points.push_back(p);
        p.x += x[i](3,0)/5.0;
        p.y += x[i](4,0)/5.0;
        p.z += x[i](5,0)/5.0;
        vel_vec.points.push_back(p);
        
        acc_vec.points.push_back(a);
        a.x += u[i](0,0)/1.0;
        a.y += u[i](1,0)/1.0;
        a.z += u[i](2,0)/1.0;
        acc_vec.points.push_back(a);
    }
    
    pos_point.header.frame_id = "map";
    pos_point.header.stamp = local_time;
    pos_point.ns = "ref_traj";
    pos_point.action = visualization_msgs::Marker::ADD;
    pos_point.lifetime = ros::Duration(0);
    pos_point.pose.orientation.w = 1.0;
    pos_point.id = 101;
    pos_point.type = visualization_msgs::Marker::POINTS;
    pos_point.scale.x = 0.03;
    pos_point.scale.y = 0.03;
    pos_point.scale.z = 0.03;
    pos_point.color.r = 0.118;
    pos_point.color.g = 0.58;
    pos_point.color.b = 1; // blue
    pos_point.color.a = 1.0;
    
    vel_vec.header.frame_id = "map";
    vel_vec.header.stamp = local_time;
    vel_vec.ns = "ref_traj";
    vel_vec.action = visualization_msgs::Marker::ADD;
    vel_vec.lifetime = ros::Duration(0);
    vel_vec.pose.orientation.w = 1.0;
    vel_vec.id = 201;
    vel_vec.type = visualization_msgs::Marker::LINE_LIST;
    vel_vec.scale.x = 0.03;
    vel_vec.color.r = 1.0f; 
    vel_vec.color.g = 0.5f;
    vel_vec.color.a = 1.0;
    
    acc_vec.header.frame_id = "map";
    acc_vec.header.stamp = local_time;
    acc_vec.ns = "ref_traj";
    acc_vec.action = visualization_msgs::Marker::ADD;
    acc_vec.lifetime = ros::Duration(0);
    acc_vec.pose.orientation.w = 1.0;
    acc_vec.id = 301;
    acc_vec.type = visualization_msgs::Marker::LINE_LIST;
    acc_vec.scale.x = 0.03;
    acc_vec.color.r = 1;
    acc_vec.color.g = 0.55f;
    acc_vec.color.a = 0.5;
    
    ref_traj_pos_point_pub_.publish(pos_point);
    ref_traj_vel_vec_pub_.publish(vel_vec);
    ref_traj_acc_vec_pub_.publish(acc_vec);
}

int main(int argc, char** argv) 
{ 
  ros::init(argc, argv, "pub_goal_node");
  ros::NodeHandle nh("~");
  goal_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/goal", 100);
  traj_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/ref_traj", 1);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom_topic", 1, &odomCallback, ros::TransportHints().tcpNoDelay());
  ref_traj_pos_point_pub_ = nh.advertise<visualization_msgs::Marker>("ref_traj_pos", 1);
  ref_traj_vel_vec_pub_ = nh.advertise<visualization_msgs::Marker>("ref_traj_vel", 1);
  ref_traj_acc_vec_pub_ = nh.advertise<visualization_msgs::Marker>("ref_traj_acc", 1);
  ros::Duration(2).sleep();
  double avg_vel;
  std::string waypoints_path;
  nh.param("avg_vel", avg_vel, 0.0);
  nh.param<std::string>("waypoints_path", waypoints_path, "/home/dji/data/ws/src/tp1917/krrt_planner/decision/state_machine/launch/indoor5F.txt");
  cout << waypoints_path << endl;
  ifstream infile;
  infile.open(waypoints_path);

  int waypoint_num(0);
  vector<Vector3d> waypoints;
  string s;
  if (getline(infile, s))
  {
    cout << s << endl;
    waypoint_num = stoi(s);
  }
  else 
  {
    cout << "fail open " << s << endl;
  }
  int i=0;
  double px(0.0), py(0.0), pz(0.0);
  while (getline(infile, s))
  {
    cout << s << endl;
    i++;
    if (i%3 == 1)
    {
      px = stod(s);
      cout << px << endl;
    }
    else if (i%3 == 2)
    {
      py = stod(s);
      cout << py << endl;
    }
    else if (i%3 == 0)
    {
      pz = stod(s);
      cout << pz << endl;
      waypoints.push_back(Vector3d(px, py, pz));
    }
  }
  infile.close();
  cout << "file waypoint_num: " << waypoint_num << ", read waypoint num: " << waypoints.size() << endl;
  if (waypoint_num != waypoints.size())
  {
    cout << "waypoint reading not match" << endl;
    return 0;
  }
  if (waypoint_num == 0)
  {
    cout << "no waypoint" << endl;
    return 0;
  }
  for (const auto& p : waypoints)
  {
    cout << "reading waypoints: " << p.transpose() << endl;
  }

  vector<Vector3d> wps = waypoints, vels, accs;
  ros::spinOnce();
  ros::spinOnce();
  wps.insert(wps.begin(), curr_pos);
  Vector3d temp(0.0, 0.0, 0.0);
  Eigen::VectorXd time_assign;
  time_assign.resize(wps.size() - 1);
  for (int i = 0; i < wps.size() - 1; ++i)
  {
    time_assign[i] = (wps[i] - wps[i+1]).norm() / avg_vel;
    vels.push_back(temp);
    accs.push_back(temp);
  }
  vels.push_back(temp);
  accs.push_back(temp);
  tgk_planner::TrajOptimizer::Ptr ref_traj_planner_ptr;
  ref_traj_planner_ptr.reset(new tgk_planner::TrajOptimizer(nh));
  ref_traj_planner_ptr->setWayPointsAndTime(wps, vels, accs, time_assign);
  ref_traj_planner_ptr->tryQPCloseForm();
  Eigen::MatrixXd coeff;
  ref_traj_planner_ptr->getCoefficient(coeff);
  RRTNodeVector path_node_g2s;
  for (int i = time_assign.rows() - 1; i >= 0; --i) 
  {
    RRTNode node;
    node.tau_from_parent = time_assign(i);
    node.n_order = 5;
    for (int j = 0; j <= 5; ++j) 
    {
      node.x_coeff[j] = coeff(i, 5-j);
      node.y_coeff[j] = coeff(i, 5-j+6);
      node.z_coeff[j] = coeff(i, 5-j+12);
    }
    path_node_g2s.push_back(node);
  }
  //add start node
  RRTNode node;
  path_node_g2s.push_back(node);
  sendTraj(path_node_g2s);

  // vector<State> vis_x;
  // vector<Control> vis_u;
  // for (int k=0; k<path_node_g2s.size()-1; ++k)
  // {
  //   int cur_order = path_node_g2s[k].n_order;
  //   double d_t = 0.015;
  //   int n = floor(path_node_g2s[k].tau_from_parent / d_t);
  //   for (int i = 0; i <= n; ++i) {
  //     double t1 = d_t*i;
  //     Eigen::Vector3d pos, vel, acc;
  //     calPVAFromCoeff(pos, vel, acc, path_node_g2s[k].x_coeff, path_node_g2s[k].y_coeff, path_node_g2s[k].z_coeff, t1, cur_order);
  //     State x;
  //     Control u;
  //     x << pos(0), pos(1), pos(2), vel(0), vel(1), vel(2);
  //     u << acc(0), acc(1), acc(2);
  //     vis_x.push_back(x);
  //     vis_u.push_back(u);
  //   }
  // }
  // visualizeRefTraj(vis_x, vis_u, ros::Time::now());

  // quadrotor_msgs::PositionCommand goal;
  // goal.header.frame_id = "map";
  // ros::Rate d_t(30);
  // for (int id=0; id<waypoint_num; ++id)
  // {
  //   Vector3d curr_target_pos = waypoints[id];
  //   Vector3d curr_target_vel(0.0, 0.0, 0.0);
  //   Vector3d curr_target_acc(0.0, 0.0, 0.0);
  //   calPVAFromCoeff(curr_target_pos, curr_target_vel, curr_target_acc, 
  //                   path_node_g2s[waypoint_num - id - 1].x_coeff, 
  //                   path_node_g2s[waypoint_num - id - 1].y_coeff, 
  //                   path_node_g2s[waypoint_num - id - 1].z_coeff, 
  //                   time_assign[id], path_node_g2s[waypoint_num - id - 1].n_order);
  //   bool pubed = false;
  //   while (ros::ok())
  //   {  
  //     d_t.sleep();
  //     ros::spinOnce();
  //     if ((curr_pos - curr_target_pos).norm() > 1.0 && !pubed)
  //     {
  //       goal.header.stamp = ros::Time::now();
  //       goal.header.seq = id + 1;
  //       goal.position.x = curr_target_pos(0);
  //       goal.position.y = curr_target_pos(1);
  //       goal.position.z = curr_target_pos(2);
  //       goal.velocity.x = 0.0;//curr_target_vel(0);
  //       goal.velocity.y = 0.0;//curr_target_vel(1);
  //       goal.velocity.z = 0.0;//curr_target_vel(2);
  //       goal.acceleration.x = 0.0;//curr_target_acc(0);
  //       goal.acceleration.y = 0.0;//curr_target_acc(1);
  //       goal.acceleration.z = 0.0;//curr_target_acc(2);
  //       goal_pub.publish(goal);
  //       pubed = true;
  //       cout << "pub goal! " << goal.position.x << ", " << goal.position.y << ", " << goal.position.z << endl;
  //       cout << "pub vel: " << goal.velocity.x << ", " << goal.velocity.y << ", " << goal.velocity.z << endl;
  //       cout << "pub acc: " << goal.acceleration.x << ", " << goal.acceleration.y << ", " << goal.acceleration.z << endl;
  //     }
  //     else if ((curr_pos - curr_target_pos).norm() <= 4) 
  //     {
  //       break;
  //     }
  //   }
  // }

  return 0;
}
