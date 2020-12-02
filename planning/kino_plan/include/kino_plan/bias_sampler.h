#ifndef _BIAS_SAMPLER_
#define _BIAS_SAMPLER_

#include "occ_grid/pos_checker.h"
#include "node_utils.h"
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <random>

using std::vector;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace tgk_planner
{

class BiasSampler
{
public:
  BiasSampler(const ros::NodeHandle nh)
  {
    nh.param("sampler/vel_mag_mean", vel_mag_mean_, 0.0);
    nh.param("sampler/pos_hrz_var", pos_hrz_var_, 0.0);
    nh.param("sampler/pos_vtc_var", pos_vtc_var_, 0.0);
    nh.param("sampler/vel_mag_var", vel_mag_var_, 0.0);
    nh.param("sampler/vel_dir_var", vel_dir_var_, 0.0);
    nh.param("sampler/resolution", resolution_, 0.0);
    ROS_WARN_STREAM("[sampler] param: vel_mag_mean: " << vel_mag_mean_);
    ROS_WARN_STREAM("[sampler] param: pos_hrz_var: " << pos_hrz_var_);
    ROS_WARN_STREAM("[sampler] param: pos_vtc_var: " << pos_vtc_var_);
    ROS_WARN_STREAM("[sampler] param: vel_mag_var: " << vel_mag_var_);
    ROS_WARN_STREAM("[sampler] param: vel_dir_var: " << vel_dir_var_);
    ROS_WARN_STREAM("[sampler] param: resolution: " << resolution_);

    std::random_device rd;
    gen_ = std::mt19937_64(rd());
    pos_mean_rand_ = std::uniform_real_distribution<double>(0.0, 100.0);
    pos_hor_rand_ = std::normal_distribution<double>(0.0, pos_hrz_var_);
    pos_ver_rand_ = std::normal_distribution<double>(0.0, pos_vtc_var_);
    vel_mag_rand_ = std::normal_distribution<double>(vel_mag_mean_, vel_mag_var_);
    vel_hor_dir_rand_ = std::normal_distribution<double>(0.0, vel_dir_var_);
  };

  void setPosChecker(const PosChecker::Ptr &checker)
  {
    pos_checker_ = checker;
  };

  void topoSetup(const vector<pair<Vector3d, Vector3d>> &segs, const Vector3d &init_pt, const Vector3d &goal_pt)
  {
    init_pos_ = init_pt;
    goal_pos_ = goal_pt;
    all_corners_.clear();
    unit_tracks_.clear();
    p_head_.clear();
    tracks_.clear();
    rotated_unit_tracks_.clear();
    findSamplingSpace(segs, all_corners_);
    setupRandomSampling(init_pt, goal_pt, all_corners_, unit_tracks_, p_head_, tracks_, rotated_unit_tracks_);
  };

  void getTopo(vector<Vector3d> &p_head, vector<Vector3d> &tracks)
  {
    p_head = p_head_;
    tracks = tracks_;
  }

  bool samplingOnce(int idx, StatePVA &rand_state);

private:
  PosChecker::Ptr pos_checker_;
  vector<Vector3d> unit_tracks_, p_head_, tracks_, rotated_unit_tracks_;
  vector<pair<Vector3d, Vector3d>> all_corners_;
  Vector3d init_pos_, goal_pos_, zigzag_pos_;
  std::mt19937_64 gen_;                  
  std::uniform_real_distribution<double> pos_mean_rand_, seg_rand_;
  std::normal_distribution<double> pos_hor_rand_;
  std::normal_distribution<double> pos_ver_rand_;
  std::normal_distribution<double> vel_hor_dir_rand_;
  std::normal_distribution<double> vel_ver_dir_rand_;
  std::normal_distribution<double> vel_mag_rand_;
  double vel_mag_mean_;
  double pos_hrz_var_, pos_vtc_var_;
  double vel_mag_var_;
  double vel_dir_var_;
  double resolution_;

  void findSamplingSpace(const vector<pair<Vector3d, Vector3d>> &segs,
                         vector<pair<Vector3d, Vector3d>> &all_corners);

  void setupRandomSampling(const Vector3d &init_pt, const Vector3d &goal_pt,
                           const vector<pair<Vector3d, Vector3d>> &all_corners,
                           vector<Vector3d> &unit_tracks,
                           vector<Vector3d> &p_head,
                           vector<Vector3d> &tracks,
                           vector<Vector3d> &rotated_unit_tracks);

  void rotateClockwise2d(double theta, Vector2d &v)
  {
    Matrix2d r_m;
    r_m << cos(theta), sin(theta),
        -sin(theta), cos(theta);
    v = r_m * v;
  };

  void rotateClockwise3d(double theta, Vector3d &v)
  {
    Matrix3d r_m;
    r_m << cos(theta), -sin(theta), 0,
        sin(theta), cos(theta), 0,
        0, 0, 1;
    v = r_m * v;
  };

  Vector3d rotate90Clockwise3d(const Vector3d &v)
  {
    Matrix3d r_m;
    r_m << 0, 1, 0,
        -1, 0, 0,
        0, 0, 1;
    return r_m * v;
  };

  Vector3d rotate90AntiClockwise3d(const Vector3d &v)
  {
    Matrix3d r_m;
    r_m << 0, -1, 0,
        1, 0, 0,
        0, 0, 1;
    return r_m * v;
  };

  Vector2d rotate90Clockwise2d(const Vector2d &v)
  {
    Matrix2d r_m;
    r_m << 0, 1,
        -1, 0;
    return r_m * v;
  };

  Vector2d rotate90AntiClockwise2d(const Vector2d &v)
  {
    Matrix2d r_m;
    r_m << 0, -1,
        1, 0;
    return r_m * v;
  };

};

} // namespace tgk_planner
#endif