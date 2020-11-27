#ifndef _POS_CHECKER_
#define _POS_CHECKER_

#include "occ_grid/occ_map.h"
#include "occ_grid/raycast.h"
#include "poly_traj_utils/traj_utils.hpp"
#include <ros/ros.h>
#include <Eigen/Eigen>

using Eigen::Vector3d;

namespace tgk_planner
{

class PosChecker
{
private:
  OccMap::Ptr occ_map_;
  double hrz_safe_radius_, vtc_safe_radius_;
  double copter_diag_len_;
  double resolution_;
  double dt_;
  bool inflate_;

  void getlineGrids(const Vector3d &s_p, const Vector3d &e_p, vector<Vector3d> &grids);

  bool checkState(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc);

  inline bool curvatureValid(const Vector3d &vel, const Vector3d &acc)
  {
    double tmp = vel.norm() * vel.norm() * vel.norm();
    double k = (vel.cross(acc)).norm() / tmp;
    if (k >= 8)
      return false;
    else
      return true;
  };

public:
  PosChecker(){};

  ~PosChecker(){};

  void init(const ros::NodeHandle &nh)
  {
    nh.param("pos_checker/hrz_safe_radius", hrz_safe_radius_, 0.0);
    nh.param("pos_checker/vtc_safe_radius", vtc_safe_radius_, 0.0);
    nh.param("pos_checker/copter_diag_len", copter_diag_len_, 0.0);
    nh.param("pos_checker/dt", dt_, 0.0);
    nh.param("pos_checker/inflate", inflate_, false);
    ROS_WARN_STREAM("[pos_checker] param: hrz_safe_radius: " << hrz_safe_radius_);
    ROS_WARN_STREAM("[pos_checker] param: vtc_safe_radius: " << vtc_safe_radius_);
    ROS_WARN_STREAM("[pos_checker] param: copter_diag_len: " << copter_diag_len_);
    ROS_WARN_STREAM("[pos_checker] param: dt: " << dt_);
    ROS_WARN_STREAM("[pos_checker] param: inflate: " << inflate_);
  };

  void setMap(const OccMap::Ptr &occ_map)
  {
    occ_map_ = occ_map;
    resolution_ = occ_map_->getResolution();
  };

  int getVoxelState(const Vector3d& pos)
  {
    return occ_map_->getVoxelState(pos);
  };

  ros::Time getLocalTime()
  {
    return occ_map_->getLocalTime();
  };

  bool validatePosSurround(const Vector3d &pos);

  void getCheckPos(const Vector3d &pos, const Vector3d &vel,
                   const Vector3d &acc, vector<Vector3d> &grids,
                   double hor_radius, double ver_radius);

  bool checkPolySeg(const Piece& seg);

  bool checkPolySeg(const Piece& seg, double t_s, double t_e);

  bool checkPolySeg(const Piece &seg, vector<pair<Vector3d, Vector3d>> &traversal_lines);

  bool checkPolyTraj(const Trajectory &traj, double t_s, double t_e, Vector3d &collide_pos, double &remain_safe_time);

  bool checkPolyTraj(const Trajectory &traj);

  typedef shared_ptr<PosChecker> Ptr;
};

} // namespace tgk_planner

#endif