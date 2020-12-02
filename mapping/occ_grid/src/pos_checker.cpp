#include "occ_grid/pos_checker.h"

namespace tgk_planner
{

bool PosChecker::validatePosSurround(const Vector3d &pos)
{
  int x_size = ceil(copter_diag_len_ / 2.0 / resolution_);
  int y_size = ceil(copter_diag_len_ / 2.0 / resolution_);
  int z_size = ceil(vtc_safe_radius_ / resolution_);
  Vector3d grid(pos);
  for (int i = -x_size; i <= x_size; ++i)
    for (int j = -y_size; j <= y_size; ++j)
      for (int k = -z_size; k <= z_size; ++k)
      {
        grid = pos + Vector3d(i, j, k) * resolution_;
        if (occ_map_->getVoxelState(grid) != 0)
        {
          return false;
        }
      }
  return true;
}

void PosChecker::getCheckPos(const Vector3d &pos, const Vector3d &vel,
                             const Vector3d &acc, vector<Vector3d> &grids,
                             double hor_radius, double ver_radius)
{
  Eigen::Vector3d cw_edge_pos, ccw_edge_pos;
  Eigen::Vector2d vel_hor, cw_radius_vec;
  cw_edge_pos[2] = pos[2];
  ccw_edge_pos[2] = pos[2];
  vel_hor[0] = vel[0];
  vel_hor[1] = vel[1];
  double v_hor_norm = vel_hor.norm();
  if (v_hor_norm < 1e-4)
  {
    vel_hor[0] = 1;
    vel_hor[1] = 1;
  }
  Eigen::Matrix2d r_m;
  r_m << 0, 1,
      -1, 0;
  cw_radius_vec = r_m * vel_hor;
  cw_radius_vec = cw_radius_vec.normalized() * hor_radius;
  cw_edge_pos.head(2) = pos.head(2) + cw_radius_vec;
  ccw_edge_pos.head(2) = pos.head(2) - cw_radius_vec;
  //add horizontal vox;
  getlineGrids(cw_edge_pos, ccw_edge_pos, grids);
  Eigen::Vector3d vertical_up(pos), vertical_down(pos);
  vertical_up(2) += ver_radius;
  vertical_down(2) -= ver_radius;
  //add veltical vox;
  getlineGrids(vertical_up, vertical_down, grids);
}

void PosChecker::getlineGrids(const Vector3d &s_p, const Vector3d &e_p, vector<Vector3d> &grids)
{
  RayCaster raycaster;
  Eigen::Vector3d ray_pt;
  Eigen::Vector3d start = s_p / resolution_, end = e_p / resolution_;
  bool need_ray = raycaster.setInput(start, end);
  if (need_ray)
  {
    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt)*resolution_;
      tmp[0] += resolution_ / 2.0;
      tmp[1] += resolution_ / 2.0;
      tmp[2] += resolution_ / 2.0;
      grids.push_back(tmp);
    }
  }

  //check end
  Eigen::Vector3d end_idx;
  end_idx[0] = std::floor(end.x());
  end_idx[1] = std::floor(end.y());
  end_idx[2] = std::floor(end.z());

  ray_pt[0] = (double)end_idx[0];
  ray_pt[1] = (double)end_idx[1];
  ray_pt[2] = (double)end_idx[2];
  Eigen::Vector3d tmp = (ray_pt)*resolution_;
  tmp[0] += resolution_ / 2.0;
  tmp[1] += resolution_ / 2.0;
  tmp[2] += resolution_ / 2.0;
  grids.push_back(tmp);
}

//if state pos valid, return true
bool PosChecker::checkState(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc)
{
  if (inflate_)
  {
    vector<Vector3d> line_grids;
    getCheckPos(pos, vel, acc, line_grids, hrz_safe_radius_, vtc_safe_radius_);
    for (const Vector3d &grid : line_grids)
    {
      if (occ_map_->getVoxelState(grid) != 0)
      {
        //cout << "collision: " << grid.transpose() << endl;
        return false;
      }
    }
    return true;
  }
  else
  {
    if (occ_map_->getVoxelState(pos) != 0)
    {
      // cout << "collision: "<< pos.transpose() << endl;
      return false;
    }
    return true;
  }
};

bool PosChecker::checkPolySeg(const Piece &seg)
{
  double tau = seg.getDuration();
  for (double t = 0.0; t <= tau; t += dt_)
  {
    Vector3d pos, vel, acc;
    pos = seg.getPos(t);
    vel = seg.getVel(t);
    acc = seg.getAcc(t);
    if (!checkState(pos, vel, acc))
      return false;
      
    // check curvature, ugly 
    if (t > 0.3 && t < tau - 0.3)
    {
      double tmp = vel.norm() * vel.norm() * vel.norm();
      double k = (vel.cross(acc)).norm() / tmp;
      if (k >= 8)
        return false;
    }
  }
  return true;
};

bool PosChecker::checkPolySeg(const Piece &seg, double t_s, double t_e)
{
  double tau = seg.getDuration();
  if (t_s < -FLT_EPSILON || t_e > tau + FLT_EPSILON)
  {
    ROS_WARN_STREAM("Check time violates duration, tau: " << tau << ", t_s: " << t_s << ", t_e: " << t_e);
    return false;
  }
  for (double t = t_s; t <= t_e; t += dt_)
  {
    Vector3d pos, vel, acc;
    pos = seg.getPos(t);
    vel = seg.getVel(t);
    acc = seg.getAcc(t);
    if (!checkState(pos, vel, acc))
      return false;
    // if (t > 0.3 && t < t_e - 0.3)
    // {
    //   if (!curvatureValid(vel, acc))
    //     return false;
    // }
  }
  return true;
};

bool PosChecker::checkPolySeg(const Piece &seg, vector<pair<Vector3d, Vector3d>> &traversal_lines)
{
  double tau = seg.getDuration();
  pair<Vector3d, Vector3d> line;
  bool is_valid(true);
  bool result(true);
  bool zigzag(false);
  Vector3d last_pos = seg.getPos(0.0);

  for (double t = 0.0; t <= tau; t += dt_)
  {
    Eigen::Vector3d pos, vel, acc;
    pos = seg.getPos(t);
    vel = seg.getVel(t);
    acc = seg.getAcc(t);
    if (!zigzag)
    {
      if (t > 0.3 && t < tau - 0.3)
      {
        double tmp = vel.norm() * vel.norm() * vel.norm();
        double k = (vel.cross(acc)).norm() / tmp;
        if (k >= 6)
        {
          line.first = pos;
          line.second = Eigen::Vector3d(0.0, 0.0, -1.0);
          traversal_lines.push_back(line);
          zigzag = true;
        }
      }
    }

    if (is_valid && !checkState(pos, vel, acc))
    {
      result = false;
      is_valid = false;
      line.first = last_pos;
    }
    else if (!is_valid && checkState(pos, vel, acc))
    {
      is_valid = true;
      line.second = pos;
      traversal_lines.push_back(line);
    }
    last_pos = pos;
  }
  return result;
};

bool PosChecker::checkPolyTraj(const Trajectory &traj, double t_s, double t_e, Vector3d &collide_pos, double &remain_safe_time)
{
  double tau = traj.getTotalDuration();
  if (t_s < -FLT_EPSILON || t_e > tau + FLT_EPSILON)
  {
    ROS_WARN_STREAM("Check time violates duration, tau: " << tau << ", t_s: " << t_s << ", t_e: " << t_e);
    return false;
  }
  for (double t = t_s; t <= t_e; t += dt_)
  {
    Vector3d pos, vel, acc;
    pos = traj.getPos(t);
    vel = traj.getVel(t);
    if (!checkState(pos, vel, acc))
    {
      remain_safe_time = t - t_s;
      collide_pos = pos;
      return false;
    }
  }
  remain_safe_time = t_e - t_s;
  return true;
};

bool PosChecker::checkPolyTraj(const Trajectory &traj)
{
  double tau = traj.getTotalDuration();
  for (double t = 0.0; t <= tau; t += dt_)
  {
    Vector3d pos, vel, acc;
    pos = traj.getPos(t);
    vel = traj.getVel(t);
    if (!checkState(pos, vel, acc))
    {
      return false;
    }
  }
  return true;
};

} // namespace tgk_planner
