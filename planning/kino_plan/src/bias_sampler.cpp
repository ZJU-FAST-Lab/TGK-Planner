#include "kino_plan/bias_sampler.h"

namespace tgk_planner
{

void BiasSampler::findSamplingSpace(const vector<pair<Vector3d, Vector3d>>& segs, 
                                  vector<pair<Vector3d, Vector3d>>& all_corners)
{
  pair<Vector3d, Vector3d> corner;
  for (const auto& s : segs) 
  {
    // ROS_WARN_STREAM("travel line: " << s.first.transpose() << ", " << s.second.transpose());
    if (s.second[2] == -1.0)
    {
      corner.first = s.first;
      corner.second = s.first;
      all_corners.push_back(corner);
      continue;
    }
    Vector2d middle_v = (s.first.head(2) + s.second.head(2)) / 2;
    Vector2d tail_diff_head = s.second.head(2) - s.first.head(2);
    double l = tail_diff_head.norm();
    Vector2d unit_dir = tail_diff_head / l;
    Vector3d corner_v = (s.first + s.second) / 2;
    Vector3d corner_theta = corner_v;
    // ROS_INFO("corner_m: %lf, %lf, %lf", corner_v[0], corner_v[1], corner_v[2]);
    Vector2d delta_vector;
    
    bool out_of_bound = false;
    double len_delta_theta = 0.0;
    delta_vector << 0.0, 0.0;
    rotateClockwise2d(M_PI/2.0, unit_dir);
    int j = 1;
    double out_of_bound_v = false;
    int voxel_state = -1;
    bool inside_occ = true;
    Vector3d first_free_corner_theta;
    while (1) 
    {
      delta_vector = unit_dir * resolution_ * j;
      ++j;
      corner_theta.head(2) = middle_v + delta_vector;
      voxel_state = pos_checker_->getVoxelState(corner_theta);
      if (voxel_state == -1) 
      {
        out_of_bound_v = true;
        break;
      }
      else if (voxel_state == 0)
      {
        if (inside_occ)
        {
          first_free_corner_theta = corner_theta;
          inside_occ = false;
        }
        else 
        {
          double l = (corner_theta - first_free_corner_theta).norm();
          if (l >= 0.5)
            break;
        }
      }
      else 
      {
        if (!inside_occ)
        {
          inside_occ = true;
        }
      }
    }
    if (delta_vector.norm() >= len_delta_theta) 
    {
      if (out_of_bound_v) 
      {
        out_of_bound = true;
      }
      len_delta_theta = delta_vector.norm();
      corner_v.head(2) = corner_theta.head(2);
    }
    if (out_of_bound) 
    {
      // ROS_INFO("corner1 out of bound, 0.0, 0.0, -1.0");
      corner.first << 0.0,0.0,-1.0;
    }
    else 
    {
      // ROS_INFO("corner1: %lf, %lf, %lf", corner_v[0], corner_v[1], corner_v[2]);
      corner.first = corner_v;   
    }   
    
    {
      corner_v = (s.first + s.second) / 2;
      out_of_bound = false;
      delta_vector << 0.0, 0.0;
      len_delta_theta = 0.0;
      rotateClockwise2d(M_PI, unit_dir);
      int j = 1;
      double out_of_bound_v = false;
      double voxel_state = -1;
      bool inside_occ = true;
      Vector3d first_free_corner_theta;
      while (1) 
      {
        delta_vector = unit_dir * resolution_ * j;
        ++j;
        corner_theta.head(2) = middle_v + delta_vector;
        voxel_state = pos_checker_->getVoxelState(corner_theta);
        if (voxel_state == -1) 
        {
          out_of_bound_v = true;
          break;
        }
        else if (voxel_state == 0)
        {
          if (inside_occ)
          {
            first_free_corner_theta = corner_theta;
            inside_occ = false;
          }
          else 
          {
            double l = (corner_theta - first_free_corner_theta).norm();
            if (l >= 0.5)
              break;
          }
        }
        else 
        {
          if (!inside_occ)
          {
            inside_occ = true;
          }
        }
      }
      if (delta_vector.norm() > len_delta_theta) 
      {
        if (out_of_bound_v) 
        {
          out_of_bound = true;
        }
        len_delta_theta = delta_vector.norm();
          corner_v.head(2) = corner_theta.head(2);
      }
      if (out_of_bound) 
      {
        // ROS_INFO("corner2 out of bound, 0.0, 0.0, -1.0");
        corner.second << 0.0,0.0,-1.0;
      }
      else 
      {
        // ROS_INFO("corner2: %lf, %lf, %lf", corner_v[0], corner_v[1], corner_v[2]);
        corner.second = corner_v;
      }
    }
    all_corners.push_back(corner);
  }
}

void BiasSampler::setupRandomSampling(const Vector3d& init_pt, const Vector3d& goal_pt, 
                            const vector<pair<Vector3d, Vector3d>>& all_corners,
                            vector<Vector3d>& unit_tracks,
                            vector<Vector3d>& p_head,
                            vector<Vector3d>& tracks, 
                            vector<Vector3d>& rotated_unit_tracks)
{
  // all_corners.size() is the same as segs.size()
  Vector3d x0 = init_pt, x1 = goal_pt;
  Vector3d track, unit_track, rotated_unit_track;
  
  //segs is empty
  if (all_corners.size() == 0)
  {
    p_head.push_back(x0);
    track = x1 - x0;
    tracks.push_back(track);
    unit_track = track/track.norm();
    unit_tracks.push_back(unit_track);
    rotated_unit_track = rotate90Clockwise3d(unit_track);
    rotated_unit_tracks.push_back(rotated_unit_track);
    return;
  }
    
  if (all_corners.front().first[2] != -1.0) 
  {
    p_head.push_back(x0);
    track = all_corners.front().first - x0;
    tracks.push_back(track);
    unit_track = track/track.norm();
    unit_tracks.push_back(unit_track);
    rotated_unit_track = rotate90Clockwise3d(unit_track);
    rotated_unit_tracks.push_back(rotated_unit_track);
  }
  if (all_corners.front().second[2] != -1.0) 
  {
    p_head.push_back(x0);
    track = all_corners.front().second - x0;
    tracks.push_back(track);
    unit_track = track/track.norm();
    unit_tracks.push_back(unit_track);
    rotated_unit_track = rotate90Clockwise3d(unit_track);
    rotated_unit_tracks.push_back(rotated_unit_track);
  }
  for (int i=0; i<(int)all_corners.size()-1; ++i) 
  {
    if (all_corners[i+1].first[2] != -1.0 && all_corners[i].first[2] != -1.0) 
    {
      p_head.push_back(all_corners[i].first);
      track = all_corners[i+1].first - all_corners[i].first;
      tracks.push_back(track);
      unit_track = track/track.norm();
      unit_tracks.push_back(unit_track);
      rotated_unit_track = rotate90Clockwise3d(unit_track);
      rotated_unit_tracks.push_back(rotated_unit_track);
    }
    /******** X style bridges  ************/
//     if (all_corners[i+1].first[2] != -1.0 && all_corners[i].second[2] != -1.0) {
//         p_head.push_back(all_corners[i].second);
//         track = all_corners[i+1].first - all_corners[i].second;
//         tracks.push_back(track);
//         unit_track = track/track.norm();
//         unit_tracks.push_back(unit_track);
//         rotated_unit_track = rotate90Clockwise3d(unit_track);
//         rotated_unit_tracks.push_back(rotated_unit_track);
//     }
//     if (all_corners[i+1].second[2] != -1.0 && all_corners[i].first[2] != -1.0) {
//         p_head.push_back(all_corners[i].first);
//         track = all_corners[i+1].second - all_corners[i].first;
//         tracks.push_back(track);
//         unit_track = track/track.norm();
//         unit_tracks.push_back(unit_track);
//         rotated_unit_track = rotate90Clockwise3d(unit_track);
//         rotated_unit_tracks.push_back(rotated_unit_track);
//     }
    /****      ****/
    if (all_corners[i+1].second[2] != -1.0 && all_corners[i].second[2] != -1.0) 
    {
      p_head.push_back(all_corners[i].second);
      track = all_corners[i+1].second - all_corners[i].second;
      tracks.push_back(track);
      unit_track = track/track.norm();
      unit_tracks.push_back(unit_track);
      rotated_unit_track = rotate90Clockwise3d(unit_track);
      rotated_unit_tracks.push_back(rotated_unit_track);
    }
  }
  if (all_corners.back().first[2] != -1.0) 
  {
    p_head.push_back(all_corners.back().first);
    track = x1 - all_corners.back().first;
    tracks.push_back(track);
    unit_track = track/track.norm();
    unit_tracks.push_back(unit_track);
    rotated_unit_track = rotate90Clockwise3d(unit_track);
    rotated_unit_tracks.push_back(rotated_unit_track);
  }
  if (all_corners.back().second[2] != -1.0) 
  {
    p_head.push_back(all_corners.back().second);
    track = x1 - all_corners.back().second;
    tracks.push_back(track);
    unit_track = track/track.norm();
    unit_tracks.push_back(unit_track);
    rotated_unit_track = rotate90Clockwise3d(unit_track);
    rotated_unit_tracks.push_back(rotated_unit_track);
  }
  
  //segs is not empty but all_corners are out of bound
  if (tracks.size() == 0)
  {
    p_head.push_back(x0);
    track = x1 - x0;
    tracks.push_back(track);
    unit_track = track/track.norm();
    unit_tracks.push_back(unit_track);
    rotated_unit_track = rotate90Clockwise3d(unit_track);
    rotated_unit_tracks.push_back(rotated_unit_track);
  }
//     for (int i=0; i<p_head.size(); ++i) {
//         ROS_INFO("p_head:              %lf, %lf, %lf", p_head[i][0],p_head[i][1], p_head[i][2]);
//         ROS_INFO("tracks:              %lf, %lf, %lf", tracks[i][0],tracks[i][1], tracks[i][2]);
//         ROS_INFO("unit_tracks:         %lf, %lf, %lf", unit_tracks[i][0],unit_tracks[i][1], unit_tracks[i][2]);
//         ROS_INFO("rotated_unit_tracks: %lf, %lf, %lf", rotated_unit_tracks[i][0],rotated_unit_tracks[i][1], rotated_unit_tracks[i][2]);
//     }
}

bool BiasSampler::samplingOnce(int idx, StatePVA& rand_state)
{
  idx = idx % tracks_.size();
  double pos_mean = pos_mean_rand_(gen_);
  double pos_hor = pos_hor_rand_(gen_);
  double pos_ver = pos_ver_rand_(gen_);
  double vel_mag = vel_mag_rand_(gen_);
  if (vel_mag > vel_mag_mean_) 
    vel_mag = vel_mag_mean_ - (vel_mag - vel_mag_mean_);
  if (vel_mag <= 0)
    return false;
  double vel_hor_dir = vel_hor_dir_rand_(gen_);
  Vector3d p_m = p_head_[idx] + tracks_[idx] * pos_mean / 100.0;
  Vector3d p = p_m + rotated_unit_tracks_[idx] * pos_hor;
  rand_state[0] = p[0];
  rand_state[1] = p[1];
  rand_state[2] = p_m[2] + pos_ver;
  Vector3d pos;
  pos = rand_state.head(3);
  
  Vector3d v_m = unit_tracks_[idx] * vel_mag;
  rotateClockwise3d(vel_hor_dir, v_m);
  rand_state[3] = v_m[0];
  rand_state[4] = v_m[1];
  rand_state[5] = v_m[2];

  rand_state[6] = 0.0;
  rand_state[7] = 0.0;
  rand_state[8] = 0.0;

  if (!pos_checker_->validatePosSurround(pos)) 
  {
    return false;
  }

  if ((pos - init_pos_).norm() < 1.5 || (pos - goal_pos_).norm() < 1.5)
  {
    return false;
  }

  return true;
}

}