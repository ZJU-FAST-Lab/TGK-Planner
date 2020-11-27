#include "occ_grid/occ_map.h"
#include "occ_grid/raycast.h"
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

//for img debug
#include <opencv2/opencv.hpp>

namespace tgk_planner
{
void OccMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  min_pos(0) = max(min_pos(0), min_range_(0));
  min_pos(1) = max(min_pos(1), min_range_(1));
  min_pos(2) = max(min_pos(2), min_range_(2));

  max_pos(0) = min(max_pos(0), max_range_(0));
  max_pos(1) = min(max_pos(1), max_range_(1));
  max_pos(2) = min(max_pos(2), max_range_(2));

  Eigen::Vector3i min_id, max_id;

  posToIndex(min_pos, min_id);
  posToIndex(max_pos - Eigen::Vector3d(resolution_ / 2, resolution_ / 2, resolution_ / 2), max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        occupancy_buffer_[x * grid_size_y_multiply_z_ + y * grid_size_(2) + z] = clamp_min_log_;
      }
}

inline bool OccMap::isInLocalMap(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idx;
  posToIndex(pos, idx);
  return isInLocalMap(idx);
}

inline bool OccMap::isInLocalMap(const Eigen::Vector3i &id)
{
  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min_, min_id);
  posToIndex(local_range_max_, max_id);
  min_id(0) = max(0, min_id(0));
  min_id(1) = max(0, min_id(1));
  min_id(2) = max(0, min_id(2));
  max_id(0) = min(grid_size_[0], max_id(0));
  max_id(1) = min(grid_size_[1], max_id(1));
  max_id(2) = min(grid_size_[2], max_id(2));
  return (((id[0] - min_id[0]) | (max_id[0] - id[0]) | (id[1] - min_id[1]) | (max_id[1] - id[1]) | (id[2] - min_id[2]) | (max_id[2] - id[2])) >= 0);
};

inline bool OccMap::isInMap(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idx;
  posToIndex(pos, idx);
  return isInMap(idx);
}

inline bool OccMap::isInMap(const Eigen::Vector3i &id)
{
  return ((id[0] | (grid_size_[0] - 1 - id[0]) | id[1] | (grid_size_[1] - 1 - id[1]) | id[2]| (grid_size_[2] - 1 - id[2])) >= 0);
};

void OccMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
}

void OccMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
  pos = origin_;
  for (int i = 0; i < 3; ++i)
    pos(i) += (id(i) + 0.5) * resolution_;
}

void OccMap::setOccupancy(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i id;
  posToIndex(pos, id);
  // cout << "id: " << id.transpose() << ", idx: " <<id(0) * grid_size_y_multiply_z_ + id(1) * grid_size_(2) + id(2) << ", is in map? " << isInMap(id) << endl;
  if (!isInMap(id))
    return;

  occupancy_buffer_[id(0) * grid_size_y_multiply_z_ + id(1) * grid_size_(2) + id(2)] = clamp_max_log_;
}

int OccMap::getVoxelState(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id))
    return -1;
  if (!isInLocalMap(id))
    return 0;
  
  // (x, y, z) -> x*ny*nz + y*nz + z
  return occupancy_buffer_[id(0) * grid_size_y_multiply_z_ + id(1) * grid_size_(2) + id(2)] > min_occupancy_log_ ? 1 : 0;
}

int OccMap::getVoxelState(const Eigen::Vector3i &id)
{
  if (!isInMap(id))
    return -1;
  if (!isInLocalMap(id))
    return 0;

  // (x, y, z) -> x*ny*nz + y*nz + z
  return occupancy_buffer_[id(0) * grid_size_y_multiply_z_ + id(1) * grid_size_(2) + id(2)] > min_occupancy_log_ ? 1 : 0;
}

void OccMap::pubPointCloudFromDepth(const std_msgs::Header& header, 
                                    const cv::Mat& depth_img, 
                                    const Eigen::Matrix3d& intrinsic_K, 
                                    const string& camera_name)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ point; 

  for (int v = 0; v < depth_img.rows; v++)
    for (int u = 0; u < depth_img.cols; u++)
    {
      double depth = depth_img.at<u_int16_t>(v, u) / depth_scale_;
      Eigen::Vector3d uvd = Eigen::Vector3d(u, v, 1.0) * depth;
      Eigen::Vector3d xyz = intrinsic_K.inverse() * uvd;
      point.x = xyz(0);
      point.y = xyz(1);
      point.z = xyz(2);
      cloud.points.push_back(point);
    }

  cloud.width = (int)cloud.points.size();
  cloud.height = 1;    //height=1 implies this is not an "ordered" point cloud

  // Convert the cloud to ROS message
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud, output);
  output.header = header;
  output.header.frame_id = camera_name;
  origin_pcl_pub_.publish(output);
}

void OccMap::globalOccVisCallback(const ros::TimerEvent& e)
{
  //for vis
  history_view_cloud_ptr_->points.clear();
  for (int x = 0; x < grid_size_[0]; ++x)
    for (int y = 0; y < grid_size_[1]; ++y)
      for (int z = 0; z < grid_size_[2]; ++z)
      {
        //cout << "p(): " << occupancy_buffer_[x * grid_size_y_multiply_z_ + y * grid_size_(2) + z] << endl;
        if (occupancy_buffer_[x * grid_size_y_multiply_z_ + y * grid_size_(2) + z] > min_occupancy_log_)
        {
          Eigen::Vector3i idx(x,y,z);
          Eigen::Vector3d pos;
          indexToPos(idx, pos);
          pcl::PointXYZ pc(pos[0], pos[1], pos[2]);
          history_view_cloud_ptr_->points.push_back(pc);
        }
      }
  history_view_cloud_ptr_->width = history_view_cloud_ptr_->points.size();
  history_view_cloud_ptr_->height = 1;
  history_view_cloud_ptr_->is_dense = true;
  history_view_cloud_ptr_->header.frame_id = "map";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*history_view_cloud_ptr_, cloud_msg);
  hist_view_cloud_pub_.publish(cloud_msg);
}

void OccMap::localOccVisCallback(const ros::TimerEvent& e)
{
  //for vis
  // ros::Time t_s = ros::Time::now();
  curr_view_cloud_ptr_->points.clear();
  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min_, min_id);
  posToIndex(local_range_max_, max_id);
// 	ROS_INFO_STREAM("local_range_min_: " << local_range_min_.transpose());
// 	ROS_INFO_STREAM("local_range_max_: " << local_range_max_.transpose());
// 	ROS_INFO_STREAM("min_id: " << min_id.transpose());
// 	ROS_INFO_STREAM("max_id: " << max_id.transpose());
	
  min_id(0) = max(0, min_id(0));
  min_id(1) = max(0, min_id(1));
  min_id(2) = max(0, min_id(2));
  max_id(0) = min(grid_size_[0], max_id(0));
  max_id(1) = min(grid_size_[1], max_id(1));
  max_id(2) = min(grid_size_[2], max_id(2));
  for (int x = min_id(0); x < max_id(0); ++x)
    for (int y = min_id(1); y < max_id(1); ++y)
      for (int z = min_id(2); z < max_id(2); ++z)
      {
        if (occupancy_buffer_[x * grid_size_y_multiply_z_ + y * grid_size_(2) + z] > min_occupancy_log_)
        {
          Eigen::Vector3i idx(x,y,z);
          Eigen::Vector3d pos;
          indexToPos(idx, pos);
					// ROS_INFO_STREAM("occupied idx: " << idx.transpose());
					// ROS_INFO_STREAM("occupied pos: " << pos.transpose());
          pcl::PointXYZ pc(pos[0], pos[1], pos[2]);
          curr_view_cloud_ptr_->points.push_back(pc);
        }
      }
  curr_view_cloud_ptr_->width = curr_view_cloud_ptr_->points.size();
  curr_view_cloud_ptr_->height = 1;
  curr_view_cloud_ptr_->is_dense = true;
  curr_view_cloud_ptr_->header.frame_id = "map";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*curr_view_cloud_ptr_, cloud_msg);
  curr_view_cloud_pub_.publish(cloud_msg);
  // ROS_INFO_STREAM("local vis once uses: " << (ros::Time::now() - t_s).toSec());
}


void OccMap::depthOdomCallback(const sensor_msgs::ImageConstPtr& depth_msg, 
                              const nav_msgs::OdometryConstPtr& odom, 
                              const Eigen::Matrix4d& T_ic, 
                              Eigen::Matrix4d& last_T_wc, 
                              cv::Mat& last_depth_image, 
                              const string& camera_name)
{
  { //TF map^ T ego
    static tf2_ros::TransformBroadcaster br_map_ego;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = depth_msg->header.stamp;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "ego";
    transformStamped.transform.translation.x = odom->pose.pose.position.x;
    transformStamped.transform.translation.y = odom->pose.pose.position.y;
    transformStamped.transform.translation.z = odom->pose.pose.position.z;
    transformStamped.transform.rotation.x = odom->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom->pose.pose.orientation.w;
    br_map_ego.sendTransform(transformStamped);
  }

  { //TF imu^ T _camera_i
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = depth_msg->header.stamp;
    static_transformStamped.header.frame_id = "ego";
    static_transformStamped.child_frame_id = camera_name;
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    Eigen::Quaterniond quat(T_ic.block<3,3>(0,0));
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
  }

  /* ---------- get pose ---------- */
  // w, x, y, z -> q0, q1, q2, q3
  Eigen::Matrix3d R_wi = Eigen::Quaterniond(odom->pose.pose.orientation.w, 
                                            odom->pose.pose.orientation.x, 
                                            odom->pose.pose.orientation.y, 
                                            odom->pose.pose.orientation.z).toRotationMatrix();
  Eigen::Matrix4d T_wi;
  T_wi.setZero();
  T_wi(0, 3) = odom->pose.pose.position.x;
  T_wi(1, 3) = odom->pose.pose.position.y;
  T_wi(2, 3) = odom->pose.pose.position.z;
  T_wi(3, 3) = 1.0;
  T_wi.block<3,3>(0,0) = R_wi;
  Eigen::Matrix4d T_wc = T_wi * T_ic;
  Eigen::Vector3d t_wc = T_wc.block<3,1>(0,3);
  local_range_min_ = t_wc - sensor_range_;
	local_range_max_ = t_wc + sensor_range_;

  /* ---------- get depth image ---------- */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, depth_scale_);
  }
  cv_ptr->image.copyTo(depth_image_);

  if (show_raw_depth_)
  {
    pubPointCloudFromDepth(depth_msg->header, depth_image_, K_depth_, camera_name);
  }

  proj_points_cnt_ = 0;
  projectDepthImage(K_depth_, T_wc, depth_image_, last_T_wc, last_depth_image, depth_msg->header.stamp);
  raycastProcess(t_wc);

  local_map_valid_ = true;
  latest_odom_time_ = odom->header.stamp;
  curr_posi_[0] = odom->pose.pose.position.x;
  curr_posi_[1] = odom->pose.pose.position.y;
  curr_posi_[2] = odom->pose.pose.position.z;
  curr_twist_[0] = odom->twist.twist.linear.x;
  curr_twist_[1] = odom->twist.twist.linear.y;
  curr_twist_[2] = odom->twist.twist.linear.z;
  curr_q_.w() = odom->pose.pose.orientation.w;
  curr_q_.x() = odom->pose.pose.orientation.x;
  curr_q_.y() = odom->pose.pose.orientation.y;
  curr_q_.z() = odom->pose.pose.orientation.z;
  have_odom_ = true;
}

void OccMap::projectDepthImage(const Eigen::Matrix3d& K, 
                               const Eigen::Matrix4d& T_wc, const cv::Mat& depth_image, 
                               Eigen::Matrix4d& last_T_wc, cv::Mat& last_depth_image, ros::Time r_s)
{
  int cols = depth_image.cols;
  int rows = depth_image.rows;
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointXYZRGB point; //colored point clouds also have RGB values
  
  double depth;
//   ROS_WARN("project for one rcved image");
  if (!use_shift_filter_)
  {
    for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_)
    {
      for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_)
      {
        depth = depth_image.at<uint16_t>(v, u) / depth_scale_;
				if (isnan(depth) || isinf(depth))
					continue;
			  if (depth < depth_filter_mindist_)
          continue;
				Eigen::Vector3d proj_pt_NED, proj_pt_cam;
        proj_pt_cam(0) = (u - K(0,2)) * depth / K(0,0);
        proj_pt_cam(1) = (v - K(1,2)) * depth / K(1,1);
        proj_pt_cam(2) = depth;
				proj_pt_NED = T_wc.block<3,3>(0,0) * proj_pt_cam + T_wc.block<3,1>(0,3);
        proj_points_[proj_points_cnt_++] = proj_pt_NED;
        // cout << "pt in cam: " << proj_pt_cam.transpose() << ", depth: " << depth << endl;
        // cout << "pt in map: " << proj_pt_NED.transpose() << ", depth: " << depth << endl;
        if (show_filter_proj_depth_)
	      {
          point.x = proj_pt_NED[0];
          point.y = proj_pt_NED[1];
          point.z = proj_pt_NED[2];
          point.r = 255;
          point.g = 0;
          point.b = 0;
          cloud.points.push_back(point);
        }
      }
    }
  }
  /* ---------- use depth filter ---------- */
  else
  {
    if (!has_first_depth_)
      has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_NED, pt_reproj;

      for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_)
      {
        for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_)
        {
          depth = depth_image.at<uint16_t>(v, u) / depth_scale_;
					if (isnan(depth) || isinf(depth))
						continue;
					// points with depth > depth_filter_maxdist_ or < depth_filter_mindist_ are not trusted 
					if (depth < depth_filter_mindist_)
            continue;
					pt_cur(0) = (u - K(0,2)) * depth / K(0,0);
          pt_cur(1) = (v - K(1,2)) * depth / K(1,1);
          pt_cur(2) = depth;
					
          // check consistency
					pt_NED = T_wc.block<3,3>(0,0) * pt_cur + T_wc.block<3,1>(0,3);
          pt_reproj = last_T_wc.block<3,3>(0,0).inverse() * (pt_NED - last_T_wc.block<3,1>(0,3));
          double uu = pt_reproj.x() * K(0,0) / pt_reproj.z() + K(0,2);
          double vv = pt_reproj.y() * K(1,1) / pt_reproj.z() + K(1,2);
          if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
          {
            double drift_dis = fabs(last_depth_image.at<uint16_t>((int)vv, (int)uu) / depth_scale_ - pt_reproj.z());
            //cout << "drift dis: " << drift_dis << endl;
            if (drift_dis < depth_filter_tolerance_)
            {
              proj_points_[proj_points_cnt_++] = pt_NED;
              if (show_filter_proj_depth_)
            	{
                point.x = pt_NED[0];
                point.y = pt_NED[1];
                point.z = pt_NED[2];
                point.r = 255;
                point.g = 0;
                point.b = 0;
                cloud.points.push_back(point);
              }
            }
          }
          else
          {
						// new point
            proj_points_[proj_points_cnt_++] = pt_NED;
            if (show_filter_proj_depth_)
          	{
              point.x = pt_NED[0];
              point.y = pt_NED[1];
              point.z = pt_NED[2];
              point.r = 255;
              point.g = 0;
              point.b = 0;
              cloud.points.push_back(point);
            }
          }
        }
      }
    }
    /* ---------- maintain last ---------- */
    last_T_wc = T_wc;
    last_depth_image = depth_image;
  }


  if (show_filter_proj_depth_)
	{
    cloud.width = (int)cloud.points.size();
    cloud.height = 1;    //height=1 implies this is not an "ordered" point cloud
    // Convert the cloud to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.stamp = r_s;
    output.header.frame_id = "map";
    projected_pc_pub_.publish(output);
  }
}

void OccMap::raycastProcess(const Eigen::Vector3d& t_wc)
{
  if (proj_points_cnt_ == 0)
    return;

  // raycast_num_ = (raycast_num_ + 1) % 100000;
  raycast_num_ += 1;

//   ROS_INFO_STREAM("proj_points_ size: " << proj_points_cnt_);

  /* ---------- iterate projected points ---------- */
  int set_cache_idx;
  for (int i = 0; i < proj_points_cnt_; ++i)
  {
    /* ---------- occupancy of ray end ---------- */
    Eigen::Vector3d pt_w = proj_points_[i];
    double length = (pt_w - t_wc).norm();
// 		ROS_INFO_STREAM("len: " << length);
    if (length < min_ray_length_)
      continue;
    else if (length > max_ray_length_)
    {
      pt_w = (pt_w - t_wc) / length * max_ray_length_ + t_wc;
      set_cache_idx = setCacheOccupancy(pt_w, 0);
    }
    else
      set_cache_idx = setCacheOccupancy(pt_w, 1);

    /* ---------- raycast will ignore close end ray ---------- */
    if (set_cache_idx != INVALID_IDX)
    {
      if (cache_rayend_[set_cache_idx] == raycast_num_)
      {
        continue;
      }
      else
        cache_rayend_[set_cache_idx] = raycast_num_;
    }

    //ray casting backwards from point in world frame to camera pos, 
    //the backwards way skips the overlap grids in each ray end by recording cache_traverse_.
    RayCaster raycaster;
    bool need_ray = raycaster.setInput(pt_w / resolution_, t_wc / resolution_); //(ray start, ray end)
    if (!need_ray)
      continue;
    Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
    Eigen::Vector3d ray_pt;
    if (!raycaster.step(ray_pt)) // skip the ray start point since it's the projected point.
      continue;
    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * resolution_;
      set_cache_idx = setCacheOccupancy(tmp, 0);
      if (set_cache_idx != INVALID_IDX)
      {
        //skip overlap grids in each ray
        if (cache_traverse_[set_cache_idx] == raycast_num_)
          break;
        else
          cache_traverse_[set_cache_idx] = raycast_num_;
      }
    }
  }

  /* ---------- update occupancy in batch ---------- */
  while (!cache_voxel_.empty())
  {
    Eigen::Vector3i idx = cache_voxel_.front();
    int idx_ctns = idx(0) * grid_size_y_multiply_z_ + idx(1) * grid_size_(2) + idx(2);
    cache_voxel_.pop();

    double log_odds_update =
        cache_hit_[idx_ctns] >= cache_all_[idx_ctns] - cache_hit_[idx_ctns] ? prob_hit_log_ : prob_miss_log_;
    cache_hit_[idx_ctns] = cache_all_[idx_ctns] = 0;

    if ((log_odds_update >= 0 && occupancy_buffer_[idx_ctns] >= clamp_max_log_) ||
        (log_odds_update <= 0 && occupancy_buffer_[idx_ctns] <= clamp_min_log_))
      continue;

    // Eigen::Vector3i min_id, max_id;
    // posToIndex(local_range_min_, min_id);
    // posToIndex(local_range_max_, max_id);
    // bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) && idx(1) <= max_id(1) &&
    //                 idx(2) >= min_id(2) && idx(2) <= max_id(2);
    // if (!in_local)
    // {
    //   occupancy_buffer_[idx_ctns] = clamp_min_log_;
    // }

    occupancy_buffer_[idx_ctns] =
        std::min(std::max(occupancy_buffer_[idx_ctns] + log_odds_update, clamp_min_log_), clamp_max_log_);
  }
}

int OccMap::setCacheOccupancy(const Eigen::Vector3d &pos, int occ)
{
  if (occ != 1 && occ != 0)
  {
    return INVALID_IDX;
  }

  Eigen::Vector3i id;
  posToIndex(pos, id);

  if (!isInMap(id))
  {
    return INVALID_IDX;
  }

  int idx_ctns = id(0) * grid_size_y_multiply_z_ + id(1) * grid_size_(2) + id(2);

  cache_all_[idx_ctns] += 1;

  if (cache_all_[idx_ctns] == 1)
  {
    cache_voxel_.push(id);
  }

  if (occ == 1)
    cache_hit_[idx_ctns] += 1;

  return idx_ctns;
}


void OccMap::indepOdomCallback(const nav_msgs::OdometryConstPtr& odom)
{
	latest_odom_time_ = odom->header.stamp;
	curr_posi_[0] = odom->pose.pose.position.x;
  curr_posi_[1] = odom->pose.pose.position.y;
  curr_posi_[2] = odom->pose.pose.position.z;
  curr_twist_[0] = odom->twist.twist.linear.x;
  curr_twist_[1] = odom->twist.twist.linear.y;
  curr_twist_[2] = odom->twist.twist.linear.z;
  curr_q_.w() = odom->pose.pose.orientation.w;
  curr_q_.x() = odom->pose.pose.orientation.x;
  curr_q_.y() = odom->pose.pose.orientation.y;
  curr_q_.z() = odom->pose.pose.orientation.z;
	have_odom_ = true;
  local_range_min_ = curr_posi_ - sensor_range_;
  local_range_max_ = curr_posi_ + sensor_range_;

  { //TF map^ T ego
    static tf2_ros::TransformBroadcaster br_map_ego;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = odom->header.stamp;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "ego";
    transformStamped.transform.translation.x = odom->pose.pose.position.x;
    transformStamped.transform.translation.y = odom->pose.pose.position.y;
    transformStamped.transform.translation.z = odom->pose.pose.position.z;
    transformStamped.transform.rotation.x = odom->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom->pose.pose.orientation.w;
    br_map_ego.sendTransform(transformStamped);
  }
}

void OccMap::globalCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if(!use_global_map_ || has_global_cloud_)
    return;

	pcl::PointCloud<pcl::PointXYZ> global_cloud;
  pcl::fromROSMsg(*msg, global_cloud);
  global_map_valid_ = true;

  if (global_cloud.points.size() == 0)
    return;

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d;
  for (size_t i = 0; i < global_cloud.points.size(); ++i)
  {
    pt = global_cloud.points[i];
    p3d(0) = pt.x; p3d(1) = pt.y; p3d(2) = pt.z;
    this->setOccupancy(p3d);
  }
  has_global_cloud_ = true;
  global_cloud_sub_.shutdown();
}

void OccMap::init(const ros::NodeHandle& nh)
{
  node_ = nh;
  /* ---------- param ---------- */
  node_.param("occ_map/origin_x", origin_(0), -20.0);
  node_.param("occ_map/origin_y", origin_(1), -20.0);
  node_.param("occ_map/origin_z", origin_(2), 0.0);
  node_.param("occ_map/map_size_x", map_size_(0), 40.0);
  node_.param("occ_map/map_size_y", map_size_(1), 40.0);
  node_.param("occ_map/map_size_z", map_size_(2), 5.0);
  node_.param("occ_map/local_radius_x", sensor_range_(0), -1.0);
  node_.param("occ_map/local_radius_y", sensor_range_(1), -1.0);
  node_.param("occ_map/local_radius_z", sensor_range_(2), -1.0);

  node_.param("occ_map/resolution", resolution_, 0.2);
  node_.param("occ_map/use_global_map", use_global_map_, false);

	node_.param("occ_map/depth_scale", depth_scale_, -1.0);
  node_.param("occ_map/use_shift_filter", use_shift_filter_, true);
  node_.param("occ_map/depth_filter_tolerance", depth_filter_tolerance_, -1.0);
  node_.param("occ_map/depth_filter_maxdist", depth_filter_maxdist_, -1.0);
  node_.param("occ_map/depth_filter_mindist", depth_filter_mindist_, -1.0);
  node_.param("occ_map/depth_filter_margin", depth_filter_margin_, -1);
  node_.param("occ_map/skip_pixel", skip_pixel_, -1);
  node_.param("occ_map/show_raw_depth", show_raw_depth_, false);
  node_.param("occ_map/show_filter_proj_depth", show_filter_proj_depth_, false);
  
  node_.param("occ_map/min_ray_length", min_ray_length_, -0.1);
  node_.param("occ_map/max_ray_length", max_ray_length_, -0.1);

  node_.param("occ_map/prob_hit_log", prob_hit_log_, 0.70);
  node_.param("occ_map/prob_miss_log", prob_miss_log_, 0.35);
  node_.param("occ_map/clamp_min_log", clamp_min_log_, 0.12);
  node_.param("occ_map/clamp_max_log", clamp_max_log_, 0.97);
  node_.param("occ_map/min_occupancy_log", min_occupancy_log_, 0.80);

  node_.param("occ_map/fx", fx_, -1.0);
  node_.param("occ_map/fy", fy_, -1.0);
  node_.param("occ_map/cx", cx_, -1.0);
  node_.param("occ_map/cy", cy_, -1.0);
  node_.param("occ_map/rows", rows_, 480);
  node_.param("occ_map/cols", cols_, 320);

  cout << "use_shift_filter_: " << use_shift_filter_ << endl;
  cout << "map size: " << map_size_.transpose() << endl;
  cout << "resolution: " << resolution_ << endl;

  cout << "hit: " << prob_hit_log_ << endl;
  cout << "miss: " << prob_miss_log_ << endl;
  cout << "min: " << clamp_min_log_ << endl;
  cout << "max: " << clamp_max_log_ << endl;
  cout << "thresh: " << min_occupancy_log_ << endl;
  cout << "skip: " << skip_pixel_ << endl;
	cout << "sensor_range: " << sensor_range_.transpose() << endl;

  /* ---------- setting ---------- */
  have_odom_ = false;
  global_map_valid_ = false;
  local_map_valid_ = false;
  has_global_cloud_ = false;
  has_first_depth_ = false;

  resolution_inv_ = 1 / resolution_;
  for (int i = 0; i < 3; ++i)
    grid_size_(i) = ceil(map_size_(i) / resolution_);
  
  curr_view_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  history_view_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  T_ic0_ << 0.0, 0.0, 1.0, 0.0,
           -1.0, 0.0, 0.0, 0.0,
            0.0,-1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
								
  cout << "origin_: " << origin_.transpose() << endl;
  min_range_ = origin_;
  max_range_ = origin_ + map_size_;
  cout << "min_range_: " << min_range_.transpose() << endl;
  cout << "max_range_: " << max_range_.transpose() << endl;

  //init proj_points_ buffer
  proj_points_.resize(rows_ * cols_ / skip_pixel_ / skip_pixel_);
  
  K_depth_.setZero();
  K_depth_(0, 0) = fx_; //fx
  K_depth_(1, 1) = fy_; //fy
  K_depth_(0, 2) = cx_; //cx
  K_depth_(1, 2) = cy_; //cy
  K_depth_(2, 2) = 1.0;
  cout << "intrinsic: " << K_depth_ << endl;
								
  // initialize size of buffer
  grid_size_y_multiply_z_ = grid_size_(1) * grid_size_(2);
  int buffer_size = grid_size_(0) * grid_size_y_multiply_z_;
  cout << "buffer size: " << buffer_size << endl;
  occupancy_buffer_.resize(buffer_size);

  cache_all_.resize(buffer_size);
  cache_hit_.resize(buffer_size);

  cache_rayend_.resize(buffer_size);
  cache_traverse_.resize(buffer_size);
  raycast_num_ = 0;
  
  proj_points_cnt_ = 0;

  fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), clamp_min_log_);

  fill(cache_all_.begin(), cache_all_.end(), 0);
  fill(cache_hit_.begin(), cache_hit_.end(), 0);

  fill(cache_rayend_.begin(), cache_rayend_.end(), -1);
  fill(cache_traverse_.begin(), cache_traverse_.end(), -1);

  //set x-y boundary occ
  // for (double cx = min_range_[0]+resolution_/2; cx <= max_range_[0]-resolution_/2; cx += resolution_)
  //   for (double cz = min_range_[2]+resolution_/2; cz <= max_range_[2]-resolution_/2; cz += resolution_)
  //   {
  //     this->setOccupancy(Eigen::Vector3d(cx, min_range_[1]+resolution_/2, cz));
  //     this->setOccupancy(Eigen::Vector3d(cx, max_range_[1]-resolution_/2, cz));
  //   }
  // for (double cy = min_range_[1]+resolution_/2; cy <= max_range_[1]-resolution_/2; cy += resolution_)
  //   for (double cz = min_range_[2]+resolution_/2; cz <= max_range_[2]-resolution_/2; cz += resolution_)
  //   {
  //     this->setOccupancy(Eigen::Vector3d(min_range_[0]+resolution_/2, cy, cz));
  //     this->setOccupancy(Eigen::Vector3d(max_range_[0]-resolution_/2, cy, cz));
  //   }

    /* ---------- sub and pub ---------- */
	if (!use_global_map_)
	{
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/depth_topic", 1, ros::TransportHints().tcpNoDelay()));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/odom_topic", 1, ros::TransportHints().tcpNoDelay()));
    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&OccMap::depthOdomCallback, this, _1, _2, T_ic0_, last_T_wc0_, last_depth0_image_, "camera_front"));
    //global_occ_vis_timer_ = node_.createTimer(ros::Duration(5), &OccMap::globalOccVisCallback, this);
    local_occ_vis_timer_ = node_.createTimer(ros::Duration(0.3), &OccMap::localOccVisCallback, this);
  }
	else
	{
    indep_odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/odom_topic", 10, &OccMap::indepOdomCallback, this, ros::TransportHints().tcpNoDelay());
		// global_occ_vis_timer_ = node_.createTimer(ros::Duration(5), &OccMap::globalOccVisCallback, this);
    local_occ_vis_timer_ = node_.createTimer(ros::Duration(0.3), &OccMap::localOccVisCallback, this);
	}
  curr_view_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/occ_map/local_view_cloud", 1);
  hist_view_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/occ_map/history_view_cloud", 1);
	global_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/global_cloud", 1, &OccMap::globalCloudCallback, this);
	origin_pcl_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/occ_map/raw_pcl", 1);
  projected_pc_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/occ_map/filtered_pcl", 1);
}

}  // namespace tgk_planner
