// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"
#include <chrono> 
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  // 模式切换与调试参数
  this->declare_parameter("use_3d_method", true);     // true=原有ndt+gicp, false=OpenCV 2D
  this->declare_parameter("debug", false);            // 无论哪种模式的通用调试开关
  this->declare_parameter("map_topic", "map");        // nav2地图话题

  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");
  
  // NDT 参数
  this->declare_parameter("use_ndt", true);
  this->declare_parameter("ndt_resolution", 1.0);
  this->declare_parameter("ndt_num_threads", 4);
  this->declare_parameter("error_threshold", 1.0);
  this->declare_parameter("skip_step", 4);

  this->get_parameter("use_3d_method", use_3d_method_);
  this->get_parameter("debug", debug_);
  this->get_parameter("map_topic", map_topic_);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);
  
  this->get_parameter("use_ndt", use_ndt_);
  this->get_parameter("ndt_resolution", ndt_resolution_);
  this->get_parameter("ndt_num_threads", ndt_num_threads_);
  this->get_parameter("error_threshold", error_threshold_);
  this->get_parameter("skip_step",skip_step_);

  is_lost_ = true; 
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  if (use_3d_method_) {
    RCLCPP_INFO(this->get_logger(), "已开启 3D 重定位模式 (NDT + small_gicp).");
    global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    register_ = std::make_shared<
      small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

    if (use_ndt_) {
      ndt_omp_ = std::make_shared<pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>();
      ndt_omp_->setResolution(ndt_resolution_);
      ndt_omp_->setNumThreads(ndt_num_threads_);
      ndt_omp_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      RCLCPP_INFO(this->get_logger(), "NDT OMP 已初始化。 分辨率: %.2f", ndt_resolution_);
    }

    loadGlobalMap(prior_pcd_file_);
    
  } else {
    RCLCPP_INFO(this->get_logger(), "已开启 2D 重定位模式 (OpenCV 特征匹配). 正在监听 /%s", map_topic_.c_str());
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local();
    map_qos.reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&SmallGicpRelocalizationNode::mapCallback, this, std::placeholders::_1));
  }

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

void SmallGicpRelocalizationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_resolution_2d_ = msg->info.resolution;
  map_origin_2d_ = msg->info.origin;
  int width = msg->info.width;
  int height = msg->info.height;

  cv::Mat map_img = cv::Mat::zeros(height, width, CV_8UC1);
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      int val = msg->data[v * width + u];
      if (val > 50) {
        map_img.at<uchar>(v, u) = 255;
      }
    }
  }
  // 稍微膨胀地图以便提取更稳定的角点特征，模拟线型地图
  cv::dilate(map_img, global_map_2d_, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
  RCLCPP_INFO(this->get_logger(), "已收到 2D 地图，可用于 OpenCV 重定位。");
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "无法读取 PCD 文件: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "加载了包含 %zu 个点的全局地图", global_map_->points.size());

  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF 查找失败: %s 正在重试...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);

  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (use_ndt_ && ndt_omp_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_target(new pcl::PointCloud<pcl::PointXYZ>());
    ndt_target->resize(target_->size());
    ndt_target->header = target_->header;
    ndt_target->width = target_->width;
    ndt_target->height = target_->height;
    ndt_target->is_dense = target_->is_dense;

    #pragma omp parallel for num_threads(num_threads_)
    for (size_t i = 0; i < target_->size(); i++) {
      ndt_target->points[i].x = target_->points[i].x;
      ndt_target->points[i].y = target_->points[i].y;
      ndt_target->points[i].z = target_->points[i].z;
    }
    
    ndt_omp_->setInputTarget(ndt_target);
    RCLCPP_INFO(this->get_logger(), "NDT 目标地图已设置: %zu 个点", ndt_target->size());
  }
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  *accumulated_cloud_ += *scan;
}

void SmallGicpRelocalizationNode::performOpenCVRegistration()
{
  std::chrono::high_resolution_clock::time_point t_start;
  if (debug_) t_start = std::chrono::high_resolution_clock::now();

  if (global_map_2d_.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[OpenCV 模式] 2D地图为空，等待 /%s", map_topic_.c_str());
    return;
  }

  // 1. 将当前点云投影为2D图像 (以感知范围 30m 左右设定分辨率大小)
  int img_size = 800; 
  int center = img_size / 2;
  cv::Mat scan_img = cv::Mat::zeros(img_size, img_size, CV_8UC1);

  for (const auto& p : accumulated_cloud_->points) {
    int u = center + int(p.x / map_resolution_2d_);
    int v = center + int(p.y / map_resolution_2d_);
    if (u >= 0 && u < img_size && v >= 0 && v < img_size) {
      scan_img.at<uchar>(v, u) = 255;
    }
  }
  // 对雷达数据进行膨胀，使其更像墙壁线条
  cv::dilate(scan_img, scan_img, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

  // 2. OpenCV ORB 特征提取与匹配
  auto orb = cv::ORB::create(1000);
  std::vector<cv::KeyPoint> kp_scan, kp_map;
  cv::Mat des_scan, des_map;

  orb->detectAndCompute(scan_img, cv::noArray(), kp_scan, des_scan);
  orb->detectAndCompute(global_map_2d_, cv::noArray(), kp_map, des_map);

  if (des_scan.empty() || des_map.empty()) {
    if (debug_) RCLCPP_WARN(this->get_logger(), "[OpenCV 调试] ORB: 无法提取特征点。");
    return;
  }

  cv::BFMatcher matcher(cv::NORM_HAMMING, true);
  std::vector<cv::DMatch> matches;
  matcher.match(des_scan, des_map, matches);

  if (matches.size() < 10) {
    if (debug_) RCLCPP_WARN(this->get_logger(), "[OpenCV 调试] 匹配点太少 (%zu).", matches.size());
    return;
  }

  // 按距离排序并取前 N 个最佳匹配
  std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
    return a.distance < b.distance;
  });
  int num_good_matches = std::min((int)matches.size(), 50);

  std::vector<cv::Point2f> src_pts, dst_pts;
  for (int i = 0; i < num_good_matches; i++) {
    src_pts.push_back(kp_scan[matches[i].queryIdx].pt);
    dst_pts.push_back(kp_map[matches[i].trainIdx].pt);
  }

  std::vector<uchar> inliers;
  // 计算仿射变换
  cv::Mat M = cv::estimateAffinePartial2D(src_pts, dst_pts, inliers, cv::RANSAC, 3.0);

  if (M.empty()) {
    if (debug_) RCLCPP_WARN(this->get_logger(), "[OpenCV 调试] 仿射变换计算失败。");
    return;
  }

  // 转换 inliers 为 drawMatches 支持的类型 (uchar -> char)
  std::vector<char> inliers_char(inliers.begin(), inliers.end());

  // 3. 计算位姿
  double center_x_transformed = M.at<double>(0, 0) * center + M.at<double>(0, 1) * center + M.at<double>(0, 2);
  double center_y_transformed = M.at<double>(1, 0) * center + M.at<double>(1, 1) * center + M.at<double>(1, 2);

  double robot_x = map_origin_2d_.position.x + center_x_transformed * map_resolution_2d_;
  double robot_y = map_origin_2d_.position.y + center_y_transformed * map_resolution_2d_;
  double angle = atan2(M.at<double>(1, 0), M.at<double>(0, 0));

  // 4. 计算并更新 T_map_odom
  Eigen::Isometry3d odom_to_lidar = Eigen::Isometry3d::Identity();
  try {
    auto transform = tf_buffer_->lookupTransform(
      odom_frame_, current_scan_frame_id_, tf2::TimePointZero);
    odom_to_lidar = tf2::transformToEigen(transform.transform);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "[OpenCV] TF 错误: %s", ex.what());
    return;
  }

  Eigen::Isometry3d map_to_lidar = Eigen::Isometry3d::Identity();
  map_to_lidar.translation() << robot_x, robot_y, odom_to_lidar.translation().z(); // 保持高度
  map_to_lidar.linear() = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  result_t_ = previous_result_t_ = map_to_lidar * odom_to_lidar.inverse();
  is_lost_ = false;

  // 5. 调试输出
  if (debug_) {
    auto t_end = std::chrono::high_resolution_clock::now();
    double time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    RCLCPP_INFO(this->get_logger(),
      "\n[DEBUG OpenCV] 状态报告:\n"
      "  - 耗时: %.2f ms\n"
      "  - 内点数量: %d / %d\n"
      "  - 雷达位姿: x=%.2f, y=%.2f, yaw=%.2f (度)",
      time_ms, cv::countNonZero(inliers), num_good_matches,
      robot_x, robot_y, angle * 180.0 / M_PI
    );

    cv::Mat img_matches;
    cv::drawMatches(scan_img, kp_scan, global_map_2d_, kp_map,
                    std::vector<cv::DMatch>(matches.begin(), matches.begin() + num_good_matches),
                    img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), inliers_char);
    cv::imwrite("/tmp/opencv_relocalization_debug.png", img_matches);
    RCLCPP_INFO(this->get_logger(), "[DEBUG OpenCV] 匹配可视化已保存至 /tmp/opencv_relocalization_debug.png");
  }
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (accumulated_cloud_->empty()) return;

  // 判断执行哪种模式
  if (!use_3d_method_) {
    performOpenCVRegistration();
    accumulated_cloud_->clear();
    return;
  }

  // 以下为原始 3D (NDT+GICP) 重定位思路
  std::chrono::high_resolution_clock::time_point t_start_total;
  if (debug_) t_start_total = std::chrono::high_resolution_clock::now();

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);
  
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);
  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) return;

  Eigen::Isometry3d final_guess = previous_result_t_;
  double ndt_time_ms = 0.0;
  bool ndt_converged = false;
  double ndt_score = -1.0;

  bool should_run_ndt = use_ndt_ && is_lost_ && ndt_omp_;

  if (should_run_ndt) {
    std::chrono::high_resolution_clock::time_point t_ndt_start;
    if (debug_) t_ndt_start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_source(new pcl::PointCloud<pcl::PointXYZ>());
    ndt_source->reserve(source_->size() / skip_step_);
    for (size_t i = 0; i < source_->size(); i += skip_step_) {
      pcl::PointXYZ p;
      p.x = source_->points[i].x;
      p.y = source_->points[i].y;
      p.z = source_->points[i].z;
      ndt_source->push_back(p);
    }

    ndt_omp_->setInputSource(ndt_source);
    Eigen::Matrix4f ndt_guess_mat = previous_result_t_.matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZ> unused_result;
    
    ndt_omp_->align(unused_result, ndt_guess_mat);

    ndt_converged = ndt_omp_->hasConverged();
    if (ndt_converged) {
      final_guess.matrix() = ndt_omp_->getFinalTransformation().cast<double>();
    }

    if (debug_) {
      auto t_ndt_end = std::chrono::high_resolution_clock::now();
      ndt_time_ms = std::chrono::duration<double, std::milli>(t_ndt_end - t_ndt_start).count();
      ndt_score = ndt_omp_->getFitnessScore();
      if (!ndt_converged) RCLCPP_WARN(this->get_logger(), "[DEBUG 3D] NDT 未收敛.");
    }
  }

  std::chrono::high_resolution_clock::time_point t_gicp_start;
  if (debug_) t_gicp_start = std::chrono::high_resolution_clock::now();

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 15;

  auto result = register_->align(*target_, *source_, *target_tree_, final_guess);

  double gicp_time_ms = 0.0;
  if (debug_) {
    auto t_gicp_end = std::chrono::high_resolution_clock::now();
    gicp_time_ms = std::chrono::duration<double, std::milli>(t_gicp_end - t_gicp_start).count();
  }

  double avg_error = 0.0;

  if (result.converged) {
    result_t_ = previous_result_t_ = result.T_target_source;
    
    if (source_->size() > 0) {
      avg_error = result.error / static_cast<double>(source_->size());
    }

    if (avg_error < error_threshold_) {
        is_lost_ = false; // 稳定模式
    } else {
        is_lost_ = true;  // 丢失模式
        if (debug_) RCLCPP_WARN(this->get_logger(), "追踪不稳定 (误差: %.2f). 启用 NDT 恢复.", avg_error);
    }

  } else {
    is_lost_ = true;
    RCLCPP_WARN(this->get_logger(), "GICP 失败. 重置为丢失 (LOST) 模式.");
  }

  accumulated_cloud_->clear();

  if (debug_) {
    auto t_end_total = std::chrono::high_resolution_clock::now();
    double total_time_ms = std::chrono::duration<double, std::milli>(t_end_total - t_start_total).count();

    RCLCPP_INFO(this->get_logger(), 
      "\n[DEBUG 3D] 状态报告:\n"
      "  - 模式: %s\n" 
      "  - 总耗时: %.2f ms (NDT: %.2f | GICP: %.2f)\n"
      "  - NDT: %s (得分: %.4f)\n"
      "  - GICP: %s (平均误差: %.4f | 迭代次数: %zu)\n"
      "  - 位姿: x=%.2f, y=%.2f",
      should_run_ndt ? "恢复中 (NDT + GICP)" : "追踪中 (仅 GICP)",
      total_time_ms, ndt_time_ms, gicp_time_ms,
      should_run_ndt ? (ndt_converged ? "成功" : "失败") : "跳过", ndt_score,
      result.converged ? "成功" : "失败", avg_error, result.iterations,
      result_t_.translation().x(), result_t_.translation().y()
    );
  }
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "收到初始位姿: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
    is_lost_ = true; 
    RCLCPP_INFO(this->get_logger(), "已接受初始位姿，状态重置为丢失(LOST)以便触发快速对齐.");
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "无法转换初始位姿，从 %s 到 %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)