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
#include <cmath>
#include <random>
#include <algorithm>
#include <omp.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
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
  previous_result_t_(Eigen::Isometry3d::Identity()),
  last_best_pt_(cv::Point(0,0)),
  last_best_score_(0.0)
{
  // 模式控制
  this->declare_parameter("use_3d_method", true);
  this->declare_parameter("debug", false);
  this->declare_parameter("map_topic", "map");

  // 基础参数
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("robot_base_frame", "base_link");
  this->declare_parameter("lidar_frame", "laser");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");
  
  // 3D 模式特有参数 (NDT)
  this->declare_parameter("use_ndt", true);
  this->declare_parameter("ndt_resolution", 1.0);
  this->declare_parameter("ndt_num_threads", 4);
  this->declare_parameter("error_threshold", 1.0);
  this->declare_parameter("skip_step", 4);

  // 2D 模式精华参数
  this->declare_parameter("lidar_max_range", 8.0);
  this->declare_parameter("blind_radius", 0.1);
  this->declare_parameter("dist_tolerance", 0.2);
  this->declare_parameter("max_iterations", 40);
  this->declare_parameter("orb_features", 1500);     
  this->declare_parameter("ransac_threshold", 5.0);
  this->declare_parameter("f1_fail", 40.0);           
  this->declare_parameter("stop_search_threshold_f1", 70.0);

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
    global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    register_ = std::make_shared<
      small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

    if (use_ndt_) {
      ndt_omp_ = std::make_shared<pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>();
      ndt_omp_->setResolution(ndt_resolution_);
      ndt_omp_->setNumThreads(ndt_num_threads_);
      ndt_omp_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    }
    loadGlobalMap(prior_pcd_file_);
  } else {
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
    std::chrono::milliseconds(500), 
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), 
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
      if (val > 50 || val < 0) {
        map_img.at<uchar>(v, u) = 255;
      }
    }
  }
  // 重要：ROS 占据网格相对于 OpenCV 是上下颠倒的
  cv::flip(map_img, global_map_2d_, 0); 
  
  // 优化：在此处预先计算全局的距离变换矩阵，避免高频回调中重复计算
  cv::Mat inverted;
  cv::bitwise_not(global_map_2d_, inverted);
  cv::distanceTransform(inverted, map_dist_img_, cv::DIST_L2, 5);

  RCLCPP_INFO(this->get_logger(), "2D地图接收成功，已处理镜像关系并生成距离场.");
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "无法读取 PCD 文件: %s", file_name.c_str());
    return;
  }

  Eigen::Affine3d base_to_lidar;
  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
      base_frame_, lidar_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(2.0));
    base_to_lidar = tf2::transformToEigen(tf_stamped.transform);
  } catch (...) {
    base_to_lidar = Eigen::Affine3d::Identity();
  }
  
  pcl::transformPointCloud(*global_map_, *global_map_, base_to_lidar);

  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (use_ndt_ && ndt_omp_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_target(new pcl::PointCloud<pcl::PointXYZ>());
    ndt_target->points.reserve(target_->size());
    for (const auto& pt : target_->points) {
      ndt_target->points.emplace_back(pt.x, pt.y, pt.z);
    }
    ndt_target->width = target_->width;
    ndt_target->height = target_->height;
    ndt_target->is_dense = target_->is_dense;
    ndt_omp_->setInputTarget(ndt_target);
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

// 射线模拟函数
cv::Mat SmallGicpRelocalizationNode::simulateScan(int cx, int cy, int img_size, int center, double lidar_range_px)
{
  cv::Mat sim_scan = cv::Mat::zeros(img_size, img_size, CV_8UC1);
  for (int angle = 0; angle < 360; angle += 1) {
    double rad = angle * M_PI / 180.0;
    double dx = std::cos(rad);
    double dy = std::sin(rad);

    for (double r = 1.0; r <= lidar_range_px; r += 1.0) {
      int map_x = cx + std::round(r * dx);
      int map_y = cy + std::round(r * dy);

      if (map_x < 0 || map_x >= global_map_2d_.cols || map_y < 0 || map_y >= global_map_2d_.rows) break;

      if (global_map_2d_.at<uchar>(map_y, map_x) > 128) {
        int local_x = center + std::round(r * dx);
        int local_y = center + std::round(r * dy);
        if (local_x >= 0 && local_x < img_size && local_y >= 0 && local_y < img_size) {
          cv::circle(sim_scan, cv::Point(local_x, local_y), 1, cv::Scalar(255), -1);
        }
        break;
      }
    }
  }
  // 核心优化：膨胀模拟图。增加障碍物厚度
  cv::Mat thickened;
  cv::dilate(sim_scan, thickened, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
  return thickened;
}

double SmallGicpRelocalizationNode::getF1Score(const cv::Mat& ref, const cv::Mat& tf) 
{
  cv::Mat intersect;
  cv::bitwise_and(ref, tf, intersect);
  double TP = cv::countNonZero(intersect) + 0.001;
  // 计算平均覆盖率作为 F1 得分
  double total = (cv::countNonZero(ref) + cv::countNonZero(tf)) / 2.0 + 0.001;
  return (TP / total) * 100.0;
}

// -------------------------------------------------------------------------
// OpenCV 2D 重定位核心实现 (适配 3D 点云投影)
// -------------------------------------------------------------------------
void SmallGicpRelocalizationNode::performOpenCVRegistration()
{
  if (!is_lost_) {
    accumulated_cloud_->clear();
    return; // 已经定位成功后不再盲目全局搜索消耗 CPU，直至再次 initialpose 被重置
  }

  auto t_start = std::chrono::high_resolution_clock::now();
  if (global_map_2d_.empty() || accumulated_cloud_->empty() || map_dist_img_.empty()) return;

  double lidar_max_range = this->get_parameter("lidar_max_range").as_double();
  double blind_radius = this->get_parameter("blind_radius").as_double();
  double dist_tolerance = this->get_parameter("dist_tolerance").as_double();
  int max_iterations = this->get_parameter("max_iterations").as_int();
  int orb_features = this->get_parameter("orb_features").as_int();
  double ransac_threshold = this->get_parameter("ransac_threshold").as_double();
  double stop_f1 = this->get_parameter("stop_search_threshold_f1").as_double();
  double f1_fail = this->get_parameter("f1_fail").as_double();

  // 1. 获取 3D 点云中的最小障碍物距离
  double min_dist = 1e6;
  for (const auto& p : accumulated_cloud_->points) {
    double d = std::hypot(p.x, p.y);
    if (d > blind_radius && d < lidar_max_range && d < min_dist) min_dist = d;
  }
  if (min_dist > lidar_max_range) return;

  // 2. 利用预计算好的距离场快速生成候选掩码
  double min_dist_px = min_dist / map_resolution_2d_;
  double tol_px = dist_tolerance / map_resolution_2d_;
  cv::Mat candidates_mask = cv::Mat::zeros(map_dist_img_.size(), CV_8UC1);
  for(int i=0; i<map_dist_img_.rows; ++i) {
    for(int j=0; j<map_dist_img_.cols; ++j) {
      if (std::abs(map_dist_img_.at<float>(i, j) - min_dist_px) < tol_px) {
        candidates_mask.at<uchar>(i, j) = 255;
      }
    }
  }

  // 3. 构建智能采样点集合 (绿点)
  std::vector<cv::Point> samples;
  std::vector<cv::Point> all_pts;
  cv::findNonZero(candidates_mask, all_pts);
  if (all_pts.empty()) return;

  std::shuffle(all_pts.begin(), all_pts.end(), std::default_random_engine(std::random_device{}()));
  
  // 【优化：引入连续性记忆】如果上一次的匹配得分还不错(比如 >15)，说明机器人很可能就在附近。
  // 将上一次的最优位置和其邻域点优先塞入本次采样队列中，打破从零开始的纯随机性。
  if (last_best_score_ > 15.0) {
      samples.push_back(last_best_pt_);
      // 添加 3x3 邻域，应对轻微移动
      int offsets[] = {-2, 0, 2};
      for(int ox : offsets) {
          for(int oy : offsets) {
              if (ox == 0 && oy == 0) continue;
              cv::Point neighbor(last_best_pt_.x + ox, last_best_pt_.y + oy);
              if(neighbor.x >= 0 && neighbor.x < global_map_2d_.cols && 
                 neighbor.y >= 0 && neighbor.y < global_map_2d_.rows) {
                  samples.push_back(neighbor);
              }
          }
      }
  }

  // 用随机点填满剩余迭代次数
  for (size_t i = 0; i < all_pts.size() && samples.size() < (size_t)max_iterations; ++i) {
    samples.push_back(all_pts[i]);
  }

  // 4. 将 3D 点云投影为 2D 扫描图像 (Robot-centric)
  int img_size = std::ceil((lidar_max_range * 2.0) / map_resolution_2d_);
  if (img_size % 2 != 0) img_size++;
  int center = img_size / 2;
  cv::Mat current_scan_img = cv::Mat::zeros(img_size, img_size, CV_8UC1);
  for (const auto& p : accumulated_cloud_->points) {
    if (std::hypot(p.x, p.y) < blind_radius) continue;
    
    int u = center + std::round(p.x / map_resolution_2d_);
    int v = center - std::round(p.y / map_resolution_2d_); 
    
    if (u >= 0 && u < img_size && v >= 0 && v < img_size) {
      cv::circle(current_scan_img, cv::Point(u, v), 1, cv::Scalar(255), -1);
    }
  }

  cv::dilate(current_scan_img, current_scan_img, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

  // 5. 提前提取真实扫描的特征点
  auto orb_main = cv::ORB::create(orb_features);
  std::vector<cv::KeyPoint> kp_real;
  cv::Mat des_real;
  orb_main->detectAndCompute(current_scan_img, cv::noArray(), kp_real, des_real);
  if (des_real.empty()) return;

  double best_score = -1.0;
  cv::Point best_sample;
  cv::Mat best_M, best_sim;
  double lidar_px = lidar_max_range / map_resolution_2d_;
  bool found_sufficient = false;

  // 【优化：引入 OpenMP 多线程加速】将原本单核死循环匹配分配到多个线程
  #pragma omp parallel for num_threads(num_threads_) schedule(dynamic)
  for (int i = 0; i < (int)samples.size(); ++i) {
    // 如果其他线程已经找到了达标的结果，可以直接跳过后续计算
    if (found_sufficient) continue;

    const cv::Point pt = samples[i];
    cv::Mat sim = simulateScan(pt.x, pt.y, img_size, center, lidar_px);
    
    // ORB 特征提取器在多线程中需要独立实例以防状态冲突
    auto orb_local = cv::ORB::create(orb_features);
    std::vector<cv::KeyPoint> kp_sim;
    cv::Mat des_sim;
    orb_local->detectAndCompute(sim, cv::noArray(), kp_sim, des_sim);
    if (des_sim.empty()) continue;

    cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;
    matcher.match(des_real, des_sim, matches);
    
    if (matches.size() < 10) continue;

    std::vector<cv::Point2f> src_pts, dst_pts;
    for (const auto& m : matches) {
      src_pts.push_back(kp_real[m.queryIdx].pt);
      dst_pts.push_back(kp_sim[m.trainIdx].pt);
    }

    cv::Mat inliers;
    cv::Mat M = cv::estimateAffinePartial2D(src_pts, dst_pts, inliers, cv::RANSAC, ransac_threshold);
    if (M.empty() || cv::countNonZero(inliers) < 5) continue;

    cv::Mat warped;
    cv::warpAffine(current_scan_img, warped, M, current_scan_img.size(), cv::INTER_NEAREST);
    double score = getF1Score(sim, warped);

    // 线程安全的变量更新
    #pragma omp critical
    {
      if (score > best_score) {
        best_score = score;
        best_sample = pt;
        best_M = M.clone();
        best_sim = sim.clone();
        if (best_score >= stop_f1) found_sufficient = true;
      }
    }
  }

  // 记录本轮最优结果，留给下一轮循环作为"火种"
  if (best_score > 0) {
      last_best_score_ = best_score;
      last_best_pt_ = best_sample;
  } else {
      last_best_score_ = 0.0;
  }

  // 6. 位姿解算与发布
  bool success = (best_score > f1_fail);
  double final_x = 0, final_y = 0, final_yaw = 0;

  if (success) {
    double cos_a = best_M.at<double>(0, 0);
    double sin_a = best_M.at<double>(1, 0);
    double rot_rad = std::atan2(sin_a, cos_a);

    // 基于仿射矩阵将图像中心映射到全局地图像素坐标
    cv::Mat center_pt = (cv::Mat_<double>(3,1) << center, center, 1.0);
    cv::Mat transformed = best_M * center_pt;
    double robot_px = best_sample.x + (transformed.at<double>(0,0) - center);
    double robot_py = best_sample.y + (transformed.at<double>(1,0) - center);
    
    double robot_py_flipped = (global_map_2d_.rows - 1) - robot_py;

    final_x = map_origin_2d_.position.x + robot_px * map_resolution_2d_;
    final_y = map_origin_2d_.position.y + robot_py_flipped * map_resolution_2d_;
    final_yaw = -rot_rad; // -rot_rad 将匹配出的图像顺时针旋转转为 ROS 世界的逆时针旋转

    Eigen::Isometry3d map_to_lidar = Eigen::Isometry3d::Identity();
    map_to_lidar.translation() << final_x, final_y, 0.0;
    map_to_lidar.linear() = Eigen::AngleAxisd(final_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    try {
      auto odom_to_lidar_tf = tf_buffer_->lookupTransform(odom_frame_, current_scan_frame_id_, tf2::TimePointZero);
      Eigen::Isometry3d odom_to_lidar = tf2::transformToEigen(odom_to_lidar_tf.transform);
      result_t_ = previous_result_t_ = map_to_lidar * odom_to_lidar.inverse();
      is_lost_ = false; // 标记成功，中止无意义的连续重定位置全局搜索
    } catch (...) { success = false; }
  }

  // 7. 调试监控面板 (加固：成败皆渲染)
  if (debug_) {
    auto t_end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    cv::Mat dash = cv::Mat::zeros(600, 1000, CV_8UC3);
    // 左边：地图 + 候选区 + 采样点
    cv::Mat map_view;
    cv::cvtColor(global_map_2d_, map_view, cv::COLOR_GRAY2BGR);
    cv::Mat red_zone = cv::Mat::zeros(map_view.size(), CV_8UC3);
    red_zone.setTo(cv::Scalar(0, 0, 150), candidates_mask);
    cv::addWeighted(map_view, 1.0, red_zone, 0.4, 0.0, map_view);
    for(auto p : samples) cv::circle(map_view, p, 2, cv::Scalar(0, 255, 0), -1);
    if (best_sample.x != 0 || best_sample.y != 0) cv::circle(map_view, best_sample, 6, cv::Scalar(255, 0, 0), 2);
    
    cv::Mat map_resized;
    cv::resize(map_view, map_resized, cv::Size(500, 500));
    map_resized.copyTo(dash(cv::Rect(0, 0, 500, 500)));

    // 右边：对齐视图 (绿色 = 模拟扫描，红色 = 实时 3D 点投影对齐后)
    cv::Mat scan_view = cv::Mat::zeros(img_size, img_size, CV_8UC3);
    if (!best_M.empty() && !best_sim.empty()) {
      cv::Mat warped;
      cv::warpAffine(current_scan_img, warped, best_M, current_scan_img.size());
      std::vector<cv::Mat> channels = {cv::Mat::zeros(warped.size(), CV_8UC1), best_sim, warped};
      cv::merge(channels, scan_view);
    }
    cv::Mat scan_resized;
    cv::resize(scan_view, scan_resized, cv::Size(500, 500));
    scan_resized.copyTo(dash(cv::Rect(500, 0, 500, 500)));

    std::string info = success ? "STATUS: SUCCESS" : "STATUS: SEARCHING...";
    cv::Scalar col = success ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
    cv::putText(dash, info, cv::Point(20, 540), 1, 1.5, col, 2);
    cv::putText(dash, "F1 SCORE: " + std::to_string(best_score), cv::Point(20, 570), 1, 1.2, cv::Scalar(255, 255, 255), 1);
    cv::putText(dash, "TIME: " + std::to_string(ms) + " ms", cv::Point(400, 540), 1, 1.2, cv::Scalar(255, 255, 255), 1);
    if (success) {
        std::string p_str = "X:" + std::to_string(final_x).substr(0,5) + " Y:" + std::to_string(final_y).substr(0,5);
        cv::putText(dash, "POSE: " + p_str, cv::Point(400, 570), 1, 1.2, cv::Scalar(0, 255, 255), 1);
    }

    cv::imshow("OpenCV Relocalization Dashboard", dash);
    cv::waitKey(1);
  }
  accumulated_cloud_->clear();
}

// -------------------------------------------------------------------------
// 原始 3D 模式 (点云对齐)
// -------------------------------------------------------------------------
void SmallGicpRelocalizationNode::performRegistration()
{
  if (accumulated_cloud_->empty()) return;

  if (!use_3d_method_) {
    performOpenCVRegistration();
    return;
  }

  std::chrono::high_resolution_clock::time_point t_start;
  if (debug_) t_start = std::chrono::high_resolution_clock::now();

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);
  
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);
  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) return;

  Eigen::Isometry3d final_guess = previous_result_t_;

  if (use_ndt_ && is_lost_ && ndt_omp_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_source(new pcl::PointCloud<pcl::PointXYZ>());
    ndt_source->points.reserve(source_->size() / skip_step_);
    for (size_t i = 0; i < source_->size(); i += skip_step_) {
        ndt_source->points.emplace_back(source_->points[i].x, source_->points[i].y, source_->points[i].z);
    }
    ndt_omp_->setInputSource(ndt_source);
    pcl::PointCloud<pcl::PointXYZ> unused;
    ndt_omp_->align(unused, previous_result_t_.matrix().cast<float>());
    if (ndt_omp_->hasConverged()) {
        final_guess.matrix() = ndt_omp_->getFinalTransformation().cast<double>();
    }
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  auto result = register_->align(*target_, *source_, *target_tree_, final_guess);

  if (result.converged) {
    result_t_ = previous_result_t_ = result.T_target_source;
    double avg_err = result.error / (double)source_->size();
    is_lost_ = (avg_err > error_threshold_);
  } else {
    is_lost_ = true;
  }

  if (debug_) {
    auto t_end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    RCLCPP_INFO(this->get_logger(), "3D 点云配准完成. 耗时: %.2f ms", ms);
  }
  accumulated_cloud_->clear();
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) return;

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  tf.header.frame_id = map_frame_;
  tf.child_frame_id = odom_frame_;

  const Eigen::Vector3d t = result_t_.translation();
  const Eigen::Quaterniond q(result_t_.rotation());

  tf.transform.translation.x = t.x();
  tf.transform.translation.y = t.y();
  tf.transform.translation.z = t.z();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(tf);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "收到 InitialPose，重置状态为 LOST 以触发 OpenCV 全局搜索.");
  Eigen::Isometry3d m_to_b = Eigen::Isometry3d::Identity();
  m_to_b.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  m_to_b.linear() = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();

  try {
    auto tf = tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d b_to_o = tf2::transformToEigen(tf.transform);
    previous_result_t_ = result_t_ = m_to_b * b_to_o;
    is_lost_ = true; 
    
    // 清除上一次的追踪记忆，迫使算法重新开始大范围搜寻
    last_best_score_ = 0.0;
  } catch (...) {}
}

} // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
