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
  // 【新增】声明 NDT 参数
  this->declare_parameter("use_ndt", true);          // 默认开启
  this->declare_parameter("ndt_resolution", 1.0);    // 飞虎推荐 1.0m (粗配准网格大一点)
  this->declare_parameter("ndt_num_threads", 4);     // 并行线程数
  this->declare_parameter("debug", false);

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
  // 【新增】获取参数
  this->get_parameter("use_ndt", use_ndt_);
  this->get_parameter("ndt_resolution", ndt_resolution_);
  this->get_parameter("ndt_num_threads", ndt_num_threads_);
  this->get_parameter("debug", debug_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  // 【新增】初始化 NDT OMP 对象
  if (use_ndt_) {
    ndt_omp_ = std::make_shared<pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>();
    ndt_omp_->setResolution(ndt_resolution_);
    ndt_omp_->setNumThreads(ndt_num_threads_);
    // DIRECT7 是 ndt_omp 特有的加速搜索方法，比 KDTREE 快很多
    ndt_omp_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    RCLCPP_INFO(this->get_logger(), "NDT OMP initialized. Resolution: %.2f", ndt_resolution_);
  }


  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

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

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);

  // 1. 准备 GICP Target (你代码里已经有了，保持不动)
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  // 2. 【新增】准备 NDT Target (这是飞虎逻辑的关键补充)
  if (use_ndt_ && ndt_omp_) {
    // NDT 需要 PointXYZ 格式。
    // 为了省事且保持一致，我们直接把 GICP 降采样后的 target_ 转回 PointXYZ 给 NDT 用。
    pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_target(new pcl::PointCloud<pcl::PointXYZ>());

    // pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_target(new pcl::PointCloud<pcl::PointXYZ>());
    
    // 【修改】手动拷贝，避开 PCL 模板检查
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
    RCLCPP_INFO(this->get_logger(), "NDT Target set with %zu points", ndt_target->size());
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

void SmallGicpRelocalizationNode::performRegistration()
{
  if (accumulated_cloud_->empty()) {
    // RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }
  auto t_start_total = std::chrono::high_resolution_clock::now();
  // 1. 预处理当前帧 (Source) - 下采样和协方差估计
  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);
  
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);
  
  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  // ==========================================
  // 【核心修改】飞虎战队思路：Coarse (NDT) -> Fine (GICP)
  // ==========================================
  
  // 定义最终传给 GICP 的初值 (默认为上一帧的结果)
  Eigen::Isometry3d final_guess = previous_result_t_;
  double ndt_time_ms = 0.0;
  bool ndt_converged = false;
  double ndt_score = -1.0;
  // --- 阶段一：NDT 粗配准 ---
  if (use_ndt_ && ndt_omp_) {
    auto t_ndt_start = std::chrono::high_resolution_clock::now();
    // 准备 NDT Source (需要 PointXYZ 格式，从 source_ 拷贝坐标)
    pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_source(new pcl::PointCloud<pcl::PointXYZ>());
    
    // 【修改】手动拷贝，避开 PCL 模板检查
    // ndt_source->resize(source_->size());
    // ndt_source->header = source_->header;
    // ndt_source->width = source_->width;
    // ndt_source->height = source_->height;
    // ndt_source->is_dense = source_->is_dense;

    // #pragma omp parallel for num_threads(num_threads_)
    // for (size_t i = 0; i < source_->size(); i++) {
    //   ndt_source->points[i].x = source_->points[i].x;
    //   ndt_source->points[i].y = source_->points[i].y;
    //   ndt_source->points[i].z = source_->points[i].z;
    // }
    // 【更高效的写法】手动“跳着”拷贝点，代替 VoxelGrid (省去计算量)
    // 每 2 个点取 1 个，或者每 3 个点取 1 个
    int skip_step = 2; // 降采样倍率
    ndt_source->reserve(source_->size() / skip_step);
    
    for (size_t i = 0; i < source_->size(); i += skip_step) {
      pcl::PointXYZ p;
      p.x = source_->points[i].x;
      p.y = source_->points[i].y;
      p.z = source_->points[i].z;
      ndt_source->push_back(p);
    }

    ndt_omp_->setInputSource(ndt_source);

    // NDT 需要 float 类型的 Matrix4f
    Eigen::Matrix4f ndt_guess_mat = previous_result_t_.matrix().cast<float>();
    
    pcl::PointCloud<pcl::PointXYZ> unused_result;
    // 执行 NDT 对齐
    ndt_omp_->align(unused_result, ndt_guess_mat);
    auto t_ndt_end = std::chrono::high_resolution_clock::now();
    ndt_time_ms = std::chrono::duration<double, std::milli>(t_ndt_end - t_ndt_start).count();
    
    // 【新增】记录状态
    ndt_converged = ndt_omp_->hasConverged();
    ndt_score = ndt_omp_->getFitnessScore();
    if (ndt_omp_->hasConverged()) {
      // 如果 NDT 收敛，更新 final_guess，这比单纯用上一帧更准
      final_guess.matrix() = ndt_omp_->getFinalTransformation().cast<double>();
      // 可选：RCLCPP_INFO(this->get_logger(), "NDT score: %f", ndt_omp_->getFitnessScore());
    } else {
      RCLCPP_WARN(this->get_logger(), "NDT did not converge. Using previous guess.");
    }
  }
  auto t_gicp_start = std::chrono::high_resolution_clock::now();
  // --- 阶段二：Small GICP 精配准 ---
  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 15; // 稍微增加迭代次数以获得更好精度

  // 【关键】使用 final_guess (可能来自 NDT) 作为 GICP 的初值
  auto result = register_->align(*target_, *source_, *target_tree_, final_guess);
  auto t_gicp_end = std::chrono::high_resolution_clock::now();
  double gicp_time_ms = std::chrono::duration<double, std::milli>(t_gicp_end - t_gicp_start).count();
  // 更新最终结果
  if (result.converged) {
    result_t_ = previous_result_t_ = result.T_target_source;
  } else {
    RCLCPP_WARN(this->get_logger(), "Small GICP did not converge.");
  }

  accumulated_cloud_->clear();

  auto t_end_total = std::chrono::high_resolution_clock::now();
  double total_time_ms = std::chrono::duration<double, std::milli>(t_end_total - t_start_total).count();

  if (debug_) {
    RCLCPP_INFO(this->get_logger(), 
      "\n[DEBUG] Reloc Report:\n"
      "  - Total Time: %.2f ms (NDT: %.2f ms | GICP: %.2f ms)\n"
      "  - NDT Status: %s (Score: %.4f)\n"
      "  - GICP Status: %s (Error: %.4f | Iter: %zu)\n"
      "  - Pose: x=%.2f, y=%.2f",
      total_time_ms, ndt_time_ms, gicp_time_ms,
      ndt_converged ? "OK" : "FAIL", ndt_score,
      result.converged ? "OK" : "FAIL", result.error, result.iterations,
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
  // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
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
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
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
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
