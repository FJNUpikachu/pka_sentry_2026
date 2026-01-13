#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <vector>
#include <iostream>

// ROS & PCL & TF
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/registration/ndt.h" 
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

// OpenCV & Nanoflann
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nanoflann.hpp>

using namespace Eigen;
using namespace nanoflann;
namespace fs = std::filesystem;

// ===================================================================================
// PART 1: 工具类定义 (保持不变)
// ===================================================================================

class TicToc
{
public:
    TicToc() { tic(); }
    void tic() { start = std::chrono::system_clock::now(); }
    void toc( std::string _about_task ) {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        double elapsed_ms = elapsed_seconds.count() * 1000;
        (void)elapsed_ms;
        (void)_about_task;
    }
private:  
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

// Nanoflann Adaptor
template <class VectorOfVectorsType, typename num_t = double, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = size_t>
struct KDTreeVectorOfVectorsAdaptor {
    typedef KDTreeVectorOfVectorsAdaptor<VectorOfVectorsType,num_t,DIM,Distance> self_t;
    typedef typename Distance::template traits<num_t,self_t>::distance_t metric_t;
    typedef nanoflann::KDTreeSingleIndexAdaptor< metric_t,self_t,DIM,IndexType>  index_t;
    index_t* index; 
    const VectorOfVectorsType &m_data;
    KDTreeVectorOfVectorsAdaptor(const int dimensionality, const VectorOfVectorsType &mat, const int leaf_max_size = 10) : m_data(mat) {
        assert(mat.size() != 0 && mat[0].size() != 0);
        const size_t dims = mat[0].size();
        index = new index_t( static_cast<int>(dims), *this, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size ) );
        index->buildIndex();
    }
    ~KDTreeVectorOfVectorsAdaptor() { delete index; }
    inline void query(const num_t *query_point, const size_t num_closest, IndexType *out_indices, num_t *out_distances_sq) const {
        nanoflann::KNNResultSet<num_t,IndexType> resultSet(num_closest);
        resultSet.init(out_indices, out_distances_sq);
        index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
    }
    const self_t & derived() const { return *this; }
    self_t & derived() { return *this; }
    inline size_t kdtree_get_point_count() const { return m_data.size(); }
    inline num_t kdtree_get_pt(const size_t idx, const size_t dim) const { return m_data[idx][dim]; }
    template <class BBOX> bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }
};

// ===================================================================================
// PART 2: ScanContext Manager (保持不变)
// ===================================================================================
namespace small_gicp_relocalization {

using SCPointType = pcl::PointXYZ; 
using KeyMat = std::vector<std::vector<float>>;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;

class SCManager
{
public: 
    SCManager( ) = default;
    const double LIDAR_HEIGHT = 2.0; 
    const int    PC_NUM_RING = 20; 
    const int    PC_NUM_SECTOR = 60; 
    const double PC_MAX_RADIUS = 80.0; 
    const double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
    const double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);
    const int    NUM_EXCLUDE_RECENT = 30; 
    const int    NUM_CANDIDATES_FROM_TREE = 3; 
    const double SEARCH_RATIO = 0.1; 
    const int    TREE_MAKING_PERIOD_ = 10; 
    double sc_dist_thres_ = 0.65; 
    int tree_making_period_conter = 0;
    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;
    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;

    void setDistThreshold(double val) { sc_dist_thres_ = val; }

    float xy2theta( const float & _x, const float & _y ) {
        if ( (_x >= 0) & (_y >= 0)) return (180/M_PI) * std::atan(_y / _x);
        if ( (_x < 0) & (_y >= 0)) return 180 - ( (180/M_PI) * std::atan(_y / (-_x)) );
        if ( (_x < 0) & (_y < 0)) return 180 + ( (180/M_PI) * std::atan(_y / _x) );
        if ( (_x >= 0) & (_y < 0)) return 360 - ( (180/M_PI) * std::atan((-_y) / _x) );
        return 0;
    }

    MatrixXd circshift( MatrixXd &_mat, int _num_shift ) {
        if( _num_shift == 0 ) return _mat;
        MatrixXd shifted_mat = MatrixXd::Zero( _mat.rows(), _mat.cols() );
        for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ ) {
            int new_location = (col_idx + _num_shift) % _mat.cols();
            shifted_mat.col(new_location) = _mat.col(col_idx);
        }
        return shifted_mat;
    }

    std::vector<float> eig2stdvec( MatrixXd _eigmat ) {
        std::vector<float> vec( _eigmat.data(), _eigmat.data() + _eigmat.size() );
        return vec;
    }

    MatrixXd makeScancontext( pcl::PointCloud<SCPointType> & _scan_down ) {
        int num_pts_scan_down = _scan_down.points.size();
        const int NO_POINT = -1000;
        MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
        SCPointType pt;
        float azim_angle, azim_range; 
        int ring_idx, sctor_idx;
        for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++) {
            pt.x = _scan_down.points[pt_idx].x; 
            pt.y = _scan_down.points[pt_idx].y;
            pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; 
            azim_range = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            azim_angle = xy2theta(pt.x, pt.y);
            if( azim_range > PC_MAX_RADIUS ) continue;
            ring_idx = std::max( std::min( PC_NUM_RING, int(std::ceil( (azim_range / PC_MAX_RADIUS) * PC_NUM_RING )) ), 1 );
            sctor_idx = std::max( std::min( PC_NUM_SECTOR, int(std::ceil( (azim_angle / 360.0) * PC_NUM_SECTOR )) ), 1 );
            if ( desc(ring_idx-1, sctor_idx-1) < pt.z ) 
                desc(ring_idx-1, sctor_idx-1) = pt.z; 
        }
        for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
            for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
                if( desc(row_idx, col_idx) == NO_POINT ) desc(row_idx, col_idx) = 0;
        return desc;
    }

    MatrixXd makeRingkeyFromScancontext( Eigen::MatrixXd &_desc ) {
        Eigen::MatrixXd invariant_key(_desc.rows(), 1);
        for ( int row_idx = 0; row_idx < _desc.rows(); row_idx++ ) {
            Eigen::MatrixXd curr_row = _desc.row(row_idx);
            invariant_key(row_idx, 0) = curr_row.mean();
        }
        return invariant_key;
    }

    MatrixXd makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc ) {
        Eigen::MatrixXd variant_key(1, _desc.cols());
        for ( int col_idx = 0; col_idx < _desc.cols(); col_idx++ ) {
            Eigen::MatrixXd curr_col = _desc.col(col_idx);
            variant_key(0, col_idx) = curr_col.mean();
        }
        return variant_key;
    }

    double distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 ) {
        int num_eff_cols = 0; 
        double sum_sector_similarity = 0;
        for ( int col_idx = 0; col_idx < _sc1.cols(); col_idx++ ) {
            VectorXd col_sc1 = _sc1.col(col_idx);
            VectorXd col_sc2 = _sc2.col(col_idx);
            if( (col_sc1.norm() == 0) | (col_sc2.norm() == 0) ) continue; 
            double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());
            sum_sector_similarity = sum_sector_similarity + sector_similarity;
            num_eff_cols = num_eff_cols + 1;
        }
        double sc_sim = sum_sector_similarity / num_eff_cols;
        return 1.0 - sc_sim;
    }

    int fastAlignUsingVkey( MatrixXd & _vkey1, MatrixXd & _vkey2) {
        int argmin_vkey_shift = 0;
        double min_veky_diff_norm = 10000000;
        for ( int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++ ) {
            MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);
            MatrixXd vkey_diff = _vkey1 - vkey2_shifted;
            double cur_diff_norm = vkey_diff.norm();
            if( cur_diff_norm < min_veky_diff_norm ) {
                argmin_vkey_shift = shift_idx;
                min_veky_diff_norm = cur_diff_norm;
            }
        }
        return argmin_vkey_shift;
    }

    std::pair<double, int> distanceBtnScanContext( MatrixXd &_sc1, MatrixXd &_sc2 ) {
        MatrixXd vkey_sc1 = makeSectorkeyFromScancontext( _sc1 );
        MatrixXd vkey_sc2 = makeSectorkeyFromScancontext( _sc2 );
        int argmin_vkey_shift = fastAlignUsingVkey( vkey_sc1, vkey_sc2 );
        const int SEARCH_RADIUS = round( 0.5 * SEARCH_RATIO * _sc1.cols() ); 
        std::vector<int> shift_idx_search_space { argmin_vkey_shift };
        for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ ) {
            shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols() );
            shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols() );
        }
        std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());
        int argmin_shift = 0;
        double min_sc_dist = 10000000;
        for ( int num_shift: shift_idx_search_space ) {
            MatrixXd sc2_shifted = circshift(_sc2, num_shift);
            double cur_sc_dist = distDirectSC( _sc1, sc2_shifted );
            if( cur_sc_dist < min_sc_dist ) {
                argmin_shift = num_shift;
                min_sc_dist = cur_sc_dist;
            }
        }
        return std::make_pair(min_sc_dist, argmin_shift);
    }

    void makeAndSaveScancontextAndKeys( pcl::PointCloud<SCPointType> & _scan_down ) {
        Eigen::MatrixXd sc = makeScancontext(_scan_down); 
        Eigen::MatrixXd ringkey = makeRingkeyFromScancontext( sc );
        Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext( sc );
        std::vector<float> polarcontext_invkey_vec = eig2stdvec( ringkey );
        polarcontexts_.push_back( sc ); 
        polarcontext_invkeys_.push_back( ringkey );
        polarcontext_vkeys_.push_back( sectorkey );
        polarcontext_invkeys_mat_.push_back( polarcontext_invkey_vec );
    }

    std::pair<int, float> detectLoopClosureID ( void ) {
        int loop_id { -1 }; 
        if( (int)polarcontext_invkeys_mat_.size() < NUM_EXCLUDE_RECENT + 1) 
            return std::make_pair(loop_id, 0.0);
        auto curr_key = polarcontext_invkeys_mat_.back(); 
        auto curr_desc = polarcontexts_.back(); 
        if( tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) {
            polarcontext_invkeys_to_search_.clear();
            polarcontext_invkeys_to_search_.assign( polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end() - 1 ); 
            polarcontext_tree_.reset(); 
            polarcontext_tree_ = std::make_unique<InvKeyTree>(PC_NUM_RING, polarcontext_invkeys_to_search_, 10); 
        }
        tree_making_period_conter++; 
        double min_dist = 10000000; 
        int nn_align = 0; 
        int nn_idx = 0; 
        std::vector<size_t> candidate_indexes( NUM_CANDIDATES_FROM_TREE ); 
        std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE ); 
        nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE ); 
        knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] ); 
        polarcontext_tree_->index->findNeighbors( knnsearch_result, &curr_key[0], nanoflann::SearchParams(10) ); 
        for ( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ ) {
            MatrixXd polarcontext_candidate = polarcontexts_[ candidate_indexes[candidate_iter_idx] ]; 
            std::pair<double, int> sc_dist_result = distanceBtnScanContext( curr_desc, polarcontext_candidate ); 
            if( sc_dist_result.first < min_dist ) {
                min_dist = sc_dist_result.first;
                nn_align = sc_dist_result.second;
                nn_idx = candidate_indexes[candidate_iter_idx];
            }
        }
        if( min_dist < sc_dist_thres_ ) loop_id = nn_idx; 
        float yaw_diff_rad = (nn_align * PC_UNIT_SECTORANGLE) * M_PI / 180.0;
        return std::make_pair(loop_id, yaw_diff_rad); 
    }

    void loadSCDFile(const std::string& filepath) {
        FILE *fp = fopen(filepath.c_str(), "rb");
        if(fp == NULL) return;
        Eigen::MatrixXd sc = Eigen::MatrixXd::Zero(PC_NUM_RING, PC_NUM_SECTOR);
        for(int r=0; r<PC_NUM_RING; r++) {
            for(int c=0; c<PC_NUM_SECTOR; c++) {
                float val;
                if(fread(&val, sizeof(float), 1, fp) != 1) break;
                sc(r,c) = double(val);
            }
        }
        fclose(fp);
        Eigen::MatrixXd ringkey = makeRingkeyFromScancontext( sc );
        Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext( sc );
        std::vector<float> polarcontext_invkey_vec = eig2stdvec( ringkey );
        polarcontexts_.push_back( sc ); 
        polarcontext_invkeys_.push_back( ringkey );
        polarcontext_vkeys_.push_back( sectorkey );
        polarcontext_invkeys_mat_.push_back( polarcontext_invkey_vec );
    }
};

// ===================================================================================
// PART 3: SmallGicpRelocalizationNode 实现
// ===================================================================================

static std::string padZeros(int val, int num_digits = 6) {
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
    return out.str();
}

static void loadPosesHelper(const std::string& file_path, Eigen::MatrixXd& poses_mat) {
    std::ifstream infile(file_path);
    if (!infile.is_open()) {
        std::cerr << "Error opening pose file: " << file_path << std::endl;
        return;
    }
    std::vector<std::vector<double>> poses_vec;
    std::string line;
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        std::vector<double> pose;
        double val;
        while (ss >> val) {
            pose.push_back(val);
        }
        poses_vec.push_back(pose);
    }
    infile.close();
    if (poses_vec.empty()) return;
    poses_mat.resize(poses_vec.size(), poses_vec[0].size());
    for (size_t i = 0; i < poses_vec.size(); ++i) {
        for (size_t j = 0; j < poses_vec[0].size(); ++j) {
            poses_mat(i, j) = poses_vec[i][j];
        }
    }
}

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity()),
  is_initialized_(false),
  last_scan_time_(0) // 初始化为 0
{
    RCLCPP_INFO(this->get_logger(), "OpenMP Max Threads: %d", omp_get_max_threads());
    // 参数声明
    // this->declare_parameter("use_sim_time", false);
    this->declare_parameter("num_threads", 4);
    this->declare_parameter("num_neighbors", 20);
    this->declare_parameter("global_leaf_size", 0.25);
    this->declare_parameter("registered_leaf_size", 0.25);
    this->declare_parameter("max_dist_sq", 1.0);
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("robot_base_frame", "base_link");
    this->declare_parameter("lidar_frame", "lidar_link");
    this->declare_parameter("prior_pcd_file", "");
    this->declare_parameter("scd_directory", "");
    this->declare_parameter("pose_file", "");
    this->declare_parameter("enable_sc_relocalization", false); 
    this->declare_parameter("sc_dist_thresh", 0.5);
    this->declare_parameter("ndt_resolution", 1.0);
    this->declare_parameter("ndt_step_size", 0.1);
    this->declare_parameter("ndt_epsilon", 0.01);
    this->declare_parameter("ndt_max_iterations", 35);

    this->get_parameter("use_sim_time", use_sim_time_);
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
    this->get_parameter("scd_directory", scd_directory_);
    this->get_parameter("pose_file", pose_file_);
    this->get_parameter("enable_sc_relocalization", enable_sc_relocalization_);
    this->get_parameter("sc_dist_thresh", sc_dist_thresh_);
    this->get_parameter("ndt_resolution", ndt_resolution_);
    this->get_parameter("ndt_step_size", ndt_step_size_);
    this->get_parameter("ndt_epsilon", ndt_epsilon_);
    this->get_parameter("ndt_max_iterations", ndt_max_iterations_);

    heartbeat_ = HeartBeatPublisher::create(this);
    scManager_ = std::make_shared<SCManager>();
    scManager_->setDistThreshold(sc_dist_thresh_); 
    
    registered_scan_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_scan_for_init_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    register_ = std::make_shared<
        small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    // 调用加载地图
    loadGlobalMap(prior_pcd_file_);

    // 加载SC数据
    if (enable_sc_relocalization_ && !scd_directory_.empty() && !pose_file_.empty()) {
        loadScanContextData();
    } else {
        if(enable_sc_relocalization_) {
            RCLCPP_WARN(this->get_logger(), "ScanContext enabled but paths missing!");
        }
    }

    // 下采样Target (地图)
    target_ = small_gicp::voxelgrid_sampling_omp<
        pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *global_map_, global_leaf_size_);
    small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
    target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        target_, small_gicp::KdTreeBuilderOMP(num_threads_));

    // Subs & Timers
    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "registered_scan", 10,
        std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10,
        std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

    register_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), 
        std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

    // 2. 修改 TF 定时器的创建，把它加入到上面那个组里
    transform_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), 
        std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

SmallGicpRelocalizationNode::~SmallGicpRelocalizationNode() = default;

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded global map: %zu points", global_map_->points.size());

    // NOTE: 修改了这里的逻辑，增加超时退出，防止死锁
    Eigen::Affine3d odom_to_lidar_odom = Eigen::Affine3d::Identity();
    int retry_count = 0;
    const int MAX_RETRIES = 5; // 尝试5秒

    while (retry_count < MAX_RETRIES) {
        try {
            auto tf_stamped = tf_buffer_->lookupTransform(
                base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
            odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
            RCLCPP_INFO_STREAM(
                this->get_logger(), "Map Transform (Base->Lidar) loaded: " 
                << odom_to_lidar_odom.translation().transpose());
            break;
        } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Waiting for TF %s -> %s: %s", 
                base_frame_.c_str(), lidar_frame_.c_str(), ex.what());
            retry_count++;
            // 这里不再使用 sleep_for 阻塞，而是依赖 lookupTransform 的 timeout 参数
        }
    }
    
    if (retry_count >= MAX_RETRIES) {
        RCLCPP_ERROR(this->get_logger(), "FAILED to get Base->Lidar transform. Assuming Identity (might be wrong!).");
    }

    pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
}


void SmallGicpRelocalizationNode::loadScanContextData()
{
    loadPosesHelper(pose_file_, matrix_poses_);
    RCLCPP_INFO(this->get_logger(), "Loaded %ld history poses.", matrix_poses_.rows());

    file_count_ = 0;
    if (fs::exists(scd_directory_)) {
        for (const auto& entry : fs::directory_iterator(scd_directory_)) {
            if (fs::is_regular_file(entry)) file_count_++;
        }
    }
    for(int i = 0; i < file_count_; ++i) {
        std::string filename = padZeros(i);
        std::string scd_path = scd_directory_ + "/" + filename + ".scd";
        scManager_->loadSCDFile(scd_path);
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %d ScanContext descriptors.", file_count_);
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    last_scan_time_ = msg->header.stamp;
    pcl::fromROSMsg(*msg, *registered_scan_);

    if (!is_initialized_) {
        if (enable_sc_relocalization_) {
            pcl::PointCloud<pcl::PointXYZ> cloud_in_lidar_frame;
            Eigen::Isometry3d odom_to_lidar = Eigen::Isometry3d::Identity();
            try {
                auto tf_msg = tf_buffer_->lookupTransform(
                    lidar_frame_, msg->header.frame_id, tf2::TimePointZero);
                odom_to_lidar = tf2::transformToEigen(tf_msg.transform);
                pcl::transformPointCloud(*registered_scan_, cloud_in_lidar_frame, odom_to_lidar.matrix());
                
                *cloud_scan_for_init_ = cloud_in_lidar_frame;
                
                if (performGlobalLocalization()) {
                    RCLCPP_INFO(this->get_logger(), "Global Localization Successful!");
                    is_initialized_ = true;
                }
            } catch (tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "TF Error for SC Init: %s", ex.what());
            }
            return; 
        } else {
            is_initialized_ = true;
            RCLCPP_INFO_ONCE(this->get_logger(), "ScanContext disabled. Started at Origin.");
        }
    }

    {
        std::lock_guard<std::mutex> lock(result_mtx_);
        source_ = small_gicp::voxelgrid_sampling_omp<
            pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
            *registered_scan_, registered_leaf_size_);

        small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

        source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
            source_, small_gicp::KdTreeBuilderOMP(num_threads_));
    }
}

bool SmallGicpRelocalizationNode::performGlobalLocalization()
{
    if (scManager_->polarcontexts_.empty()) return false;

    scManager_->makeAndSaveScancontextAndKeys(*cloud_scan_for_init_);
    auto detectResult = scManager_->detectLoopClosureID(); 
    int best_index = detectResult.first;
    float yaw_diff = detectResult.second;

    if (best_index == -1) return false;
    RCLCPP_INFO(this->get_logger(), "SC Match: Index %d", best_index);

    if (best_index >= matrix_poses_.rows()) return false;

    Eigen::MatrixXd p = matrix_poses_.row(best_index);
    Eigen::Matrix3d R_hist;
    Eigen::Vector3d t_hist;
    R_hist << p(0), p(1), p(2),
              p(4), p(5), p(6),
              p(8), p(9), p(10);
    t_hist << p(3), p(7), p(11);

    Eigen::Isometry3d pose_map_lidar = Eigen::Isometry3d::Identity();
    pose_map_lidar.linear() = R_hist;
    pose_map_lidar.translation() = t_hist;
    
    pose_map_lidar.linear() = pose_map_lidar.linear() * Eigen::AngleAxisd(yaw_diff, Eigen::Vector3d::UnitZ());

    refineInitialization(pose_map_lidar);
    return true;
}

void SmallGicpRelocalizationNode::refineInitialization(const Eigen::Isometry3d& initial_guess_map_to_lidar)
{
    RCLCPP_INFO(this->get_logger(), "Refining with NDT...");

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setResolution(ndt_resolution_); 
    ndt.setStepSize(ndt_step_size_);
    ndt.setTransformationEpsilon(ndt_epsilon_);
    ndt.setMaximumIterations(ndt_max_iterations_);
    
    pcl::PointCloud<pcl::PointXYZ> source_in_base;
    try {
        auto tf = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
        Eigen::Isometry3d T_base_lidar = tf2::transformToEigen(tf.transform);
        pcl::transformPointCloud(*cloud_scan_for_init_, source_in_base, T_base_lidar.matrix());
    } catch (...) {
        source_in_base = *cloud_scan_for_init_;
    }

    ndt.setInputSource(source_in_base.makeShared());
    ndt.setInputTarget(global_map_);

    Eigen::Isometry3d guess_map_base = initial_guess_map_to_lidar;
    try {
        auto tf = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
        Eigen::Isometry3d T_base_lidar = tf2::transformToEigen(tf.transform);
        guess_map_base = initial_guess_map_to_lidar * T_base_lidar.inverse();
    } catch(...) {}

    pcl::PointCloud<pcl::PointXYZ> unused;
    ndt.align(unused, guess_map_base.matrix().cast<float>());

    Eigen::Isometry3d T_map_base_refined = guess_map_base;
    if (ndt.hasConverged()) {
        T_map_base_refined.matrix() = ndt.getFinalTransformation().cast<double>();
    } else {
        RCLCPP_WARN(this->get_logger(), "NDT Failed, using coarse guess.");
    }

    Eigen::Isometry3d T_base_odom = Eigen::Isometry3d::Identity();
    try {
        auto tf_msg = tf_buffer_->lookupTransform(
            robot_base_frame_, registered_scan_->header.frame_id, tf2::TimePointZero);
        T_base_odom = tf2::transformToEigen(tf_msg.transform);
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
    }

    {
        std::lock_guard<std::mutex> lock(result_mtx_);
        result_t_ = T_map_base_refined * T_base_odom;
        previous_result_t_ = result_t_; 
    }
}

void SmallGicpRelocalizationNode::performRegistration()
{
    if (!is_initialized_) return;
    // === 计时开始 ===
    // auto start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointCovariance>::Ptr curr_source;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> curr_source_tree;
    
    {
        std::lock_guard<std::mutex> lock(result_mtx_);
        if (!source_ || !source_tree_) return;
        curr_source = source_;
        curr_source_tree = source_tree_;
    }

    register_->reduction.num_threads = num_threads_;
    register_->rejector.max_dist_sq = max_dist_sq_;

    Eigen::Isometry3d prev_guess;
    {
        std::lock_guard<std::mutex> lock(result_mtx_);
        prev_guess = previous_result_t_;
    }

    auto result = register_->align(*target_, *curr_source, *target_tree_, prev_guess);

    if (!result.converged) {
        RCLCPP_WARN(this->get_logger(), "GICP lost.");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(result_mtx_);
        result_t_ = result.T_target_source;
        previous_result_t_ = result.T_target_source;
    }
    // === 计时结束 ===
    // auto end = std::chrono::high_resolution_clock::now();
    // double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

    // // 打印耗时
    // if (elapsed_ms > 100.0) { // 如果超过100ms就报警告
    //     RCLCPP_WARN(this->get_logger(), "GICP aligns took too long: %.2f ms", elapsed_ms);
    // } else {
    //     RCLCPP_INFO(this->get_logger(), "GICP align time: %.2f ms", elapsed_ms);
    // }
}

void SmallGicpRelocalizationNode::publishTransform()
{
    // std::cout<<123<<std::endl;
    std::lock_guard<std::mutex> lock(result_mtx_);
    // Identity matrix is valid, checking if IsZero (all elements 0) is enough
    if (result_t_.matrix().isZero()) return;

    geometry_msgs::msg::TransformStamped transform_stamped;
    
    // 关键修复：如果还没有收到点云，last_scan_time_ 为 0，TF 会发布到 1970 年
    // 这会导致 Rviz 显示 No Transform。
    // 修复：如果时间戳无效，使用当前时间。
    rclcpp::Time tf_time = last_scan_time_;
    if (tf_time.nanoseconds() == 0) {
        tf_time = this->now();
    }

    transform_stamped.header.stamp = tf_time + rclcpp::Duration::from_seconds(0.1);
    transform_stamped.header.frame_id = map_frame_;
    transform_stamped.child_frame_id = odom_frame_;

    const Eigen::Vector3d t = result_t_.translation();
    const Eigen::Quaterniond q(result_t_.rotation());

    transform_stamped.transform.translation.x = t.x();
    transform_stamped.transform.translation.y = t.y();
    transform_stamped.transform.translation.z = t.z();
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    RCLCPP_INFO(
        this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
        msg->pose.pose.position.y, msg->pose.pose.position.z);

    Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
    map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    map_to_robot_base.linear() = Eigen::Quaterniond(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();

    try {
        auto transform = tf_buffer_->lookupTransform(
            robot_base_frame_, registered_scan_->header.frame_id, tf2::TimePointZero);
        
        Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
        Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

        std::lock_guard<std::mutex> lock(result_mtx_);
        result_t_ = map_to_odom;
        previous_result_t_ = result_t_;
        is_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Manual initialization set.");

    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(
            this->get_logger(), "Could not transform initial pose from %s to %s: %s",
            robot_base_frame_.c_str(), registered_scan_->header.frame_id.c_str(), ex.what());
    }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
