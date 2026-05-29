#pragma once

#include "custom_pcl_pointxyzrgbi.hpp"

#include <filesystem>  // or #include <filesystem> for C++17 and up
#include <iomanip>     // for std::setw and std::setfill
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>
#include <pcl_ros/transforms.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#ifdef USE_LASER_GEOMETRY
  #include <laser_geometry/laser_geometry.hpp>
#endif
namespace utils {
struct UtilParams {
    // Default constructor with default values
    UtilParams(const int& mean_k = 9, const int& multi_thresh = 3, const double& outlier_radius = 25,
               const double& voxel_leaf_size = 2e-2, const std::string& save_directory = "/tmp/pcds",
               const double& mls_search_radius = 0.05, const int& mls_poly_order = 2,
               const std::vector<double>& cropbox_min = {-1.0, -1.0, -1.0},
               const std::vector<double>& cropbox_max = {1.0, 1.0, 1.0})
        : mean_k_(mean_k),
          multi_thresh_(multi_thresh),
          mls_poly_order_(mls_poly_order),
          outlier_radius_(outlier_radius),
          voxel_leaf_size_(voxel_leaf_size),
          mls_search_radius_(mls_search_radius),
          save_directory_(save_directory),
          cropbox_min_(cropbox_min),
          cropbox_max_(cropbox_max) {
    }

    int mean_k_, multi_thresh_, mls_poly_order_;
    double outlier_radius_, voxel_leaf_size_, mls_search_radius_;
    std::string save_directory_;
    std::vector<double> cropbox_min_, cropbox_max_;
};

class Utils {
   public:
    typedef std::shared_ptr<Utils> Ptr;

    Utils();

    Utils(const UtilParams& params);

    ~Utils() = default;

  #ifdef USE_LASER_GEOMETRY
    void laserscanToPointcloud2(const sensor_msgs::msg::LaserScan& laserscan,
                                sensor_msgs::msg::PointCloud2& pointcloud);
    sensor_msgs::msg::PointCloud2
    laserscanToPointcloud2(const sensor_msgs::msg::LaserScan& laserscan);
  #endif

    sensor_msgs::msg::PointCloud2 laserscanToPointcloud2(const sensor_msgs::msg::LaserScan& laserscan);

    template <typename PointT>
    void filterPointCloud(const sensor_msgs::msg::PointCloud2& cloud_in,
                          typename pcl::PointCloud<PointT>::Ptr& cloud_out, const std::string& filter_name = "voxel");

    template <typename PointT>
    void filterPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                          sensor_msgs::msg::PointCloud2& cloud_out, const std::string& filter_name = "voxel");

    template <typename PointT>
    void filterPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                          typename pcl::PointCloud<PointT>::Ptr& cloud_out, const std::string& filter_name = "voxel");

    template <typename PointT>
    void cloudSave(const std::string& save_name, const pcl::PointCloud<PointT>& cloud);

    template <typename PointT>
    void savePointcloud(const pcl::PointCloud<PointT>& cloud);

    template <typename PointT>
    void savePointcloud(const pcl::PointCloud<PointT>& cloud, const std::string& save_directory);

    template <typename PointT>
    void savePointcloud(const pcl::PointCloud<PointT>& cloud, const std::string& save_directory,
                        const std::string& name);

    void makeDirectory(const std::string& directory);

    template <typename PointT>
    void convertRos2Pcl(const sensor_msgs::msg::PointCloud2& cloud_in,
                        typename pcl::PointCloud<PointT>::Ptr& cloud_out);

    template <typename PointT>
    void convertPcl2Ros(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                        sensor_msgs::msg::PointCloud2& cloud_out);

    geometry_msgs::msg::Quaternion multiplyQuaternions(const geometry_msgs::msg::Quaternion& q1,
                                                       const geometry_msgs::msg::Quaternion& q2);

    geometry_msgs::msg::TransformStamped eulerToQuaternions(const double xyz[3]);
    geometry_msgs::msg::TransformStamped eulerToQuaternions(const std::vector<double>& xyz);

    template <typename PointT>
    std::vector<typename pcl::PointCloud<PointT>::Ptr> loadPCDs(const std::string& folder_path);

    std::vector<std::string> getPcdPath(const std::string& folder_path);

    void saveTrajectory(const std::vector<std::vector<double>>& curve, const std::string& filename);

   protected:
    UtilParams params_;

   private:
    void initialize();

    template <typename PointT>
    double getDistance(const PointT& p);

    template <typename PointT>
    void statisticalOutlierRemoval(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                                   typename pcl::PointCloud<PointT>::Ptr& cloud_out);
    template <typename PointT>
    void voxelGridFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                         typename pcl::PointCloud<PointT>::Ptr& cloud_out);
    template <typename PointT>
    void movingLeastSquares(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                            typename pcl::PointCloud<PointT>::Ptr& cloud_out);
    template <typename PointT>
    void distanceFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                        typename pcl::PointCloud<PointT>::Ptr& cloud_out);
    template <typename PointT>
    void cropBoxFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                       typename pcl::PointCloud<PointT>::Ptr& cloud_out);

    int i_, j_;

  #ifdef USE_LASER_GEOMETRY
    laser_geometry::LaserProjection projector_;
  #endif

    pcl::PCDWriter writer_;
};

template <typename PointT>
void Utils::filterPointCloud(const sensor_msgs::msg::PointCloud2& cloud_in,
                             typename pcl::PointCloud<PointT>::Ptr& cloud_out, const std::string& filter_name) {
    // Radius filter: remove points which are outside a sphere of radius `outlier_radius_`
    typename pcl::PointCloud<PointT>::Ptr pcl_pointcloud(new pcl::PointCloud<PointT>);
    convertRos2Pcl<PointT>(cloud_in, pcl_pointcloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pcl_pointcloud, *pcl_pointcloud, indices);

    if (filter_name == "cropbox") {
        cropBoxFilter<PointT>(pcl_pointcloud, cloud_out);
    } else if (filter_name == "voxel") {
        voxelGridFilter<PointT>(pcl_pointcloud, cloud_out);
    } else if (filter_name == "sor") {
        statisticalOutlierRemoval<PointT>(pcl_pointcloud, cloud_out);
    } else if (filter_name == "distance") {
        distanceFilter<PointT>(pcl_pointcloud, cloud_out);
    } else if (filter_name == "mls") {
        movingLeastSquares<PointT>(pcl_pointcloud, cloud_out);
    }
}

template <typename PointT>
void Utils::filterPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                             sensor_msgs::msg::PointCloud2& cloud_out_ros, const std::string& filter_name) {
    typename pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>);
    filterPointCloud<PointT>(cloud_in, cloud_out, filter_name);
    convertPcl2Ros<PointT>(cloud_out, cloud_out_ros);
}

template <typename PointT>
void Utils::filterPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                             typename pcl::PointCloud<PointT>::Ptr& cloud_out, const std::string& filter_name) {
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);

    if (filter_name == "cropbox") {
        cropBoxFilter<PointT>(cloud_in, cloud_out);
    } else if (filter_name == "voxel") {
        voxelGridFilter<PointT>(cloud_in, cloud_out);
    } else if (filter_name == "sor") {
        statisticalOutlierRemoval<PointT>(cloud_in, cloud_out);
    } else if (filter_name == "distance") {
        distanceFilter<PointT>(cloud_in, cloud_out);
    } else if (filter_name == "mls") {
        movingLeastSquares<PointT>(cloud_in, cloud_out);
    }
}

template <typename PointT>
void Utils::cloudSave(const std::string& save_name, const pcl::PointCloud<PointT>& cloud) {
    if (!cloud.empty()) {
        try {
            writer_.writeBinary(save_name, cloud);
        } catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }
}

template <typename PointT>
void Utils::savePointcloud(const pcl::PointCloud<PointT>& cloud, const std::string& save_directory,
                           const std::string& name) {
    makeDirectory(save_directory);
    std::string save_name = save_directory + name + ".pcd";
    cloudSave<PointT>(save_name, cloud);
}

template <typename PointT>
void Utils::savePointcloud(const pcl::PointCloud<PointT>& cloud) {
    std::ostringstream ss;
    ss << std::setw(5) << std::setfill('0') << i_;
    std::string save_name = params_.save_directory_ + "pcd_map_" + ss.str() + ".pcd";
    cloudSave<PointT>(save_name, cloud);
    ++i_;
    // pcl::io::savePCDFileASCII (save_name, cloud);
}

template <typename PointT>
void Utils::savePointcloud(const pcl::PointCloud<PointT>& cloud, const std::string& save_directory) {
    makeDirectory(save_directory);
    std::ostringstream ss;
    ss << std::setw(5) << std::setfill('0') << j_;
    std::string save_name = save_directory + "reg_pcds_" + ss.str() + ".pcd";
    cloudSave<PointT>(save_name, cloud);
    ++j_;
}

template <typename PointT>
void Utils::convertRos2Pcl(const sensor_msgs::msg::PointCloud2& cloud_in,
                           typename pcl::PointCloud<PointT>::Ptr& cloud_out) {
    pcl::fromROSMsg(cloud_in, *cloud_out);
}

template <typename PointT>
void Utils::convertPcl2Ros(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                           sensor_msgs::msg::PointCloud2& cloud_out) {
    pcl::toROSMsg(*cloud_in, cloud_out);
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> Utils::loadPCDs(const std::string& folder_path) {
    int i = 0;
    std::vector<std::string> pcd_paths = getPcdPath(folder_path);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> point_clouds;

    for (const auto& p : pcd_paths) {
        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        if (pcl::io::loadPCDFile<PointT>(p, *cloud) == -1) {
            std::cerr << "Couldn't read file " << p << std::endl;
            continue;
        }
        point_clouds.push_back(cloud);
        ++i;
    }
    std::cout << "Loaded " << i << " pcd files" << std::endl;
    return point_clouds;
}

template <typename PointT>
double Utils::getDistance(const PointT& p) {
    return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

template <typename PointT>
void Utils::statisticalOutlierRemoval(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                                      typename pcl::PointCloud<PointT>::Ptr& cloud_out) {
    /*
      Statistical Outlier removal:
      https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html The resulting
      cloud_out contains all points of cloud_in that have an average distance to their 8 nearest
      neighbors that is below the computed threshold Using a standard deviation multiplier of 1.0
      and assuming the average distances are normally distributed there is a 84.1% chance that a
      point will be an inlier
    */
    pcl::StatisticalOutlierRemoval<PointT> stat_outlier_remover(true);
    stat_outlier_remover.setInputCloud(cloud_in);
    stat_outlier_remover.setMeanK(params_.mean_k_);
    stat_outlier_remover.setStddevMulThresh(params_.multi_thresh_);
    stat_outlier_remover.filter(*cloud_out);
}

template <typename PointT>
void Utils::voxelGridFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                            typename pcl::PointCloud<PointT>::Ptr& cloud_out) {
    // Downsample pointcloud: https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html
    pcl::VoxelGrid<PointT> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(cloud_in);
    voxel_grid_filter.setLeafSize(params_.voxel_leaf_size_, params_.voxel_leaf_size_, params_.voxel_leaf_size_);
    voxel_grid_filter.filter(*cloud_out);
}

template <typename PointT>
void Utils::movingLeastSquares(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                               typename pcl::PointCloud<PointT>::Ptr& cloud_out) {
    // Moving Least Squares: https://pointclouds.org/documentation/tutorials/resampling.html
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::MovingLeastSquares<PointT, PointT> mls;
    typename pcl::PointCloud<PointT>::Ptr mls_cloud(new pcl::PointCloud<PointT>);

    mls.setInputCloud(cloud_in);
    mls.setPolynomialOrder(params_.mls_poly_order_);
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::RANDOM_UNIFORM_DENSITY);
    mls.setPointDensity(15);
    mls.setSqrGaussParam(0.021025);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(params_.mls_search_radius_);
    mls.process(*mls_cloud);
    *cloud_out = *mls_cloud;
}

template <typename PointT>
void Utils::distanceFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                           typename pcl::PointCloud<PointT>::Ptr& cloud_out) {
    auto condition = [this](const PointT& p) { return getDistance(p) > params_.outlier_radius_; };
    cloud_in->erase(std::remove_if(cloud_in->begin(), cloud_in->end(), condition), cloud_in->end());

    cloud_out = cloud_in;
}

template <typename PointT>
void Utils::cropBoxFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                          typename pcl::PointCloud<PointT>::Ptr& cloud_out) {
    pcl::CropBox<PointT> cropbox_filter(true);
    cropbox_filter.setNegative(true);
    cropbox_filter.setInputCloud(cloud_in);
    Eigen::Vector4f min_pt(params_.cropbox_min_[0], params_.cropbox_min_[1], params_.cropbox_min_[2], 1.0);
    Eigen::Vector4f max_pt(params_.cropbox_max_[0], params_.cropbox_max_[1], params_.cropbox_max_[2], 1.0);
    cropbox_filter.setMin(min_pt);
    cropbox_filter.setMax(max_pt);
    cropbox_filter.filter(*cloud_out);
}
}  // namespace utils
