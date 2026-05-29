#include "utils/utils.hpp"
namespace fs = std::filesystem;

namespace utils {

Utils::Utils() : params_(UtilParams()) {
  initialize();
}

Utils::Utils(const UtilParams& params) : params_(params) {
  initialize();
}

void Utils::initialize() {
  makeDirectory(params_.save_directory_);
  j_ = 1;
  i_ = 1;
}

void Utils::makeDirectory(const std::string& directory) {
  if (!fs::is_directory(directory) || !fs::exists(directory)) {  // Check if directory exists
    fs::create_directory(directory);                             // create directory
  }
}

#ifdef USE_LASER_GEOMETRY
void Utils::laserscanToPointcloud2(const sensor_msgs::msg::LaserScan& laserscan,
                                   sensor_msgs::msg::PointCloud2& lasercloud) {
  projector_.projectLaser(laserscan, lasercloud);  // convert laserscan to pointcloud
}

sensor_msgs::msg::PointCloud2
Utils::laserscanToPointcloud2(const sensor_msgs::msg::LaserScan& laserscan) {
  sensor_msgs::msg::PointCloud2 lasercloud;
  projector_.projectLaser(laserscan, lasercloud);  // convert laserscan to pointcloud

  return lasercloud;
}
#endif

geometry_msgs::msg::Quaternion
Utils::multiplyQuaternions(const geometry_msgs::msg::Quaternion& q1,
                           const geometry_msgs::msg::Quaternion& q2) {
  geometry_msgs::msg::Quaternion result;

  result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

  return result;
}

geometry_msgs::msg::TransformStamped Utils::eulerToQuaternions(const double xyz[3]) {
  tf2::Quaternion q;
  q.setRPY(xyz[0], xyz[1], xyz[2]);

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();

  transform_stamped.transform.translation.y = 0;
  transform_stamped.transform.translation.z = 0;
  transform_stamped.transform.translation.x = 0;

  return transform_stamped;
}

geometry_msgs::msg::TransformStamped Utils::eulerToQuaternions(const std::vector<double>& xyz) {
  if (xyz.size() < 3) {
    throw std::invalid_argument("xyz vector must have at least 3 elements");
  }
  double rot[3] = {xyz[0], xyz[1], xyz[2]};
  return eulerToQuaternions(rot);
}

std::vector<std::string> Utils::getPcdPath(const std::string& folder_path) {
  std::vector<std::string> pcd_paths;

  for (const auto& entry : fs::directory_iterator(folder_path)) {
    if (entry.path().extension() == ".pcd") {
      pcd_paths.push_back(entry.path().string());
    }
  }
  // Define a custom comparator for sorting based on filenames
  auto natural_sort_comparator = [](const std::string& a, const std::string& b) {
    std::regex re("\\d+");
    std::smatch match_a, match_b;

    // Extract filenames
    std::string filename_a = fs::path(a).filename().string();
    std::string filename_b = fs::path(b).filename().string();

    // Find the first numeric part in each filename
    std::regex_search(filename_a, match_a, re);
    std::regex_search(filename_b, match_b, re);

    // If both filenames have numeric parts, compare them numerically
    if (!match_a.empty() && !match_b.empty()) {
      int num_a = std::stoi(match_a.str());
      int num_b = std::stoi(match_b.str());
      if (num_a != num_b)
        return num_a < num_b;
    }

    // Fallback to lexicographical comparison if numbers are the same or absent
    return filename_a < filename_b;
  };

  std::sort(pcd_paths.begin(), pcd_paths.end(), natural_sort_comparator);
  return pcd_paths;
}

void Utils::saveTrajectory(const std::vector<std::vector<double>>& curve,
                           const std::string& filename) {
  // Open the file in write mode
  std::ofstream outfile(filename);

  // Check if the file is open
  if (outfile.is_open()) {
    // Loop through the 2D vector and write each element to the file
    for (const auto& row : curve) {
      for (size_t i = 0; i < row.size(); ++i) {
        outfile << row[i];
        if (i < row.size() - 1) {
          outfile << ",";  // Add a comma between elements
        }
      }
      outfile << "\n";  // Newline after each row
    }
    // Close the file
    outfile.close();
    std::cout << "Data successfully written to " << filename << std::endl;
  } else {
    std::cerr << "Error: Could not open the file " << filename << std::endl;
  }
}
}  // namespace utils
