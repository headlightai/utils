#include <utils/eigen.hpp>

namespace utils {
Eigen::Matrix3f rotationFromRollPitchYaw(const Eigen::Vector3d& roll_pitch_yaw) {
    return (Eigen::AngleAxisd(roll_pitch_yaw.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(roll_pitch_yaw.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll_pitch_yaw.x(), Eigen::Vector3d::UnitX()))
        .toRotationMatrix()
        .cast<float>();
}

// // Embeds a planar pose in 3D (z = 0, rotation about Z).
Eigen::Affine3f poseToTransform(const Eigen::Isometry2d& pose) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.linear().topLeftCorner<2, 2>() = pose.linear().cast<float>();
    transform.translation().head<2>() = pose.translation().cast<float>();
    return transform;
}
}  // namespace utils