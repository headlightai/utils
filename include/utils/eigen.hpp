#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace utils {
// Converts roll, pitch, and yaw angles to a 3x3 rotation matrix.
Eigen::Matrix3f rotationFromRollPitchYaw(const Eigen::Vector3d& roll_pitch_yaw);

// Converts a 2D pose to a 3D affine transform (z = 0, rotation about Z).
Eigen::Affine3f poseToTransform(const Eigen::Isometry2d& pose);
}  // namespace utils