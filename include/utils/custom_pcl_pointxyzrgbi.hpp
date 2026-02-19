#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>  // POINT_CLOUD_REGISTER_POINT_STRUCT

/*
    The realsense camera pointclouds are of type PointXYZRGB and
    the lidar pointclouds are of type PointXYZI. To fuse them together,
    we define a custom point type PointXYZRGBI which has fields for
    x,y,z,rgb and intensity.
*/
namespace pcl {
struct EIGEN_ALIGN16 PointXYZRGBI {
    PCL_ADD_POINT4D;  // adds float x,y,z and padding (data[4])
    PCL_ADD_RGB
    float intensity;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl

// Register the struct with PCL (names in parenthesis must match member names)
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZRGBI,
                                  (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(float, intensity, intensity))

// #define PCL_INSTANTIATE_KEYWORD(PointXYZRGBI)
