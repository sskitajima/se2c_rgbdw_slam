#ifndef OPENVSLAM_ROS_UTIL_H
#define OPENVSLAM_ROS_UTIL_H
#include "se2c_rgbdw_slam/type.h"

#include <tf/transform_broadcaster.h>

namespace openvslam_ros
{
class converter
{
public:
    static tf::Transform Eigen2TF(const openvslam::Mat44_t& mat);
    static openvslam::Mat44_t TF2Eigen(const tf::Transform& tf);
};
}

#endif