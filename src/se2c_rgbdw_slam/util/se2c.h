#ifndef OPENVSLAM_UTIL_SE2C_H
#define OPENVSLAM_UTIL_SE2C_H

#include "se2c_rgbdw_slam/type.h"

#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace openvslam {
namespace util {

class se2c {
public:

static openvslam::Matrix3D Jl(const openvslam::Vector3D &v3d);
static openvslam::Matrix3D inv_Jl(const openvslam::Vector3D &v3d);
static openvslam::Mat66_t Adj_TR(const g2o::SE3Quat & pose);
static openvslam::Mat66_t inv_JJl(const openvslam::Vec6_t &v6d);

};

} // namespace util
} // namespace openvslam

#endif // OPENVSLAM_UTIL_SE2C_H
