#ifndef OPENVSLAM_OPTIMIZE_G2O_SE2CONSTRAINT_ODOMETRY_CONSTRAINT_WRAPPER_H
#define OPENVSLAM_OPTIMIZE_G2O_SE2CONSTRAINT_ODOMETRY_CONSTRAINT_WRAPPER_H

#include "se2c_rgbdw_slam/type.h"
#include "se2c_rgbdw_slam/util/converter.h"
#include "se2c_rgbdw_slam/optimize/g2o/se2constraint/odometry_constraint_edge.h"

#include <g2o/core/sparse_optimizer.h>
#include <Eigen/Geometry>
#include <ros/ros.h>

// flag
#define ENABLE_ODOMETRY_CONSTRAINT

namespace openvslam {

namespace util
{
class coordinate_transformer;
}
class config_se2_constraint;

namespace optimize {
namespace g2o {
namespace se2c {

class odometry_constraint_wrapper
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    odometry_constraint_wrapper(const std::shared_ptr<util::coordinate_transformer>& coord_transformer, const std::shared_ptr<config_se2_constraint>& plane_config);


    odometry_constraint_edge* create_odometry_constraint( ::g2o::SparseOptimizer& optimizer,
                                                          const openvslam::Mat44_t& odom_measurement_bb,
                                                          const openvslam::Mat66_t& info_odom,
                                                          const double sqrt_chi_sq,
                                                          const int vId1, const int vId2,
                                                          const int num_feature_matching=-1
                                                );
    void update_param();

    std::shared_ptr<util::coordinate_transformer> coord_transformer_;
    std::shared_ptr<config_se2_constraint> plane_config_;

    openvslam::Mat44_t extPara_rc_;
    openvslam::Mat44_t extPara_cr_;
    openvslam::Mat66_t info_odom_;
    ::g2o::SE3Quat T_bc_;
    openvslam::Mat66_t Adj_T_bc_;
};

} // namespace se2c
} // namespace g2o
} // namespace optimize
} // namespace openvslam

#endif