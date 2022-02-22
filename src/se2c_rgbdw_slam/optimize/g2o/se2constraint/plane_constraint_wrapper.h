#ifndef OPENVSLAM_OPTIMIZE_G2O_SE2CONSTRAINT_PLANE_CONSTRAINT_WRAPPER_H
#define OPENVSLAM_OPTIMIZE_G2O_SE2CONSTRAINT_PLANE_CONSTRAINT_WRAPPER_H

#include "se2c_rgbdw_slam/type.h"
#include "se2c_rgbdw_slam/util/converter.h"
#include "se2c_rgbdw_slam/optimize/g2o/se2constraint/plane_constraint_edge.h"

#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <Eigen/Geometry>
#include <ros/ros.h>

// flag
#define ENABLE_PLANE_CONSTRAINT

namespace openvslam {

namespace util
{
class coordinate_transformer;
}
class config_se2_constraint;

namespace optimize {
namespace g2o {
namespace se2c {

class plane_constraint_wrapper
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    plane_constraint_wrapper(const std::shared_ptr<util::coordinate_transformer>& coord_transformer, const std::shared_ptr<config_se2_constraint>& plane_config);


    plane_constraint_edge* create_plane_constraint( ::g2o::SparseOptimizer& optimizer,
                                                    const openvslam::Mat44_t& cam_pose_cw, 
                                                    const double sqrt_chi_sq,
                                                    const int vId,
                                                    const int num_feature_matching=100
                                                );
    
    void update_param();

    std::shared_ptr<util::coordinate_transformer> coord_transformer_;
    std::shared_ptr<config_se2_constraint> plane_config_;

    openvslam::Mat44_t extPara_rc_;
    openvslam::Mat44_t extPara_cr_;
    Eigen::Matrix<double, 6, 6> info_bib_;
    ::g2o::SE3Quat T_bc_;
    ::g2o::SE3Quat T_cb_;

};

} // namespace se2c
} // namespace g2o
} // namespace optimize
} // namespace openvslam

#endif