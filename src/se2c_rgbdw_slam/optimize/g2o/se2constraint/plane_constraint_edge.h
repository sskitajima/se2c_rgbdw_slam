#ifndef OPENVSLAM_OPTIMIZER_G2O_SE2C_PLANE_CONSTRAINT_H
#define OPENVSLAM_OPTIMIZER_G2O_SE2C_PLANE_CONSTRAINT_H

#include "se2c_rgbdw_slam/type.h"
#include "se2c_rgbdw_slam/optimize/g2o/se3/shot_vertex.h"

#include <g2o/core/base_unary_edge.h>

namespace openvslam {
namespace optimize {
namespace g2o {
namespace se2c {

class plane_constraint_edge final : public ::g2o::BaseUnaryEdge<6, ::g2o::SE3Quat, g2o::se3::shot_vertex>
{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    plane_constraint_edge();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    // 誤差関数の定義
    void computeError() override;

    void setMeasurement(const ::g2o::SE3Quat& m) override;

    // ヤコビアンの定義
    void linearizeOplus() override;

    Measurement _measurementInverse;
    Eigen::Matrix<double, 6, 6> _measurementInverseAdj;

};

} // namespace se2c
} // namespace g2o
} // namespace optimize
} // namespace openvslam

#endif