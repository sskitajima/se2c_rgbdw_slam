#include "se2c_rgbdw_slam/optimize/g2o/se2constraint/plane_constraint_edge.h"

namespace openvslam {
namespace optimize {
namespace g2o {
namespace se2c {

plane_constraint_edge::plane_constraint_edge()
: ::g2o::BaseUnaryEdge<6, ::g2o::SE3Quat, g2o::se3::shot_vertex>{}
{

}
bool plane_constraint_edge::read(std::istream& is)
{
    return true;
}

bool plane_constraint_edge::write(std::ostream& os) const
{
    return true;
}

// 誤差関数の定義
void plane_constraint_edge::computeError()
{
    g2o::se3::shot_vertex *v = static_cast<g2o::se3::shot_vertex*>(_vertices[0]);
    ::g2o::SE3Quat err = _measurementInverse * v->estimate() ;
    _error = err.log(); 
}

void plane_constraint_edge::setMeasurement(const ::g2o::SE3Quat& measurement_pose_cir)
{
    _measurement = measurement_pose_cir;    
   _measurementInverse = measurement_pose_cir.inverse();
   _measurementInverseAdj = _measurementInverse.adj();
}

// ヤコビアンの定義
void plane_constraint_edge::linearizeOplus()
{
    _jacobianOplusXi = _measurementInverseAdj;
}

} // namespace se2c
} // namespace g2o
} // namespace optimize
} // namespace openvslam