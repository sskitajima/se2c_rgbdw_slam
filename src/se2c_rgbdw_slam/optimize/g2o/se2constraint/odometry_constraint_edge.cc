#include "odometry_constraint_edge.h"

namespace openvslam {
namespace optimize {
namespace g2o {
namespace se2c {

odometry_constraint_edge::odometry_constraint_edge()
: ::g2o::BaseBinaryEdge<6, ::g2o::SE3Quat, g2o::se3::shot_vertex, g2o::se3::shot_vertex>{}
{

}

bool odometry_constraint_edge::read(std::istream& is)
{
    return true;
}

bool odometry_constraint_edge::write(std::ostream& os) const
{
    return true;
}


// 誤差関数の定義
void odometry_constraint_edge::computeError()
{
    g2o::se3::shot_vertex *v0 = static_cast<g2o::se3::shot_vertex*>(_vertices[0]);       // from
    g2o::se3::shot_vertex *v1 = static_cast<g2o::se3::shot_vertex*>(_vertices[1]);       // to 
    ::g2o::SE3Quat err = _measurementInverse * v0->estimate() * v1->estimate().inverse() ;       // equation (22)
    _error = err.log();
}

void odometry_constraint_edge::setMeasurement(const ::g2o::SE3Quat& m)
{
    _measurement = m;    
   _measurementInverse = m.inverse();
   _measurementInverseAdj = _measurementInverse.adj();
}

// ヤコビアンの定義
void odometry_constraint_edge::linearizeOplus()
{
    g2o::se3::shot_vertex *v0 = static_cast<g2o::se3::shot_vertex*>(_vertices[0]);       // from
    g2o::se3::shot_vertex *v1 = static_cast<g2o::se3::shot_vertex*>(_vertices[1]);       // to 

    _jacobianOplusXi = _measurementInverseAdj;
    _jacobianOplusXj = -1 * (_measurementInverse * v0->estimate() *  v1->estimate().inverse()).adj();
}

} // namespace se2c
} // namespace g2o
} // namespace optimize
} // namespace openvslam