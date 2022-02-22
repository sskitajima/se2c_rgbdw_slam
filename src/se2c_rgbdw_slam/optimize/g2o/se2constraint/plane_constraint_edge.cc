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
 
    // std::cout << "plane_constraint_edge computeError plane constraint id: "<< _id << "\n" << err << "\n" << _error << std::endl;


    //////////////////////////////////

    // ::g2o::SE3Quat err = _transform_rc * _measurementInverse * v->estimate() * _transform_cr;
    // _error = err.log();
    // std::cout << "plane_constraint_edge computeError plane constraint id: "<< _id << "\n" << err << "\n" << _error << std::endl;
}

void plane_constraint_edge::setMeasurement(const ::g2o::SE3Quat& measurement_pose_cir)
{
    _measurement = measurement_pose_cir;    
   _measurementInverse = measurement_pose_cir.inverse();
   _measurementInverseAdj = _measurementInverse.adj();
//    _measurement_irc_Adj = (_transform_rc * _measurementInverse).adj();

//    std::cout << "measurement\n " << _measurement << std::endl;
//    std::cout << "measurementInverse\n " << _measurementInverse << std::endl;
// //    std::cout << "_measurement_irc_Adj\n " << _measurement_irc_Adj<< std::endl;
//    std::cout << "_measurementInverseAdj\n " << _measurementInverseAdj << std::endl;
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