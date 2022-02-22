#include "se2c_rgbdw_slam/util/se2c.h"

namespace openvslam {
namespace util {

Matrix3D se2c::Jl(const Vector3D &v3d){
    double th = v3d.norm();
    double inv_th = 1. / th;
    double sint = std::sin(th);
    double cost = std::cos(th);
    Vector3D a = v3d * inv_th;

    Matrix3D Jl =
            sint * inv_th * Matrix3D::Identity()
            + ( 1 - sint * inv_th) * a * a.transpose()
            + ( 1 - cost ) * inv_th * g2o::skew(a);
    return Jl;
}

Matrix3D se2c::inv_Jl(const Vector3D &v3d) {
    double th = v3d.norm();
    double th_half = th * 0.5;
    double inv_th = 1. / th;
    double cot_half = std::tan(M_PI_2 - th_half);
    Vector3D a = v3d * inv_th;

    Matrix3D inv_Jl =
            th_half * cot_half * Matrix3D::Identity()
            + (1 - th_half * cot_half) * a * a.transpose()
            - th_half * g2o::skew(a);

    return inv_Jl;
}

openvslam::Mat66_t se2c::Adj_TR(const g2o::SE3Quat & pose)
{
    Matrix3D R = pose.rotation().toRotationMatrix();
    openvslam::Mat66_t res;
    res.block(0,0,3,3) = R;
    res.block(3,3,3,3) = R;
    res.block(0,3,3,3) = g2o::skew(pose.translation())*R;
    res.block(3,0,3,3) = Matrix3D::Zero(3,3);
    return res;
}

openvslam::Mat66_t se2c::inv_JJl(const openvslam::Vec6_t &v6d) {

    //! rho: translation; phi: rotation
    //! vector order: [rot, trans]

    Vector3D rho, phi;
    for(int i = 0; i < 3; i++) {
        phi[i] = v6d[i];
        rho[i] = v6d[i+3];
    }
    double th = phi.norm();
    Matrix3D Phi = g2o::skew(phi);
    Matrix3D Rho = g2o::skew(rho);
    double sint = sin(th);
    double cost = cos(th);
    double th2 = th * th;
    double th3 = th * th2;
    double th4 = th2 * th2;
    double th5 = th4  * th;
    double inv_th = 1./th;
    double inv_th3 = 1./th3;
    double inv_th4 = 1./th4;
    double inv_th5 = 1./th5;
    Matrix3D PhiRho = Phi * Rho;
    Matrix3D RhoPhi = Rho * Phi;
    Matrix3D PhiRhoPhi = PhiRho * Phi;
    Matrix3D PhiPhiRho = Phi * PhiRho;
    Matrix3D RhoPhiPhi = RhoPhi * Phi;
    Matrix3D PhiRhoPhiPhi = PhiRhoPhi * Phi;
    Matrix3D PhiPhiRhoPhi = Phi * PhiRhoPhi;

    double temp = (1. - 0.5 * th2 - cost) * inv_th4;

    Matrix3D Ql =
            0.5 * Rho + (th - sint) * inv_th3 * (PhiRho + RhoPhi + PhiRhoPhi)
            - temp * (PhiPhiRho + RhoPhiPhi -3. * PhiRhoPhi)
            - 0.5 * (temp - ( 3. * (th - sint) + th3 * 0.5) * inv_th5 ) * (PhiRhoPhiPhi + PhiPhiRhoPhi);


    double th_half = th * 0.5;
    double cot_half = tan(M_PI_2 - th_half);
    Vector3D a = phi * inv_th;
    Matrix3D inv_Jl =
            th_half * cot_half * Matrix3D::Identity()
            + (1 - th_half * cot_half) * a * a.transpose()
            - th_half * g2o::skew(a);

    openvslam::Mat66_t inv_JJl = openvslam::Mat66_t::Zero();
    inv_JJl.block<3,3>(0,0) = inv_Jl;
    inv_JJl.block<3,3>(3,0) = - inv_Jl * Ql * inv_Jl;
    inv_JJl.block<3,3>(3,3) = inv_Jl;
    return inv_JJl;
}

} // namespace util
} // namespace openvslam
