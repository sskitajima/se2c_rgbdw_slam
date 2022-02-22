#include "odometry_constraint_wrapper.h"

#include "se2c_rgbdw_slam/config_se2_constraint.h"
#include "se2c_rgbdw_slam/util/coordinate_transformer.h"
#include "se2c_rgbdw_slam/util/se2c.h"

#include "se2c_rgbdw_slam/util/converter.h"

#include <g2o/core/robust_kernel_impl.h>

namespace openvslam {
namespace optimize {
namespace g2o {
namespace se2c {

    odometry_constraint_wrapper::odometry_constraint_wrapper(const std::shared_ptr<util::coordinate_transformer>& coord_transformer, const std::shared_ptr<config_se2_constraint>& plane_config)
    : coord_transformer_(coord_transformer), plane_config_(plane_config)
    {   
        // [rot trans]
        for(int i=0; i<6; i++)
            for(int j=0; j<6; j++)
                info_odom_(i,j) = 0;  
        info_odom_(0,0) = 1e-18;
        info_odom_(1,1) = 1e-18;
        info_odom_(2,2) = 1e0;
        info_odom_(3,3) = 1e0;
        info_odom_(4,4) = 1e0;
        info_odom_(5,5) = 1e-18;

        update_param();

        // std::cout << "[odometry_constraint_wrapper]\n";
        // std::cout << T_bc_ << std::endl;
        // std::cout << info_odom_ << std::endl;
        // std::cout << std::endl;
    }

    void odometry_constraint_wrapper::update_param()
    {
        extPara_rc_ = coord_transformer_->get_rc();
        extPara_cr_ = extPara_rc_.inverse();

        T_bc_ = ::openvslam::util::converter::to_g2o_SE3(extPara_rc_);
        Adj_T_bc_ = ::openvslam::util::se2c::Adj_TR(T_bc_);    

        Adj_T_bc_.block(3,0,3,3) = Adj_T_bc_.block(0,3,3,3);       // convert the order of rotation and translation
        Adj_T_bc_.block(0,3,3,3) = openvslam::Mat33_t::Zero();   // [trans rot] -> [rot trans]
    }

    odometry_constraint_edge* odometry_constraint_wrapper::create_odometry_constraint( ::g2o::SparseOptimizer& optimizer,
                                                    const openvslam::Mat44_t& odom_measurement_bb,
                                                    const openvslam::Mat66_t& info_odom,
                                                    const double sqrt_chi_sq,
                                                    const int vId1, const int vId2,
                                                    const int num_feature_matching
                                                )
    {
        openvslam::Mat44_t odom_measurement_cc = extPara_cr_ * odom_measurement_bb * extPara_rc_;
        ::g2o::SE3Quat odom_measurement_cc_g2o = ::openvslam::util::converter::to_g2o_SE3(odom_measurement_cc);

        openvslam::Vec6_t xi_cc = odom_measurement_cc_g2o.log();        // [rot trans]
        for(int i=0; i<5; i++) xi_cc(i) += 1e-18;

        openvslam::Mat66_t inv_JJl = ::openvslam::util::se2c::inv_JJl(-1 * xi_cc);
        openvslam::Mat66_t Jbc = -1 * Adj_T_bc_ * inv_JJl;

        // info_odom_(3,3) = info_odom(5,5);
        // info_odom_(4,4) = info_odom(0,0);
        // info_odom_(2,2) = info_odom(1,1);

        // info_odom_(3,2) = info_odom(0,5);       // x
        info_odom_(3,3) = info_odom(0,0);       // x
        // info_odom_(3,4) = info_odom(0,1);       // x

        // info_odom_(4,2) = info_odom(1,5);       // y
        // info_odom_(4,3) = info_odom(1,0);       // y
        info_odom_(4,4) = info_odom(1,1);       // y

        info_odom_(2,2) = info_odom(5,5);       // th
        // info_odom_(2,3) = info_odom(5,0);       // th
        // info_odom_(2,4) = info_odom(5,1);       // th

        openvslam::Mat66_t info_cam = Jbc.transpose() * info_odom_  * Jbc;

        // openvslam::Mat66_t info_cam = Adj_T_bc_.transpose() * info_odom  * Adj_T_bc_;
        // openvslam::Mat66_t info_cam = openvslam::Mat66_t::Identity();
        // info_cam(0,0) = 1e-5;
        // info_cam(1,1) = 1e5;
        // info_cam(2,2) = 1e-5;
        // info_cam(3,3) = 1e5;
        // info_cam(4,4) = 1e-5;
        // info_cam(5,5) = 1e5;


        for(int i = 0; i < 6; i++)
            for(int j = 0; j < i; j++)
                info_cam(i,j) = info_cam(j,i);


        info_cam *= num_feature_matching / 100.;
        // std::cout << "[odometry_constraint_wrapper::create_odometry_constraint()] num_feature_matching " << num_feature_matching << std::endl;

        odometry_constraint_edge* odomConstraint = new odometry_constraint_edge();
        odomConstraint->setInformation(info_cam);
        odomConstraint->setMeasurement(odom_measurement_cc_g2o);
        odomConstraint->vertices()[0] = optimizer.vertex(vId1);     // from
        odomConstraint->vertices()[1] = optimizer.vertex(vId2);     // to


        // using std::cout;
        // using std::endl;
        // cout << "vId " << vId1 << " " << vId2 << endl;
        // cout << "odom_measurement\n" << odom_measurement_bb << endl;
        // cout << "measurement_cc\n" << odom_measurement_cc << endl;
        // cout << "info_odom\n" << info_odom.diagonal() << endl;
        // cout << "info_odom_\n" << info_odom_ << endl;
        // cout << "xi_cc\n" << xi_cc << endl;
        // cout << "Adj_T_bc_\n" << Adj_T_bc_ << endl;
        // cout << "Jbc\n" << Jbc << endl;
        // cout << "info_cam\n" << info_cam << endl;
        // cout << endl;

        return odomConstraint;
    }

} // namespace se2c
} // namespace g2o
} // namespace optimize
} // namespace openvslam