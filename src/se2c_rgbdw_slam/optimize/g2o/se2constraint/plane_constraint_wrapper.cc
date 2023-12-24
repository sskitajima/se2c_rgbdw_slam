#include "plane_constraint_wrapper.h"

#include "se2c_rgbdw_slam/config_se2_constraint.h"
#include "se2c_rgbdw_slam/util/coordinate_transformer.h"
#include "se2c_rgbdw_slam/util/se2c.h"


namespace openvslam {
namespace optimize {
namespace g2o {
namespace se2c {

    plane_constraint_wrapper::plane_constraint_wrapper(const std::shared_ptr<util::coordinate_transformer>& coord_transformer, const std::shared_ptr<config_se2_constraint>& plane_config)
    : coord_transformer_(coord_transformer), plane_config_(plane_config)
    {   
        // [rot trans]
        for(int i=0; i<6; i++)
            for(int j=0; j<6; j++)
                info_bib_(i,j) = 0;
    
        info_bib_(0,0) = plane_config_->info_rx_;
        info_bib_(1,1) = plane_config_->info_ry_;
        info_bib_(2,2) = plane_config_->info_rz_;
        info_bib_(3,3) = plane_config_->info_x_;
        info_bib_(4,4) = plane_config_->info_y_;
        info_bib_(5,5) = plane_config_->info_z_;

        update_param();
    }

    void plane_constraint_wrapper::update_param()
    {
        extPara_rc_ = coord_transformer_->get_rc();
        extPara_cr_ = extPara_rc_.inverse();

        T_bc_ = ::openvslam::util::converter::to_g2o_SE3(extPara_rc_);
        T_cb_ = T_bc_.inverse();
    }


    plane_constraint_edge* plane_constraint_wrapper::create_plane_constraint( ::g2o::SparseOptimizer& optimizer,
                                                    const openvslam::Mat44_t& cam_pose_cir, 
                                                    const double sqrt_chi_sq,
                                                    const int vId,
                                                    const int num_feature_matching
                                                )
    {
        using Matrix6d = Eigen::Matrix<double, 6, 6>;

        ::g2o::SE3Quat pose_cir = ::openvslam::util::converter::to_g2o_SE3(cam_pose_cir);
        ::g2o::SE3Quat T_bib = T_bc_ * pose_cir;
        
        // cam coordとrobot coordの関係による
        Eigen::AngleAxisd AngleAxis_bib(T_bib.rotation());
        Eigen::Vector3d Log_Rbib = AngleAxis_bib.angle() * AngleAxis_bib.axis();
        AngleAxis_bib = Eigen::AngleAxisd(Log_Rbib[2], Eigen::Vector3d::UnitZ());
        T_bib.setRotation(Eigen::Quaterniond(AngleAxis_bib));
        

        Eigen::Vector3d xyz_bib = T_bib.translation();
        xyz_bib[2] =0;
        T_bib.setTranslation(xyz_bib);

        // measurement
        ::g2o::SE3Quat T_cib = T_cb_ * T_bib;


        openvslam::Vec6_t xi_ibb_measurement = (T_cib.inverse() * T_cb_).log();       // [rot trans]
        for(int i=0; i<3; i++) xi_ibb_measurement(i) += 1e-18;
        Matrix6d inv_JJl = ::openvslam::util::se2c::inv_JJl(xi_ibb_measurement);
        Matrix6d info_cib = inv_JJl.transpose() * info_bib_ * inv_JJl;        // equation (39)

        // Make sure the infor matrix is symmetric
        for(int i = 0; i < 6; i++)
            for(int j = 0; j < i; j++)
                info_cib(i,j) = info_cib(j,i);

        info_cib *= num_feature_matching / 100.;


        plane_constraint_edge* planeConstraint = new plane_constraint_edge();
        planeConstraint->setInformation(info_cib);
        planeConstraint->setMeasurement(T_cib);
        planeConstraint->vertices()[0] = optimizer.vertex(vId);

        return planeConstraint;
    }

} // namespace se2c
} // namespace g2o
} // namespace optimize
} // namespace openvslam