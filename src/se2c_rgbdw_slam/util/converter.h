#ifndef OPENVSLAM_UTIL_CONVERTER_H
#define OPENVSLAM_UTIL_CONVERTER_H

#include "se2c_rgbdw_slam/type.h"

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace openvslam {
namespace util {

class converter {
public:
    //! descriptor vector
    static std::vector<cv::Mat> to_desc_vec(const cv::Mat& desc);

    //! to SE3 of g2o
    static ::g2o::SE3Quat to_g2o_SE3(const Mat44_t& cam_pose);

    //! to Eigen::Mat/Vec
    static Mat44_t to_eigen_mat(const ::g2o::SE3Quat& g2o_SE3);
    static Mat44_t to_eigen_mat(const ::g2o::Sim3& g2o_Sim3);
    static Mat44_t to_eigen_cam_pose(const Mat33_t& rot, const Vec3_t& trans);

    //! from/to angle axis
    static Vec3_t to_angle_axis(const Mat33_t& rot_mat);
    static Mat33_t to_rot_mat(const Vec3_t& angle_axis);

    //! to homogeneous coordinates
    template<typename T>
    static Vec3_t to_homogeneous(const cv::Point_<T>& pt) {
        return Vec3_t{pt.x, pt.y, 1.0};
    }

    //! to skew symmetric matrix
    static Mat33_t to_skew_symmetric_mat(const Vec3_t& vec);

    static Mat33_t SE3toSE2(const Mat44_t& pose_se3);
    static Mat44_t SE2toSE3(const Mat33_t& pose_se2);
    static Mat44_t SE2paramtoSE3(const double x, const double y, const double th);
    static void SE2paramtoSE2(const double x, const double y, const double th, Mat33_t& trans);
    static void SE3toSE2param(const Mat44_t& pose_se3, double& x, double& y, double& th);

    static Mat33_t SE3covtoSE2cov(const Mat66_t& cov_se3);
    static Mat66_t SE2covtoSE3cov(const Mat33_t& cov_se2, const double maximum_cov);


    static void substitute_mat44(const Mat44_t& src, Mat44_t& dst);
    static void substitute_mat66(const Mat66_t& src, Mat66_t& dst);
};

} // namespace util
} // namespace openvslam

#endif // OPENVSLAM_UTIL_CONVERTER_H
