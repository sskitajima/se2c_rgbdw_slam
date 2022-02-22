#include "se2c_rgbdw_slam/util/converter.h"

namespace openvslam {
namespace util {

std::vector<cv::Mat> converter::to_desc_vec(const cv::Mat& desc) {
    std::vector<cv::Mat> desc_vec(desc.rows);
    for (int i = 0; i < desc.rows; ++i) {
        desc_vec.at(i) = desc.row(i);
    }
    return desc_vec;
}

g2o::SE3Quat converter::to_g2o_SE3(const Mat44_t& cam_pose) {
    const Mat33_t rot = cam_pose.block<3, 3>(0, 0);
    const Vec3_t trans = cam_pose.block<3, 1>(0, 3);
    return g2o::SE3Quat{rot, trans};
}

Mat44_t converter::to_eigen_mat(const g2o::SE3Quat& g2o_SE3) {
    return g2o_SE3.to_homogeneous_matrix();
}

Mat44_t converter::to_eigen_mat(const g2o::Sim3& g2o_Sim3) {
    Mat44_t cam_pose = Mat44_t::Identity();
    cam_pose.block<3, 3>(0, 0) = g2o_Sim3.scale() * g2o_Sim3.rotation().toRotationMatrix();
    cam_pose.block<3, 1>(0, 3) = g2o_Sim3.translation();
    return cam_pose;
}

Mat44_t converter::to_eigen_cam_pose(const Mat33_t& rot, const Vec3_t& trans) {
    Mat44_t cam_pose = Mat44_t::Identity();
    cam_pose.block<3, 3>(0, 0) = rot;
    cam_pose.block<3, 1>(0, 3) = trans;
    return cam_pose;
}

Vec3_t converter::to_angle_axis(const Mat33_t& rot_mat) {
    const Eigen::AngleAxisd angle_axis(rot_mat);
    return angle_axis.axis() * angle_axis.angle();
}

Mat33_t converter::to_rot_mat(const Vec3_t& angle_axis) {
    Eigen::Matrix3d rot_mat;
    const double angle = angle_axis.norm();
    if (angle <= 1e-5) {
        rot_mat = Eigen::Matrix3d::Identity();
    }
    else {
        const Eigen::Vector3d axis = angle_axis / angle;
        rot_mat = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    }
    return rot_mat;
}

Mat33_t converter::to_skew_symmetric_mat(const Vec3_t& vec) {
    Mat33_t skew;
    skew << 0, -vec(2), vec(1),
        vec(2), 0, -vec(0),
        -vec(1), vec(0), 0;
    return skew;
}

Mat33_t converter::SE3toSE2(const Mat44_t& pose_se3)
{
    Mat33_t pose_se2;

    pose_se2(0, 0) = pose_se3(0, 0);
    pose_se2(0, 1) = pose_se3(0, 1);
    pose_se2(0, 2) = pose_se3(0, 2);

    pose_se2(1, 0) = pose_se2(1, 0);
    pose_se2(1, 1) = pose_se2(1, 1);
    pose_se2(1, 2) = pose_se2(1, 2);
    
    pose_se2(2, 0) = 0;
    pose_se2(2, 1) = 0;
    pose_se2(2, 2) = 1;
    
    return pose_se2;
}

Mat44_t converter::SE2toSE3(const Mat33_t& pose_se2)
{
    Mat44_t pose_se3;

    pose_se3(0, 0) = pose_se2(0, 0);
    pose_se3(0, 1) = pose_se2(0, 1);
    pose_se3(0, 2) = 0;
    pose_se3(0, 3) = pose_se2(0, 2);

    pose_se3(1, 0) = pose_se2(1, 0);
    pose_se3(1, 1) = pose_se2(1, 1);
    pose_se3(1, 2) = 0;
    pose_se3(1, 3) = pose_se2(1, 2);
    
    pose_se3(2, 0) = 0;
    pose_se3(2, 1) = 0;
    pose_se3(2, 2) = 1;
    pose_se3(2, 3) = 0;
    
    pose_se3(3, 0) = 0;
    pose_se3(3, 1) = 0;
    pose_se3(3, 2) = 0;
    pose_se3(3, 3) = 1;

    return pose_se3;
}

Mat44_t converter::SE2paramtoSE3(const double x, const double y, const double th)
{
    Mat44_t trans;
    trans(0, 0) = cos(th);
    trans(0, 1) = -sin(th);
    trans(0, 2) = 0;
    trans(0, 3) = x;

    trans(1, 0) = sin(th);
    trans(1, 1) = cos(th);
    trans(1, 2) = 0;
    trans(1, 3) = y;
    
    trans(2, 0) = 0;
    trans(2, 1) = 0;
    trans(2, 2) = 1;
    trans(2, 3) = 0;
    
    trans(3, 0) = 0;
    trans(3, 1) = 0;
    trans(3, 2) = 0;
    trans(3, 3) = 1;

    return trans;
}

void converter::SE2paramtoSE2(const double x, const double y, const double th, Mat33_t& trans)
{
    trans(0, 0) = cos(th);
    trans(0, 1) = -sin(th);
    trans(0, 3) = x;

    trans(1, 0) = sin(th);
    trans(1, 1) = cos(th);
    trans(1, 3) = y;
    
    trans(2, 0) = 0;
    trans(2, 1) = 0;
    trans(2, 2) = 1;
}

void converter::SE3toSE2param(const Mat44_t& pose_se3, double& x, double& y, double& th)
{
    x = pose_se3(0, 3);
    y = pose_se3(1, 3);

    const double sin_th = pose_se3(1, 0);
    sin_th >=0 ? th = acos(pose_se3(0,0)) : th = -1 * acos(pose_se3(0,0));
}


Mat33_t converter::SE3covtoSE2cov(const Mat66_t& cov_se3)
{
    Mat33_t cov_se2;
    cov_se2(0, 0) = cov_se3(0, 0);
    cov_se2(0, 1) = cov_se3(0, 1);
    cov_se2(0, 2) = cov_se3(0, 5);

    cov_se2(1, 0) = cov_se3(1, 0);
    cov_se2(1, 1) = cov_se3(1, 1);
    cov_se2(1, 2) = cov_se3(1, 5);

    cov_se2(2, 0) = cov_se3(5, 0);
    cov_se2(2, 1) = cov_se3(5, 1);
    cov_se2(2, 2) = cov_se3(5, 5);

    return cov_se2;
}

Mat66_t converter::SE2covtoSE3cov(const Mat33_t& cov_se2, const double maximum_cov)
{
    Mat66_t cov_se3;
    cov_se3(0,0) = cov_se2(0,0);
    cov_se3(0,1) = cov_se2(0,1);
    cov_se3(0,2) = 0;
    cov_se3(0,3) = 0;
    cov_se3(0,4) = 0;
    cov_se3(0,5) = cov_se2(0,2);

    cov_se3(1,0) = cov_se2(1,0);
    cov_se3(1,1) = cov_se2(1,1);
    cov_se3(1,2) = 0;
    cov_se3(1,3) = 0;
    cov_se3(1,4) = 0;
    cov_se3(1,5) = cov_se2(1,2);

    cov_se3(2,0) = 0;
    cov_se3(2,1) = 0;
    cov_se3(2,2) = maximum_cov;
    cov_se3(2,3) = 0;
    cov_se3(2,4) = 0;
    cov_se3(2,5) = 0;

    cov_se3(3,0) = 0;
    cov_se3(3,1) = 0;
    cov_se3(3,2) = 0;
    cov_se3(3,3) = maximum_cov;
    cov_se3(3,4) = 0;
    cov_se3(3,5) = 0;

    cov_se3(4,0) = 0;
    cov_se3(4,1) = 0;
    cov_se3(4,2) = 0;
    cov_se3(4,3) = 0;
    cov_se3(4,4) = maximum_cov;
    cov_se3(4,5) = 0;

    cov_se3(5,0) = cov_se2(2,0);
    cov_se3(5,1) = cov_se2(2,1);
    cov_se3(5,2) = 0;
    cov_se3(5,3) = 0;
    cov_se3(5,4) = 0;
    cov_se3(5,5) = cov_se2(2,2);

    return cov_se3;

}

void converter::substitute_mat44(const Mat44_t& src, Mat44_t& dst)
{
    dst(0, 0) = src(0, 0);
    dst(0, 1) = src(0, 1);
    dst(0, 2) = src(0, 2);
    dst(0, 3) = src(0, 3);

    dst(1, 0) = src(1, 0);
    dst(1, 1) = src(1, 1);
    dst(1, 2) = src(1, 2);
    dst(1, 3) = src(1, 3);

    dst(2, 0) = src(2, 0);
    dst(2, 1) = src(2, 1);
    dst(2, 2) = src(2, 2);
    dst(2, 3) = src(2, 3);

    dst(3, 0) = src(3, 0);
    dst(3, 1) = src(3, 1);
    dst(3, 2) = src(3, 2);
    dst(3, 3) = src(3, 3);

}

void converter::substitute_mat66(const Mat66_t& src, Mat66_t& dst)
{
    dst(0, 0) = src(0, 0);
    dst(0, 1) = src(0, 1);
    dst(0, 2) = src(0, 2);
    dst(0, 3) = src(0, 3);
    dst(0, 4) = src(0, 4);
    dst(0, 5) = src(0, 5);

    dst(1, 0) = src(1, 0);
    dst(1, 1) = src(1, 1);
    dst(1, 2) = src(1, 2);
    dst(1, 3) = src(1, 3);
    dst(1, 4) = src(1, 4);
    dst(1, 5) = src(1, 5);
    
    dst(2, 0) = src(2, 0);
    dst(2, 1) = src(2, 1);
    dst(2, 2) = src(2, 2);
    dst(2, 3) = src(2, 3);
    dst(2, 4) = src(2, 4);
    dst(2, 5) = src(2, 5);

    dst(3, 0) = src(3, 0);
    dst(3, 1) = src(3, 1);
    dst(3, 2) = src(3, 2);
    dst(3, 3) = src(3, 3);
    dst(3, 4) = src(3, 4);
    dst(3, 5) = src(3, 5);

    dst(4, 0) = src(4, 0);
    dst(4, 1) = src(4, 1);
    dst(4, 2) = src(4, 2);
    dst(4, 3) = src(4, 3);
    dst(4, 4) = src(4, 4);
    dst(4, 5) = src(4, 5);

    dst(5, 0) = src(5, 0);
    dst(5, 1) = src(5, 1);
    dst(5, 2) = src(5, 2);
    dst(5, 3) = src(5, 3);
    dst(5, 4) = src(5, 4);
    dst(5, 5) = src(5, 5);
}

} // namespace util
} // namespace openvslam
