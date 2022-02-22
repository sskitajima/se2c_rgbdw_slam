#ifndef OPENVSLAM_MODULE_PLANE_ESTIMATOR_H
#define OPENVSLAM_MODULE_PLANE_ESTIMATOR_H

#include "se2c_rgbdw_slam/config.h"
#include "se2c_rgbdw_slam/type.h"


namespace openvslam
{
namespace module
{
    
class plane_estimator
{
private:
    const double thres_inlier_;
    const unsigned num_ransac_iter_;
    Mat44_t transform_rc_default_;
    Mat33_t R_rc_base_;

    const double fx_;
    const double fy_;
    const double cx_;
    const double cy_;
    const double rows_;
    const double cols_;
    const double cam_dmin_;
    const double cam_dmax_;

    Vec4_t estimate_ransac(const std::vector<Vec4_t, Eigen::aligned_allocator<Eigen::Vector4d>>& points, std::vector<int>& inlier_idx) const;
    Vec4_t estimate_ls(const std::vector<Vec4_t, Eigen::aligned_allocator<Eigen::Vector4d>>& points, const std::vector<int>& in_inlier_idx, std::vector<int>& out_inlier_idx) const;

// TODO: octomap_mappingと統合
    Vec4_t recover_points(const double depth, const double ux, const double uy ) const;

    void cal_inlier(std::vector<int>& inlier_idx, const Vec4_t& plane_param, const std::vector<Vec4_t, Eigen::aligned_allocator<Eigen::Vector4d>>& points) const;

    void recover_points_from_depth_img(const cv::Mat& depth_img, std::vector<Vec4_t, Eigen::aligned_allocator<Eigen::Vector4d>>& points, const cv::Mat& mask=cv::Mat()) const;

    void cal_principal_component(const std::vector<Vec4_t, Eigen::aligned_allocator<Vec4_t>>& points, const std::vector<int>& inlier_idx,
                                 std::vector<Vec3_t, Eigen::aligned_allocator<Vec3_t>>& principal_vectors) const;

    Mat44_t estimate_transform_from_base(const std::vector<Vec3_t, Eigen::aligned_allocator<Vec3_t>>& principal_vectors) const;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //////////////////////////////////////////
    // constructor and destructor
    plane_estimator(const YAML::Node& yaml_node);
    ~plane_estimator();

    //////////////////////////////////////////
    // setter and getter
    void set_extrinsic_rc(const Mat44_t& transform_rc);

    //////////////////////////////////////////
    // static function
    // 点と平面の距離公式
    static double formula_point_and_plane(const Vec4_t& point, const Vec4_t& params);

    // 3点を通る平面の単位法線ベクトルを同時座標形式で返す
    static Vec4_t find_plane_params(const std::vector<Vec4_t, Eigen::aligned_allocator<Eigen::Vector4d>>& points);

    //////////////////////////////////////////
    // member function
    Mat44_t estimate(const cv::Mat& depth_img) const;
};


} // namespace module



} // namespace openvslam

#endif