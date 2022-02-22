#ifndef OPENVSLAM_UTIL_COORDINATE_TARANSFORMER_H
#define OPENVSLAM_UTIL_COORDINATE_TARANSFORMER_H

#include "se2c_rgbdw_slam/config.h"
#include "se2c_rgbdw_slam/type.h"

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <memory>

namespace openvslam
{

namespace optimize {
namespace g2o {
namespace se2c {
class plane_constraint_wrapper;
class odometry_constraint_wrapper;
}
}
}

namespace util
{

class coordinate_transformer
{
private:
    ros::NodeHandle nh_;
    std::string config_file_path_;

    const std::shared_ptr<config> cfg_;

    Mat44_t e_transform_rc_;
    Mat44_t e_transform_cr_;

    Mat33_t Mcam_to_robot_;
    Vec3_t Vcam_to_robot_;

    Mat33_t Mrobot_to_cam_;
    Vec3_t Vrobot_to_cam_;

    void load_param(const YAML::Node& yaml_node);

    std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper> plane_wrapper_;
    std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper> odometry_wrapper_;


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    coordinate_transformer(const std::string& config_file_path="");
    coordinate_transformer(const std::shared_ptr<openvslam::config>& cfg);
    ~coordinate_transformer();

    // T_rir -> T_cic
    Mat44_t TransformRobot2Cam(const Mat44_t& robot_pose);

    // T_cic -> T_rir
    Mat44_t TransformCam2Robot(const Mat44_t& cam_pose);

    Vec4_t InCam2InRobot(const Vec4_t& point_c);
    Vec4_t InRobot2InCam(const Vec4_t& point_r);


    Mat44_t get_rc() const;
    Mat44_t get_cr() const;
    void set_rc(const Mat44_t& transform_rc);
    void set_cr(const Mat44_t& transform_cr);

    void set_plane_wrapper(const std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper>& plane_wrapper);
    void set_odometry_wrapper(const std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper>& odometry_wrapper);

    void publish_initial_camera_pose() const;

};

} // namespace util
} // namespace openvslam

#endif