#include "se2c_rgbdw_slam/config_se2_constraint.h"
#include "se2c_rgbdw_slam/optimize/g2o/se2constraint/plane_constraint_wrapper.h"
#include "se2c_rgbdw_slam/optimize/g2o/se2constraint/odometry_constraint_wrapper.h"
#include "coordinate_transformer.h"

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <spdlog/spdlog.h>


#include <experimental/filesystem>
#include <string>


namespace openvslam
{
namespace util
{

coordinate_transformer::coordinate_transformer(const std::string& config_file_path)
    : config_file_path_(config_file_path)
{
    if(!std::experimental::filesystem::exists(config_file_path_))
    {
        nh_.param("/slam_config_path", config_file_path_, std::string(""));
        if(!std::experimental::filesystem::exists(config_file_path_))
        {
            std::cerr << "coordinate_transformer::coordinate_transformer() config_file_path doesn't exist.\npath: " << config_file_path << std::endl;
            abort();
        }
    }

    YAML::Node yaml_node = YAML::LoadFile(config_file_path_);

    spdlog::debug("CONSTRUCT: coordinate_transformer");

    spdlog::info("config file loaded: {}", config_file_path_);

    load_param(yaml_node);
}

coordinate_transformer::coordinate_transformer(const std::shared_ptr<openvslam::config>& cfg)
: cfg_(cfg)
{
    spdlog::debug("CONSTRUCT: coordinate_transformer");

    load_param(cfg_->yaml_node_);
    publish_initial_camera_pose();
}

coordinate_transformer::~coordinate_transformer()
{
}

void coordinate_transformer::load_param(const YAML::Node& yaml_node)
{
    auto yaml_transform = yaml_node["transform_rc"];
    e_transform_rc_ <<  yaml_transform[0][0].as<double>(0), yaml_transform[0][1].as<double>(0), yaml_transform[0][2].as<double>(1), yaml_transform[0][3].as<double>(0),
                        yaml_transform[1][0].as<double>(-1), yaml_transform[1][1].as<double>(0), yaml_transform[1][2].as<double>(0), yaml_transform[1][3].as<double>(0),
                        yaml_transform[2][0].as<double>(0), yaml_transform[2][1].as<double>(-1), yaml_transform[2][2].as<double>(0), yaml_transform[2][3].as<double>(0),
                        yaml_transform[3][0].as<double>(0), yaml_transform[3][1].as<double>(0), yaml_transform[3][2].as<double>(0), yaml_transform[3][3].as<double>(1);
    e_transform_cr_ = e_transform_rc_.inverse();

    Mrobot_to_cam_ = e_transform_rc_.block(0,0,3,3);
    Mcam_to_robot_ = Mrobot_to_cam_.transpose();

    Vrobot_to_cam_ = e_transform_rc_.block(0,3,3,1);
    Vcam_to_robot_ << e_transform_rc_.inverse().block(0,3,3,1);
                       
}

void coordinate_transformer::publish_initial_camera_pose() const
{
    tf::TransformBroadcaster robot_base_publisher_;
    tf::Transform tf_odom2initCamera;

    tf::Matrix3x3 tf_rotation(Mrobot_to_cam_(0, 0), Mrobot_to_cam_(0, 1), Mrobot_to_cam_(0, 2),
                              Mrobot_to_cam_(1, 0), Mrobot_to_cam_(1, 1), Mrobot_to_cam_(1, 2),
                              Mrobot_to_cam_(2, 0), Mrobot_to_cam_(2, 1), Mrobot_to_cam_(2, 2));
    tf::Quaternion tf_quat;
    tf_rotation.getRotation(tf_quat);
    tf_quat.normalize();
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;


    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "odom";
    static_transformStamped.child_frame_id = "init_camera";
    static_transformStamped.transform.translation.x = Vrobot_to_cam_(0);
    static_transformStamped.transform.translation.y = Vrobot_to_cam_(1);
    static_transformStamped.transform.translation.z = Vrobot_to_cam_(2);
    static_transformStamped.transform.rotation.x = tf_quat.x();
    static_transformStamped.transform.rotation.y = tf_quat.y();
    static_transformStamped.transform.rotation.z = tf_quat.z();
    static_transformStamped.transform.rotation.w = tf_quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
}

Mat44_t coordinate_transformer::get_cr() const
{
    return e_transform_cr_;
}

Mat44_t coordinate_transformer::get_rc() const
{
    return e_transform_rc_;
}

void coordinate_transformer::set_cr(const Mat44_t& transform_cr)
{
    e_transform_cr_ = transform_cr;
    e_transform_rc_ = transform_cr.inverse();

    plane_wrapper_->update_param();
    odometry_wrapper_->update_param();
}

void coordinate_transformer::set_rc(const Mat44_t& transform_rc)
{
    e_transform_rc_ = transform_rc;
    e_transform_cr_ = transform_rc.inverse();

    plane_wrapper_->update_param();
    odometry_wrapper_->update_param();
}

void coordinate_transformer::set_plane_wrapper(const std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper>& plane_wrapper)
{
    plane_wrapper_ = plane_wrapper;
}

void coordinate_transformer::set_odometry_wrapper(const std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper>& odometry_wrapper)
{
    odometry_wrapper_ = odometry_wrapper;
}

// T_rir -> T_icc
Mat44_t coordinate_transformer::TransformRobot2Cam(const Mat44_t &robot_pose)
{
    Mat44_t cam_pose_icc = (e_transform_cr_ * robot_pose * e_transform_rc_).inverse();
    return cam_pose_icc;
}


// T_cic -> T_irr
Mat44_t coordinate_transformer::TransformCam2Robot(const Mat44_t &cam_pose)
{
    Mat44_t robot_pose_irr = (e_transform_rc_ * cam_pose * e_transform_cr_).inverse();
    return robot_pose_irr;
}

// 同一点を異なる座標表現に変換
Vec4_t coordinate_transformer::InCam2InRobot(const Vec4_t& point_c)
{
    Vec3_t point_c_3, point_r_3;
    Vec4_t point_r_4;
    point_c_3 << point_c(0), point_c(1), point_c(2);
    // point_r_3 = Mcam_to_robot_ * point_c_3 + Vrobot_to_cam_;
    point_r_3 = Mrobot_to_cam_ * point_c_3 + Vrobot_to_cam_;
    point_r_4 << point_r_3(0), point_r_3(1), point_r_3(2), 1;
    return point_r_4;
}

Vec4_t coordinate_transformer::InRobot2InCam(const Vec4_t& point_r)
{
    Vec3_t point_r_3, point_c_3;
    Vec4_t point_c_4;
    point_r_3 << point_r(0), point_r(1), point_r(2);
    point_c_3 = Mcam_to_robot_ * point_r_3 + Vcam_to_robot_;
    point_c_4 << point_c_3(0), point_c_3(1), point_c_3(2), 1;
    return point_c_4;
}

} // namespace util
} // namespace openvslam