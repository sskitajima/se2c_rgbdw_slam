#include "se2c_rgbdw_slam/config_se2_constraint.h"
#include "se2c_rgbdw_slam/camera/perspective.h"
#include "se2c_rgbdw_slam/camera/fisheye.h"
#include "se2c_rgbdw_slam/camera/equirectangular.h"

#include <iostream>
#include <memory>
#include <experimental/filesystem>

#include <spdlog/spdlog.h>

namespace openvslam {

config_se2_constraint::config_se2_constraint(const std::string& config_file_path)
    : config_file_path_(config_file_path)
{

    if(!std::experimental::filesystem::exists(config_file_path_))
    {
        nh_.param("/slam_config_path", config_file_path_, std::string(""));
        if(!std::experimental::filesystem::exists(config_file_path_))
        {
            std::cerr << "config_se2_constraint::config_se2_constraint() config_file_path doesn't exist.\npath: " << config_file_path << std::endl;
            abort();
        }
    }

    YAML::Node yaml_node = YAML::LoadFile(config_file_path_);

    spdlog::debug("CONSTRUCT: config_se2_constraint");

    spdlog::info("config file loaded: {}", config_file_path_);


    // spdlog::debug("load feature matching margin");
    // feature_matching_margin_ = yaml_node["FeatureMatching.margin"].as<double>(50);

    // spdlog::debug("load transform rc");
    // auto yaml_transform = yaml_node["transform_rc"];
    // transform_rc_ <<  yaml_transform[0][0].as<double>(0), yaml_transform[0][1].as<double>(0), yaml_transform[0][2].as<double>(1), yaml_transform[0][3].as<double>(0),
    //                   yaml_transform[1][0].as<double>(-1), yaml_transform[1][1].as<double>(0), yaml_transform[1][2].as<double>(0), yaml_transform[1][3].as<double>(0),
    //                   yaml_transform[2][0].as<double>(0), yaml_transform[2][1].as<double>(-1), yaml_transform[2][2].as<double>(0), yaml_transform[2][3].as<double>(0),
    //                   yaml_transform[3][0].as<double>(0), yaml_transform[3][1].as<double>(0), yaml_transform[3][2].as<double>(0), yaml_transform[3][3].as<double>(1);

    spdlog::debug("load information matrix");
    info_x_ = yaml_node["PlaneConstraint.info_x"].as<double>(1e-4);
    info_y_ = yaml_node["PlaneConstraint.info_y"].as<double>(1e-4);
    info_z_ = yaml_node["PlaneConstraint.info_z"].as<double>(1);
    info_rx_ = yaml_node["PlaneConstraint.info_rx"].as<double>(1e2);
    info_ry_ = yaml_node["PlaneConstraint.info_ry"].as<double>(1e2);
    info_rz_ = yaml_node["PlaneConstraint.info_rz"].as<double>(1e-4);

    // spdlog::debug("load odometry information param");
    // odom_ax_  = yaml_node["OdometryConstraint.ax"].as<double>(1.0);
    // odom_ay_  = yaml_node["OdometryConstraint.ay"].as<double>(1.0);
    // odom_ath_ = yaml_node["OdometryConstraint.ath"].as<double>(10);
    // odom_bx_  = yaml_node["OdometryConstraint.bx"].as<double>(0.1);
    // odom_by_  = yaml_node["OdometryConstraint.by"].as<double>(0.1);
    // odom_bth_ = yaml_node["OdometryConstraint.bth"].as<double>(0.1);

}

config_se2_constraint::config_se2_constraint(const std::shared_ptr<openvslam::config>& cfg)
{
    YAML::Node yaml_node = cfg->yaml_node_;

    spdlog::debug("CONSTRUCT: config_se2_constraint");

    // spdlog::debug("load feature matching margin");
    // feature_matching_margin_ = yaml_node["FeatureMatching.margin"].as<double>(50);

    // spdlog::debug("load transform rc");
    // auto yaml_transform = yaml_node["transform_rc"];
    // transform_rc_ <<  yaml_transform[0][0].as<double>(0), yaml_transform[0][1].as<double>(0), yaml_transform[0][2].as<double>(1), yaml_transform[0][3].as<double>(0),
    //                   yaml_transform[1][0].as<double>(-1), yaml_transform[1][1].as<double>(0), yaml_transform[1][2].as<double>(0), yaml_transform[1][3].as<double>(0),
    //                   yaml_transform[2][0].as<double>(0), yaml_transform[2][1].as<double>(-1), yaml_transform[2][2].as<double>(0), yaml_transform[2][3].as<double>(0),
    //                   yaml_transform[3][0].as<double>(0), yaml_transform[3][1].as<double>(0), yaml_transform[3][2].as<double>(0), yaml_transform[3][3].as<double>(1);

    spdlog::debug("load information matrix");
    info_x_ = yaml_node["PlaneConstraint.info_x"].as<double>(1e-4);
    info_y_ = yaml_node["PlaneConstraint.info_y"].as<double>(1e-4);
    info_z_ = yaml_node["PlaneConstraint.info_z"].as<double>(1);
    info_rx_ = yaml_node["PlaneConstraint.info_rx"].as<double>(1e2);
    info_ry_ = yaml_node["PlaneConstraint.info_ry"].as<double>(1e2);
    info_rz_ = yaml_node["PlaneConstraint.info_rz"].as<double>(1e-4);

    spdlog::debug("load odometry information param");
    odom_info_factor_xy_ = yaml_node["OdometryConstraint.factor_xy"].as<double>(1000);
    odom_info_factor_th_ = yaml_node["OdometryConstraint.factor_th"].as<double>(1000);
    // odom_ax_  = yaml_node["OdometryConstraint.ax"].as<double>(1.0);
    // odom_ay_  = yaml_node["OdometryConstraint.ay"].as<double>(1.0);
    // odom_ath_ = yaml_node["OdometryConstraint.ath"].as<double>(10);
    // odom_bx_  = yaml_node["OdometryConstraint.bx"].as<double>(0.1);
    // odom_by_  = yaml_node["OdometryConstraint.by"].as<double>(0.1);
    // odom_bth_ = yaml_node["OdometryConstraint.bth"].as<double>(0.1);
}

config_se2_constraint::~config_se2_constraint() {
    spdlog::debug("DESTRUCT: config_se2_constraint");
}

std::ostream& operator<<(std::ostream& os, const config_se2_constraint& cfg) {
    // std::cout << "feature matcning margin:"  << cfg.feature_matching_margin_ << std:: endl;;

    // std::cout << "Transform_rc:" << std::endl;
    // std::cout << cfg.transform_rc_;

    std::cout << "information matrix of SE2 constraint:" << std::endl;
    std::cout << "info_x_: " << cfg.info_x_ << std::endl;
    std::cout << "info_y_: " << cfg.info_y_ << std::endl;
    std::cout << "info_z_: " << cfg.info_z_ << std::endl;
    std::cout << "info_rx_: " << cfg.info_rx_ << std::endl;
    std::cout << "info_ry_: " << cfg.info_ry_ << std::endl;
    std::cout << "info_rz_: " << cfg.info_rz_ << std::endl;


    std::cout << "information matrix of Odometry constraint:" << std::endl;
    std::cout << "OdometryConstraint.factor_xy: " << cfg.odom_info_factor_xy_ << std::endl;
    std::cout << "OdometryConstraint.factor_th: " << cfg.odom_info_factor_th_ << std::endl;
    // std::cout << "OdometryConstraint.ath: " << cfg.odom_ath_ << std::endl;
    // std::cout << "OdometryConstraint.bx: " << cfg.odom_bx_ << std::endl;
    // std::cout << "OdometryConstraint.by: " << cfg.odom_by_ << std::endl;
    // std::cout << "OdometryConstraint.bth: " << cfg.odom_bth_ << std::endl;

    return os;
}

} // namespace openvslam
