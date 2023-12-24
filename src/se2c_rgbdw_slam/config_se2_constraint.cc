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

    spdlog::info("config file loaded: {}", config_file_path_);

    info_x_ = yaml_node["PlaneConstraint.info_x"].as<double>(1e-4);
    info_y_ = yaml_node["PlaneConstraint.info_y"].as<double>(1e-4);
    info_z_ = yaml_node["PlaneConstraint.info_z"].as<double>(1);
    info_rx_ = yaml_node["PlaneConstraint.info_rx"].as<double>(1e2);
    info_ry_ = yaml_node["PlaneConstraint.info_ry"].as<double>(1e2);
    info_rz_ = yaml_node["PlaneConstraint.info_rz"].as<double>(1e-4);
}

config_se2_constraint::config_se2_constraint(const std::shared_ptr<openvslam::config>& cfg)
{
    YAML::Node yaml_node = cfg->yaml_node_;

    info_x_ = yaml_node["PlaneConstraint.info_x"].as<double>(1e-4);
    info_y_ = yaml_node["PlaneConstraint.info_y"].as<double>(1e-4);
    info_z_ = yaml_node["PlaneConstraint.info_z"].as<double>(1);
    info_rx_ = yaml_node["PlaneConstraint.info_rx"].as<double>(1e2);
    info_ry_ = yaml_node["PlaneConstraint.info_ry"].as<double>(1e2);
    info_rz_ = yaml_node["PlaneConstraint.info_rz"].as<double>(1e-4);
    odom_info_factor_xy_ = yaml_node["OdometryConstraint.factor_xy"].as<double>(1000);
    odom_info_factor_th_ = yaml_node["OdometryConstraint.factor_th"].as<double>(1000);
}

config_se2_constraint::~config_se2_constraint() {
    spdlog::debug("DESTRUCT: config_se2_constraint");
}

std::ostream& operator<<(std::ostream& os, const config_se2_constraint& cfg) {
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

    return os;
}

} // namespace openvslam
