#include "se2c_rgbdw_slam/config.h"
#include "se2c_rgbdw_slam/camera/perspective.h"
#include "se2c_rgbdw_slam/camera/fisheye.h"
#include "se2c_rgbdw_slam/camera/equirectangular.h"

#include <iostream>
#include <memory>

#include <spdlog/spdlog.h>

namespace openvslam {

config::config(const std::string& config_file_path)
    : config(YAML::LoadFile(config_file_path), config_file_path) {}

config::config(const YAML::Node& yaml_node, const std::string& config_file_path)
    : config_file_path_(config_file_path), yaml_node_(yaml_node) {
    spdlog::debug("CONSTRUCT: config");

    spdlog::info("config file loaded: {}", config_file_path_);

    //========================//
    // Load Camera Parameters //
    //========================//

    spdlog::debug("load camera model type");
    const auto camera_model_type = camera::base::load_model_type(yaml_node_);

    spdlog::debug("load camera model parameters");
    try {
        switch (camera_model_type) {
            case camera::model_type_t::Perspective: {
                camera_ = new camera::perspective(yaml_node_);
                break;
            }
            case camera::model_type_t::Fisheye: {
                camera_ = new camera::fisheye(yaml_node_);
                break;
            }
            case camera::model_type_t::Equirectangular: {
                camera_ = new camera::equirectangular(yaml_node_);
                break;
            }
        }
    }
    catch (const std::exception& e) {
        spdlog::debug("failed in loading camera model parameters: {}", e.what());
        delete camera_;
        camera_ = nullptr;
        throw;
    }

    //=====================//
    // Load ORB Parameters //
    //=====================//

    spdlog::debug("load ORB parameters");
    try {
        orb_params_ = feature::orb_params(yaml_node_);
    }
    catch (const std::exception& e) {
        spdlog::debug("failed in loading ORB parameters: {}", e.what());
        delete camera_;
        camera_ = nullptr;
        throw;
    }

    //==========================//
    // Load Tracking Parameters //
    //==========================//

    spdlog::debug("load tracking parameters");

    spdlog::debug("load depth threshold");
    if (camera_->setup_type_ == camera::setup_type_t::Stereo || camera_->setup_type_ == camera::setup_type_t::RGBD) {
        // ベースライン長の一定倍より遠いdepthは無視する
        const auto depth_thr_factor = yaml_node_["depth_threshold"].as<double>(40.0);

        switch (camera_->model_type_) {
            case camera::model_type_t::Perspective: {
                auto camera = static_cast<camera::perspective*>(camera_);
                true_depth_thr_ = camera->true_baseline_ * depth_thr_factor;
                break;
            }
            case camera::model_type_t::Fisheye: {
                auto camera = static_cast<camera::fisheye*>(camera_);
                true_depth_thr_ = camera->true_baseline_ * depth_thr_factor;
                break;
            }
            case camera::model_type_t::Equirectangular: {
                throw std::runtime_error("Not implemented: Stereo or RGBD of equirectangular camera model");
            }
        }
    }

    spdlog::debug("load depthmap factor");
    if (camera_->setup_type_ == camera::setup_type_t::RGBD) {
        depthmap_factor_ = yaml_node_["depthmap_factor"].as<double>(1.0);
    }
}

config::~config() {
    delete camera_;
    camera_ = nullptr;

    spdlog::debug("DESTRUCT: config");
}

std::ostream& operator<<(std::ostream& os, const config& cfg) {
    std::cout << "Camera Configuration:" << std::endl;
    cfg.camera_->show_parameters();

    std::cout << "ORB Configuration:" << std::endl;
    cfg.orb_params_.show_parameters();

    if (cfg.camera_->setup_type_ == camera::setup_type_t::Stereo || cfg.camera_->setup_type_ == camera::setup_type_t::RGBD) {
        std::cout << "Stereo Configuration:" << std::endl;
        std::cout << "- true baseline: " << cfg.camera_->true_baseline_ << std::endl;
        std::cout << "- true depth threshold: " << cfg.true_depth_thr_ << std::endl;
        std::cout << "- depth threshold factor: " << cfg.true_depth_thr_ / cfg.camera_->true_baseline_ << std::endl;
    }
    if (cfg.camera_->setup_type_ == camera::setup_type_t::RGBD) {
        std::cout << "Depth Image Configuration:" << std::endl;
        std::cout << "- depthmap factor: " << cfg.depthmap_factor_ << std::endl;
    }

    //===========================//
    // Print Tracking Parameters //
    //===========================//
    std::cout << std::endl;
    std::cout << "Encoder Configuration: " << std::endl;
    std::cout << "- Encoder.wheel_separation: " << cfg.yaml_node_["Encoder.wheel_separation"].as<double>(-1) << std::endl;
    std::cout << "- Encoder.wheel_radius: " << cfg.yaml_node_["Encoder.wheel_radius"].as<double>(-1) << std::endl;
    std::cout << "- Encoder.num_pulse_per_meter: " << cfg.yaml_node_["Encoder.num_pulse_per_meter"].as<double>(-1) << std::endl;
    std::cout << "- Encoder.num_pulse_per_meter_right: " << cfg.yaml_node_["Encoder.num_pulse_per_meter_right"].as<double>(-1) << std::endl;
    std::cout << "- Encoder.num_pulse_per_meter_left: " << cfg.yaml_node_["Encoder.num_pulse_per_meter_left"].as<double>(-1) << std::endl;
    std::cout << "- Encoder.displacement_covariance_factor: " << cfg.yaml_node_["Encoder.displacement_covariance_factor"].as<double>(-1) << std::endl;
    std::cout << "- Encoder.minimum_covariance: " << cfg.yaml_node_["Encoder.minimum_covariance"].as<double>(-1) << std::endl;

    std::cout << std::endl;
    std::cout << "Octomap Configuration: " << std::endl;
    std::cout << "- Octomap.min_depth: " << cfg.yaml_node_["Octomap.min_depth"].as<double>(-1) << std::endl;
    std::cout << "- Octomap.max_depth: " << cfg.yaml_node_["Octomap.max_depth"].as<double>(-1) << std::endl;
    std::cout << "- Octomap.voxel_size: " << cfg.yaml_node_["Octomap.voxel_size"].as<double>(-1) << std::endl;
    std::cout << "- Octomap.occupancy_thres: " << cfg.yaml_node_["Octomap.occupancy_thres"].as<double>(-1) << std::endl;
    std::cout << "- Octomap.prob_hit: " << cfg.yaml_node_["Octomap.prob_hit"].as<double>(-1) << std::endl;
    std::cout << "- Octomap.prob_miss: " << cfg.yaml_node_["Octomap.prob_miss"].as<double>(-1) << std::endl;

    std::cout << std::endl;
    std::cout << "Plane Estimation Configuration: " << std::endl;
    std::cout << "- PlaneEstimate.num_ransac_iter: " << cfg.yaml_node_["PlaneEstimate.num_ransac_iter"].as<double>(-1) << std::endl;
    std::cout << "- PlaneEstimate.thres_inlier: " << cfg.yaml_node_["PlaneEstimate.thres_inlier"].as<double>(-1) << std::endl;

    return os;
}

} // namespace openvslam
