#include "slam_node.h"

// util
#include <spdlog/spdlog.h>
#include <popl.hpp>


using namespace std;


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "localization_from_rosbag_node");


    // create options
    popl::OptionParser op("Allowed options");
    auto help               = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path    = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto setting_file_path  = op.add<popl::Value<std::string>>("c", "config", "setting file path");
    auto image_topic        = op.add<popl::Value<std::string>>("i", "image", "color image topic name");
    auto depth_topic        = op.add<popl::Value<std::string>>("d", "depth", "depth image topic name");
    auto mask_img_path      = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");

    auto debug_mode         = op.add<popl::Switch>("", "debug", "debug mode");
    // auto is_kinect2         = op.add<popl::Switch>("", "kinect2", "set when xavier and kinect2");
    auto eval_log           = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    
    auto is_mapping         = op.add<popl::Value<bool>>("m", "is_mapping", "wheter or mapping in addition to prebuild map");
    auto map_db_path        = op.add<popl::Value<std::string>>("", "map-db", "path to prebuild map db file which is used to localization", "");
    auto log_dir_path       = op.add<popl::Value<std::string>>("l", "log-dir", "directory to save log", "");

    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !setting_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(setting_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    openvslam_ros::slam_node node(
        cfg, 
        vocab_file_path->value(), 
        log_dir_path->value(), 
        image_topic->value(), 
        depth_topic->value(), 
        map_db_path->value()
        );

    node.localization(is_mapping->value());

    return EXIT_SUCCESS;
}