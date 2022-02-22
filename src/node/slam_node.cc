#include "slam_node.h"
#include "se2c_rgbdw_slam/publish/frame_publisher.h"

#include <tf/transform_listener.h>

#include <spdlog/spdlog.h>

using namespace std;

namespace openvslam_ros
{

slam_node::slam_node( const std::shared_ptr<openvslam::config>& cfg, 
                          const std::string& vocab_file_path, 
                          const std::string& log_dir_path,
                          const std::string& image_topic,
                          const std::string& depth_topic,
                          const std::string& prebuild_map_path
                          )
    : nh_("~"), system_(cfg, vocab_file_path, log_dir_path), cfg_(cfg), 
      it_rgb_(nh), it_img_filter_(nh), it_depth_filter_(nh), 
      topic_save_image_(image_topic), topic_save_depth_(depth_topic),
      ros_publisher_(nh),
      use_pulse_encoder_(cfg_->yaml_node_["Encoder.use_pulse_encoder"].as<bool>(false)), 
      use_kinect2_(cfg_->yaml_node_["Method.use_kinect2"].as<bool>(true)), 
      use_compressed_image_(cfg_->yaml_node_["Method.use_compressed_image"].as<bool>(true)),
      is_performance_mode_(cfg->yaml_node_["Method.performance_mode"].as<bool>(true))
{
    init();

    if(prebuild_map_path=="")
    {
        system_.startup(true);
    }
    else
    {
        system_.load_map_database(prebuild_map_path);
        system_.startup(false);
    }
}

slam_node::~slam_node()
{
}

void slam_node::init()
{
    // odom sucscribe
    if(use_pulse_encoder_)
    {   
        const double wheel_sep = cfg_->yaml_node_["Encoder.wheel_separation"].as<double>(0.265);
        const double pulse_per_m = cfg_->yaml_node_["Encoder.num_pulse_per_meter"].as<double>(5120.0 / (M_PI * 2 * 0.075));
        const double displacement_covariance_factor = cfg_->yaml_node_["Encoder.displacement_covariance_factor"].as<double>(0.1);

        system_.use_pulse_encoder(use_pulse_encoder_);

        // subscribe encoder topic only if the image topic name is not empty
        if( topic_save_image_ != ""  || topic_save_depth_ != "") sub_odom_ = nh_.subscribe<geometry_msgs::QuaternionStamped>("/encoder", queue_size_odom_, &slam_node::encoderCallback, this);
        system_.set_encoder_param(wheel_sep, pulse_per_m, displacement_covariance_factor);
    }
    else
    {
        if( topic_save_image_ != ""  || topic_save_depth_ != "") sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odom", queue_size_odom_, &slam_node::odometryCallback, this);
    }

    std::string topic_hints;
    if(use_compressed_image_)
    {
        topic_hints = "compressed";
    }
    else
    {
        topic_hints = "raw";
    }

    // subscribe encoder topic only if the image topic name is not empty
    if( topic_save_image_ != ""  || topic_save_depth_ != "" )
    {
        image_transport::TransportHints img_hints(topic_hints.c_str(), ros::TransportHints(), nh_, "img_hints");
        image_transport::TransportHints depth_hints(topic_hints.c_str(), ros::TransportHints(), nh_, "depth_hints");
        sub_img_filter_.subscribe(it_img_filter_, topic_save_image_.c_str(), queue_size_img_, img_hints);
        sub_depth_filter_.subscribe(it_depth_filter_, topic_save_depth_.c_str(), queue_size_img_, depth_hints);

        sync_.reset(new Synchronizer(SyncPolicy(queue_size_img_), sub_img_filter_, sub_depth_filter_));
        sync_->registerCallback(boost::bind(&slam_node::rgbdCallback, this, _1, _2));
    }

    // ros map publisher
    ros_publisher_.set_map_publisher(system_.get_map_publisher());

    // get octomap mapping module
    octomap_mapping_ = system_.get_octomap_mapping_module();

    auto coord_transformer = system_.get_coord_transformer();   
    ros_publisher_.set_coord_transformer(coord_transformer);
}

void slam_node::odometryCallback(const nav_msgs::OdometryConstPtr &odom_ptr)
{
    spdlog::debug("odometryCallback()");

    const double time = odom_ptr->header.stamp.sec + (1e-9) * odom_ptr->header.stamp.nsec;
    const double v = odom_ptr->twist.twist.linear.x;
    const double omega = odom_ptr->twist.twist.angular.z;

    double delta;
    if(last_time_ == 0)
    {
        last_time_ = time;
    }
    else
    {  
        delta = time - last_time_;
        last_time_ = time;
        system_.add_encoder_measurement(v, omega, delta);
    }
}

void slam_node::encoderCallback(const geometry_msgs::QuaternionStampedConstPtr &enc_ptr)
{
    spdlog::debug("encoderCallback()");
    sub_num_enc_++;

    const double time = enc_ptr->header.stamp.sec + (1e-9) * enc_ptr->header.stamp.nsec;
    const double enc_l = (enc_ptr->quaternion.x + enc_ptr->quaternion.y) / 2.0;
    const double enc_r = (enc_ptr->quaternion.z + enc_ptr->quaternion.w) / 2.0;

    double delta;
    if(last_time_enc_ == 0)
    {
        last_time_enc_ = time;
    }
    else
    {  
        delta = time - last_time_enc_;
        last_time_enc_ = time;
        system_.add_encoder_measurement(enc_r, enc_l, delta, time);
    }
}

void slam_node::rgbdCallback(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{
    spdlog::debug("rgbdCallback()");
    sub_num_rgbd_++;

    const double time = img_msg->header.stamp.sec + (1e-9) * img_msg->header.stamp.nsec;

    // subした画像の処理
    cv::Mat rgb_img = cv_bridge::toCvShare(img_msg, "bgr8")->image;

    // depth画像のスケールファクタを揃える
    cv::Mat depth_16UC1_img;
    if(use_kinect2_)
    {
        // 実機のkinect(iai_kinect経由)では、16UC1型で、mm単位の数値として格納されている
        cv::Mat depth_raw_img = cv_bridge::toCvShare(depth_msg, "16UC1")->image;
        depth_raw_img.convertTo(depth_16UC1_img, CV_16UC1, cfg_->yaml_node_["depthmap_factor"].as<double>(5000.0) / 1000.0);
    }
    else
    {
        // Gazeboのシミュレーション、及び、TUMデータセットのrosbag形式では、32FC1型で、m単位の実際の値が画像に格納されている
        cv::Mat depth_32FC1_img = cv_bridge::toCvShare(depth_msg, "32FC1")->image;
        depth_32FC1_img.convertTo(depth_16UC1_img, CV_16UC1, cfg_->yaml_node_["depthmap_factor"].as<double>(5000.0));
    }

    system_.add_rgbd_frame(rgb_img, depth_16UC1_img, time);
}

void slam_node::add_encoder_measurement(const double wheel_r, const double wheel_l, const double delta, const double time)
{
    system_.add_encoder_measurement(wheel_r, wheel_l, delta, time);
}

void slam_node::add_rgbd_frame(const cv::Mat &rgb_img, const cv::Mat &depth_img, const double time)
{
    system_.add_rgbd_frame(rgb_img, depth_img, time);
}

void slam_node::request_terminate_system()
{
    system_.request_terminate();
}

void slam_node::request_shutdown_system()
{
    system_.shutdown();
}


openvslam::Mat44_t slam_node::mainprocess()
{
    openvslam::Mat44_t cam_pose_cir = system_.mainprocess();
    return cam_pose_cir;
}

bool slam_node::publish_ros()
{
    openvslam::Mat44_t pose;
    openvslam::Mat66_t cov;
    double time;
    bool success = system_.get_publish_poses(pose, cov, time);
    if(success){
        ros_publisher_.publish_pose(pose, cov, time);
        // ros_publisher_.publish_landmark();

        std::shared_ptr<openvslam::publish::frame_publisher> frame_publisher = system_.get_frame_publisher();
        cv::Mat img = frame_publisher->draw_frame();
        ros_publisher_.publish_current_frame(img);

        auto octomap_tree = octomap_mapping_->get_octomap_tree();
        ros_publisher_.publish_octomap(octomap_tree);
    } 

    return success;
}

void slam_node::mainloop()
{
    ros::Rate r(publish_rate_);

    while(ros::ok())
    {
        if(!is_performance_mode_) publish_ros();

        ros::spinOnce();
        r.sleep();
    }

    spdlog::info("sub_num_rgbd_          {}", sub_num_rgbd_);
    spdlog::info("sub_num_enc_           {}", sub_num_enc_);

    system_.request_terminate();
    system_.shutdown();
}

void slam_node::localization(const bool is_mapping)
{
    if (is_mapping) {
        system_.enable_mapping_module();
    }
    else {
        system_.disable_mapping_module();
    }
    ros_publisher_.publish_landmark();
    mainloop();
}

} // namespace openvslam_ros