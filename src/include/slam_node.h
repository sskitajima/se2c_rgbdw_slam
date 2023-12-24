#ifndef OPENVSLAM_ROS_SLAM_NODE_H
#define OPENVSLAM_ROS_SLAM_NODE_H

// oepnvslam
#include "se2c_rgbdw_slam/system.h"
#include "se2c_rgbdw_slam/module/octomap_mapping.h"
#include "se2c_rgbdw_slam/config.h"
#include "se2c_rgbdw_slam/type.h"
#include "ros_publisher.h"

// ros
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

namespace openvslam_ros
{

class slam_node
{
private:
    const ros::NodeHandle nh;
    ros::NodeHandle nh_;

    openvslam::system system_;
    const std::shared_ptr<openvslam::config>& cfg_;

    // subscriber
    ros::Subscriber sub_odom_;
    image_transport::ImageTransport it_rgb_;
    image_transport::ImageTransport it_img_filter_, it_depth_filter_;
    image_transport::SubscriberFilter sub_depth_filter_, sub_img_filter_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> sync_;

    std::string topic_save_image_;
    std::string topic_save_depth_;

    // publisher
    ros_publisher ros_publisher_;

    // octomap
    openvslam::module::octomap_mapping* octomap_mapping_;

    // last time to recieve odometry topic
    double last_time_ = 0;
    double last_time_enc_ = 0;

    bool use_pulse_encoder_;
    bool use_kinect2_;
    bool use_compressed_image_;
    bool is_performance_mode_;
    
    // rgbd sync sucscribe
    int queue_size_img_ = 200;
    int queue_size_odom_ = 200;
    int publish_rate_ = 30;

    // debug
    int sub_num_rgbd_ = 0;
    int sub_num_enc_ = 0;



public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
       
    slam_node( const std::shared_ptr<openvslam::config>& cfg, 
                 const std::string& vocab_file_path, 
                 const std::string& log_dir_path,
                 const std::string& image_topic,
                 const std::string& depth_topic,
                 const std::string& prebuild_map_path=""
                );
    ~slam_node();

    void init();

    void odometryCallback(const nav_msgs::OdometryConstPtr& odom_ptr);
    void encoderCallback(const geometry_msgs::QuaternionStampedConstPtr& enc_ptr);
    void rgbdCallback(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::ImageConstPtr &depth_msg);

    void mainloop();
    void localization(const bool is_mapping);

    void add_encoder_measurement(const double wheel_r, const double wheel_l, const double delta, const double time);
    void add_rgbd_frame(const cv::Mat &rgb_img, const cv::Mat &depth_img, const double time);

    void request_terminate_system();
    void request_shutdown_system();
    
    bool publish_ros();

    openvslam::Mat44_t mainprocess();

};

}


#endif 