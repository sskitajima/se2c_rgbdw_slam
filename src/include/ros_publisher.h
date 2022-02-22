#ifndef OPENVSLAM_ROS_ROS_PUBLISHER_H
#define OPENVSLAM_ROS_ROS_PUBLISHER_H

#include "util.h"
#include "se2c_rgbdw_slam/type.h"
#include "se2c_rgbdw_slam/publish/map_publisher.h"
#include "se2c_rgbdw_slam/data/landmark.h"
#include "se2c_rgbdw_slam/util/coordinate_transformer.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <octomap/octomap.h>
#include <pcl_ros/point_cloud.h>

namespace openvslam_ros
{

class ros_publisher
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_landmark_;
    ros::Publisher pub_octomap_;
    ros::Publisher pub_kf_traj_;
    image_transport::Publisher pub_current_frame_; 

    // tf
    tf::TransformBroadcaster robot_base_publisher_;
    std::shared_ptr<tf::StampedTransform> transform_rc_ = nullptr;
    std::shared_ptr<openvslam::util::coordinate_transformer> coord_transformer_ = nullptr;

    std::shared_ptr<openvslam::publish::map_publisher> map_publisher_ = nullptr;
    unsigned cloud_seq_ = 0;
    unsigned pose_seq_ = 0;



public:
    ros_publisher(const ros::NodeHandle& nh);

    // setter
    void set_map_publisher(const std::shared_ptr<openvslam::publish::map_publisher> map_publisher);
    void set_transform(const std::shared_ptr<tf::StampedTransform>& transform_ptr);
    void set_coord_transformer(const std::shared_ptr<openvslam::util::coordinate_transformer>& coord_transformer);
    
    // publish
    void publish_pose(const openvslam::Mat44_t& pose, const openvslam::Mat66_t& cov, const double time);
    void publish_keyframes();
    void publish_landmark();
    void publish_current_frame(const cv::Mat& img);
    void publish_octomap(const octomap::OcTree* octomap_tree_);
};


}

#endif