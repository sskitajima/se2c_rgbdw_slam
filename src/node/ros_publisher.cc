#include "ros_publisher.h"
#include "se2c_rgbdw_slam/data/keyframe.h"

#include <opencv2/opencv.hpp>

namespace openvslam_ros
{
ros_publisher::ros_publisher(const ros::NodeHandle &nh)
    : nh_(nh), it_(nh_)
{
    pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/openvslam/pose", 100);
    pub_landmark_ = nh_.advertise<sensor_msgs::PointCloud2>("/openvslam/landmark", 100);
    pub_current_frame_ = it_.advertise("/openvslam/current_frame", 100);
    pub_octomap_ = nh_.advertise<octomap_msgs::Octomap>("/openvslam/octomap", 100);
    pub_kf_traj_ = nh_.advertise<geometry_msgs::PoseArray>("/openvslam/kf_trajectory", 100);
}

void ros_publisher::set_map_publisher(const std::shared_ptr<openvslam::publish::map_publisher> map_publisher)
{
    map_publisher_ = map_publisher;
}

void ros_publisher::set_transform(const std::shared_ptr<tf::StampedTransform>& transform_ptr)
{
    transform_rc_ = transform_ptr;
}

void ros_publisher::set_coord_transformer(const std::shared_ptr<openvslam::util::coordinate_transformer>& coord_transformer)
{
    coord_transformer_ = coord_transformer;
}

void ros_publisher::publish_pose(const openvslam::Mat44_t &pose_irr, const openvslam::Mat66_t &cov, const double time)
{
    // const auto timestamp = ros::Time(time);
    const auto timestamp = ros::Time::now();

    tf::Transform tf_irr = ::openvslam_ros::converter::Eigen2TF(pose_irr);    

    // /odom -> /base_footprint をpublish
    robot_base_publisher_.sendTransform(
            tf::StampedTransform(
                tf_irr,
                timestamp,
                "/odom", 
                "/base_footprint"
                )
    );

    publish_keyframes();
}

void ros_publisher::publish_keyframes()
{
    geometry_msgs::PoseArray pose_array_msgs;
    pose_array_msgs.header.frame_id = "/odom";
    pose_array_msgs.header.stamp = ros::Time::now();

    std::vector<openvslam::data::keyframe*> all_keyfrms;
    const int num_keyfrms = map_publisher_->get_keyframes(all_keyfrms);

    for(auto keyfrm : all_keyfrms)
    {
        geometry_msgs::Pose pose;
        auto pose_cir = keyfrm->get_cam_pose();
        openvslam::Mat44_t pose_irr_r = Eigen::PartialPivLU<openvslam::Mat44_t>(coord_transformer_->get_rc() * pose_cir).inverse();

        // translational factor
        pose.position.x = pose_irr_r(0, 3);
        pose.position.y = pose_irr_r(1, 3);
        pose.position.z = pose_irr_r(2, 3);

        // rotational factor
        const openvslam::Mat33_t rot = pose_irr_r.block(0,0,3,3);
        const openvslam::Quat_t quat_eigen(rot);
        pose.orientation.w = quat_eigen.w(); 
        pose.orientation.x = quat_eigen.x();
        pose.orientation.y = quat_eigen.y();
        pose.orientation.z = quat_eigen.z();

        pose_array_msgs.poses.push_back(pose);
    }

    pub_kf_traj_.publish(pose_array_msgs);
}

void ros_publisher::publish_landmark()
{
    std::vector<openvslam::data::landmark *> all_landmarks;
    std::set<openvslam::data::landmark *> local_landmarks;
    map_publisher_->get_landmarks(all_landmarks, local_landmarks);

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

    //点群の大きさを指定
    pcl_cloud.width = all_landmarks.size();
    pcl_cloud.height = 1;
    pcl_cloud.points.resize(pcl_cloud.width * pcl_cloud.height);

    //点群の生成
    for (unsigned int i = 0; i < pcl_cloud.width; i++)
    {
        Eigen::Vector3d pos = all_landmarks[i]->get_pos_in_world();
        pcl_cloud.points[i].x = pos(0);
        pcl_cloud.points[i].y = pos(1);
        pcl_cloud.points[i].z = pos(2);

    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(pcl_cloud, cloud_msg);

    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "/odom";
    cloud_msg.header.seq = cloud_seq_++;

    pub_landmark_.publish(cloud_msg);
}

void ros_publisher::publish_current_frame(const cv::Mat& img)
{
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    pub_current_frame_.publish(msg);
}

void ros_publisher::publish_octomap(const octomap::OcTree* octomap_tree_ )
{
    octomap_msgs::Octomap map_msg;
    octomap_msgs::binaryMapToMsg( *octomap_tree_, map_msg );
    map_msg.header.frame_id = "/odom";
    map_msg.header.stamp = ros::Time::now();
    pub_octomap_.publish ( map_msg );
}

} // namespace openvslam_ros