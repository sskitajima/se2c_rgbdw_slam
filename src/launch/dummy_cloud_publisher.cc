#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "dummpy_cloud_publisher_node");
    ros::NodeHandle nh;
    ros::Publisher pub_landmark = nh.advertise<sensor_msgs::PointCloud2>("/openvslam/landmark_dummy", 100);
    ros::Rate rate(5);
    int seq = 0;
    
    // 原点に、ダミーの点を1つpublishする
    // 点群の大きさを指定
    while(ros::ok())
    {
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl_cloud.width = 1;
        pcl_cloud.height = 1;
        pcl_cloud.points.resize(pcl_cloud.width * pcl_cloud.height);

        // 点群の生成
        pcl_cloud.points[0].x = 0;
        pcl_cloud.points[0].y = 0;
        pcl_cloud.points[0].z = 0;

        // rosのmsgに変換
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(pcl_cloud, cloud_msg);

        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "/map";
        cloud_msg.header.seq = seq++;

        pub_landmark.publish(cloud_msg);

        ROS_INFO("publish dummy landmark");

        rate.sleep();

        if(seq > 5)  break;
    }   

    ROS_INFO("terminate dummpy_cloud_publisher_node");
    return 0;
}