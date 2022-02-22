# SE2C_RGBDW_SLAM

SE(2)-constrained RGBD-Wheel SLAM system

## Build
- Prerequirements
  - Ubuntu 18.04
  - ROS Melodic

- For detail instruction, see `doc/build.md`
- This repository should be cloned at `~/catkin_ws`



## How to run

- launch from rosbag data

```
roslaunch se2c_rgbdw_slam slam_from_rosbag_node.launch
```

- launch from dumped data

```
roslaunch se2c_rgbdw_slam slam_from_dump_node.launch
```



## ROS topic

```bash
Publications: 
 * /openvslam/current_frame [sensor_msgs/Image]
 * /openvslam/kf_trajectory [geometry_msgs/PoseArray]
 * /openvslam/octomap [octomap_msgs/Octomap]
 * /openvslam/pose [geometry_msgs/PoseWithCovarianceStamped]

Subscriptions: 
 * /encoder [geometry_msgs/QuaternionStamped]
 * /kinect2/qhd/image_color [sensor_msgs/Image]
 * /kinect2/qhd/image_depth_rect [sensor_msgs/Image]

```

- Publications
  - `/openvslam/pose`: /geometry_msgs/PoseWithCovarianceStamped
    current pose
  - `/openvslam/current_frame`: sensor_msgs/Image
    current tracking frame with feature points
- Subscriptions
  - `/kinect2/qhd/image_color`: sensor_msgs/Image
    input RGB images
  - `/kinect2/qhd/image_depth_rect`: sensor_msgs/Image
    input Depth images
  - `/encoder`: geometry_msgs/Quaternion
    encoder pulses.



## Parameter files

- bag of words: `data/orb_vocab/orb_vocab.dbow2`
- system parameters: `data/*.yaml`
  - camera parameters
  - odometry parameters
  - feature matching parameters
  - SE2C parameters
  
- rviz parameters: `rviz/kinect.rviz`
- simulated robot model: `urdf/`
- launch settings: `launch/`



## LICENSE

GPLv3 (see `LICENSE`)




## Related works
- [OpenVSLAM(original)](https://github.com/xdspacelab/openvslam)
- [OpenVSLAM(current)](https://github.com/OpenVSLAM-Community/openvslam)
```
Shinya Sumikura, Mikiya Shibuya and Ken Sakurada, OpenVSLAM: A Versatile Visual SLAM Framework, Proceedings of the 27th ACM International Conference on Multimedia (2019), pp.2292-2295.
```

- [OBR-SLAM2](https://github.com/raulmur/ORB_SLAM2)
```
Raúl Mur-Artal and Juan D. Tardós, ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras, IEEE Transactions on Robotics, Vol.33, No.5 (2017), pp.1255–1262.
```

- [se2clam](https://github.com/izhengfan/se2clam)
```
Fan Zheng, Hengbo Tang, Yun-Hui Liu. Odometry-Vision-Based Ground Vehicle Motion Estimation With SE(2)-Constrained SE(3) Poses, IEEE Transactions on Cybernetics, Vol. 49, No. 7, (2019), pp.2652-2663.
```