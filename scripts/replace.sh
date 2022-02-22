#!/bin/bash

# sed -i.bak 's/Method.performance_mode: false/Method.performance_mode: true/' kinect2_new_*.yaml
# sed -i.bak 's/Debug.wheel_matching_num_path: "\/home\/kitajima\/catkin_ws\/src\/se2c_rgbdw_slam\/log\/num_matching_wheel.txt"//' kinect2_new_*.yaml
# sed -i.bak 's/Debug.wheel_pose_path: "\/home\/kitajima\/catkin_ws\/src\/se2c_rgbdw_slam\/log\/wheel_pose.txt"//' kinect2_new_*.yaml
# sed -i.bak 's/Debug.cam_pose_path: "\/home\/kitajima\/catkin_ws\/src\/se2c_rgbdw_slam\/log\/cam_pose.txt"//' kinect2_new_*.yaml
sed -i.bak "s/Encoder.displacement_covariance_factor: 0.01/Encoder.displacement_covariance_factor: 1000   # sigma_sq = factor * wheel_displacement/" kinect2_new_*.yaml