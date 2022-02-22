################################ SLAM FROM ROSBAG NODE ################################
add_executable(slam_from_rosbag_node
  slam_from_rosbag.cc
)

target_link_libraries(slam_from_rosbag_node
  ${catkin_LIBRARIES}
  se2c_rgbdw_slam_node_lib
)

################################ SLAM FROM DUMP NODE ################################
add_executable(slam_from_dump_node
  slam_from_dump.cc
)

target_link_libraries(slam_from_dump_node
  ${catkin_LIBRARIES}
  se2c_rgbdw_slam_node_lib
)


################################ LOCALIZATION FROM ROSBAG NODE ################################
add_executable(localization_from_rosbag_node
  localization_from_rosbag.cc
)

target_link_libraries(localization_from_rosbag_node
  ${catkin_LIBRARIES}
  se2c_rgbdw_slam_node_lib
)

################################ DUMMY CLOUD PUBLISHER ################################
add_executable(dummy_cloud_publisher
  dummy_cloud_publisher.cc
)

target_link_libraries(dummy_cloud_publisher
  ${catkin_LIBRARIES}
)
