set(SE2C_RGBDW_SLAM_SRC_DIR ${OpenVSLAM_ROOT}/src/se2c_rgbdw_slam)

##############################################
## source files

set(SE2C_RGBDW_SLAM_SRC_FILES "")

list(APPEND SE2C_RGBDW_SLAM_LIB_SRC_FILES

  ${SE2C_RGBDW_SLAM_SRC_DIR}/system.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/system.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/tracking_module.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/tracking_module.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/mapping_module.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/mapping_module.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/global_optimization_module.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/global_optimization_module.cc

  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/encoder.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/rgbd_frame.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/rgbd_frame_set.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/rgbd_frame_set.cc

  # all
  ${SE2C_RGBDW_SLAM_SRC_DIR}/config.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/config.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/type.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/config_se2_constraint.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/config_se2_constraint.cc

  # util
  ${SE2C_RGBDW_SLAM_SRC_DIR}/util/converter.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/util/image_converter.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/util/random_array.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/util/converter.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/util/image_converter.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/util/random_array.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/util/stereo_rectifier.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/util/quaternion.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/util/quaternion.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/util/coordinate_transformer.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/util/coordinate_transformer.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/util/se2c.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/util/se2c.cc

  # solver
  ${SE2C_RGBDW_SLAM_SRC_DIR}/solve/common.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/solve/homography_solver.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/solve/fundamental_solver.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/solve/essential_solver.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/solve/pnp_solver.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/solve/sim3_solver.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/solve/common.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/solve/homography_solver.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/solve/fundamental_solver.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/solve/essential_solver.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/solve/pnp_solver.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/solve/sim3_solver.cc

  # publish
  ${SE2C_RGBDW_SLAM_SRC_DIR}/publish/frame_publisher.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/publish/map_publisher.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/publish/frame_publisher.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/publish/map_publisher.cc

  # optimize
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/pose_optimizer.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/local_bundle_adjuster.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/transform_optimizer.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/graph_optimizer.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/global_bundle_adjuster.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/pose_optimizer.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/local_bundle_adjuster.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/transform_optimizer.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/graph_optimizer.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/global_bundle_adjuster.cc

  # g2o
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/landmark_vertex.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/landmark_vertex_container.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/landmark_vertex.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/landmark_vertex_container.cc
  
  # se3
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se3/shot_vertex.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se3/shot_vertex_container.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se3/perspective_pose_opt_edge.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se3/perspective_reproj_edge.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se3/equirectangular_pose_opt_edge.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se3/equirectangular_reproj_edge.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se3/shot_vertex.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se3/shot_vertex_container.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se3/perspective_pose_opt_edge.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se3/perspective_reproj_edge.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se3/equirectangular_pose_opt_edge.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se3/equirectangular_reproj_edge.cc

  # sim3
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/sim3/shot_vertex.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/sim3/graph_opt_edge.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/sim3/transform_vertex.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/sim3/backward_reproj_edge.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/sim3/forward_reproj_edge.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/sim3/shot_vertex.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/sim3/graph_opt_edge.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/sim3/transform_vertex.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/sim3/backward_reproj_edge.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/sim3/forward_reproj_edge.cc

  # se2constraint
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se2constraint/plane_constraint_edge.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se2constraint/plane_constraint_edge.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se2constraint/plane_constraint_wrapper.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se2constraint/plane_constraint_wrapper.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se2constraint/odometry_constraint_edge.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se2constraint/odometry_constraint_edge.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se2constraint/odometry_constraint_wrapper.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/optimize/g2o/se2constraint/odometry_constraint_wrapper.cc

  # module
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/initializer.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/relocalizer.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/frame_tracker.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/keyframe_inserter.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/two_view_triangulator.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/local_map_cleaner.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/loop_detector.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/loop_bundle_adjuster.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/initializer.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/relocalizer.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/frame_tracker.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/keyframe_inserter.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/two_view_triangulator.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/local_map_cleaner.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/loop_detector.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/loop_bundle_adjuster.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/octomap_mapping.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/plane_estimator.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/encoder_odometry.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/module/encoder_odometry.cc
  
  # match
  ${SE2C_RGBDW_SLAM_SRC_DIR}/match/area.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/match/bow_tree.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/match/fuse.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/match/projection.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/match/robust.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/match/stereo.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/match/area.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/match/bow_tree.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/match/fuse.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/match/projection.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/match/robust.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/match/stereo.cc

  # io
  ${SE2C_RGBDW_SLAM_SRC_DIR}/io/trajectory_io.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/io/map_database_io.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/io/trajectory_io.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/io/map_database_io.cc

  # initialize
  ${SE2C_RGBDW_SLAM_SRC_DIR}/initialize/base.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/initialize/perspective.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/initialize/bearing_vector.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/initialize/base.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/initialize/perspective.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/initialize/bearing_vector.cc

  # feature
  ${SE2C_RGBDW_SLAM_SRC_DIR}/feature/orb_params.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/feature/orb_extractor.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/feature/orb_extractor_node.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/feature/orb_params.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/feature/orb_extractor.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/feature/orb_extractor_node.cc

  # data
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/bow_vocabulary.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/common.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/frame.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/keyframe.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/landmark.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/graph_node.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/camera_database.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/map_database.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/bow_database.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/frame_statistics.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/common.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/frame.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/keyframe.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/landmark.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/graph_node.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/camera_database.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/map_database.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/bow_database.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/data/frame_statistics.cc

  # camera
  ${SE2C_RGBDW_SLAM_SRC_DIR}/camera/base.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/camera/perspective.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/camera/fisheye.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/camera/equirectangular.h
  ${SE2C_RGBDW_SLAM_SRC_DIR}/camera/base.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/camera/perspective.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/camera/fisheye.cc
  ${SE2C_RGBDW_SLAM_SRC_DIR}/camera/equirectangular.cc

)

##############################################
## add library

add_library(se2c_rgbdw_slam_node_lib
  node/slam_node.cc
  node/ros_publisher.cc
  node/util.cc
)

foreach(SRC_FILE IN LISTS SE2C_RGBDW_SLAM_LIB_SRC_FILES)
    # message(STATUS "test SRC_FILES: ${SRC_FILE}")
    target_sources(se2c_rgbdw_slam_node_lib
        PRIVATE
        ${SRC_FILE}
    )
endforeach()

##############################################
## include 

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenVSLAM_SRC_DIR}
        ${OpenVSLAM_SRC_DIR}/include
)

target_include_directories( se2c_rgbdw_slam_node_lib
    PUBLIC

    # openvslam
    $<BUILD_INTERFACE:${json_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${spdlog_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${BoW_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/se2c_rgbdw_slam>
    $<INSTALL_INTERFACE:include/openvslam/3rd/json/include>
    $<INSTALL_INTERFACE:include/openvslam/3rd/spdlog/include>
    $<INSTALL_INTERFACE:${BoW_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include/>
)

##############################################
## link

target_link_libraries(se2c_rgbdw_slam_node_lib
    Threads::Threads
    Eigen3::Eigen
    yaml-cpp
    opencv_core
    opencv_features2d
    opencv_calib3d
    g2o::core
    g2o::stuff
    g2o::types_sba
    g2o::types_sim3
    g2o::solver_dense
    g2o::solver_eigen
    g2o::solver_csparse
    g2o::csparse_extension
    ${CXSPARSE_LIBRARIES}
    ${SUITESPARSE_LIBRARIES}
    ${BOW_LIBS}

    ${catkin_LIBRARIES} 

    ${OCTOMAP_LIBRARIES}
    stdc++fs
)

##############################################
## Compile configuration

# OpenMP
set(G2O_USE_OPENMP ON CACHE BOOL "Use OpenMP in G2O")
# set(USE_OPENMP OFF CACHE BOOL "Use OpenMP")
set(USE_OPENMP ON CACHE BOOL "Use OpenMP")
if(USE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_FOUND)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        target_compile_definitions(se2c_rgbdw_slam_node_lib PUBLIC USE_OPENMP)
        message(STATUS "OpenMP: ENABLED")
    else()
        message(STATUS "OpenMP: DISABLED")
    endif()
else()
    message(STATUS "OpenMP: DISABLED")
endif()

set(USE_SSE_ORB OFF CACHE BOOL "Enable SSE3 instruction for ORB extraction (-msse3)")
if(USE_SSE_ORB)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msse3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3")
    target_compile_definitions(se2c_rgbdw_slam_node_lib PUBLIC USE_SSE_ORB)
    message(STATUS "SSE3 for ORB extraction (-msse3): ENABLED")
else()
    message(STATUS "SSE3 for ORB extraction (-msse3): DISABLED")
endif()

set(USE_SSE_FP_MATH OFF CACHE BOOL "Enable SSE instruction for floating-point operation (-mfpmath=sse)")
if(USE_SSE_FP_MATH)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfpmath=sse")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpmath=sse")
    message(STATUS "SSE for floating-point operation (-mfpmath=sse): ENABLED")
else()
    message(STATUS "SSE for floating-point operation (-mfpmath=sse): DISABLED")
endif()

# Check BoW Framework
set(BOW_FRAMEWORK "DBoW2" CACHE STRING "DBoW2 or FBoW")
set_property(CACHE BOW_FRAMEWORK PROPERTY STRINGS "DBoW2" "FBoW")
if(BOW_FRAMEWORK MATCHES "DBoW2")
    find_package(DBoW2 REQUIRED)
    set(BoW_INCLUDE_DIR ${DBoW2_INCLUDE_DIRS})
    set(BoW_LIBRARY ${DBoW2_LIBS})
    target_compile_definitions(se2c_rgbdw_slam_node_lib PUBLIC USE_DBOW2)
elseif(BOW_FRAMEWORK MATCHES "FBoW")
    find_package(fbow REQUIRED)
    set(BoW_INCLUDE_DIR ${fbow_INCLUDE_DIRS})
    set(BoW_LIBRARY ${fBoW_LIBS})
else()
    message(FATAL_ERROR "Invalid BoW framework: ${BOW_FRAMEWORK}")
endif()
message(STATUS "BoW framework: ${BOW_FRAMEWORK} (found in ${BoW_INCLUDE_DIR})")

##############################################

# add_subdirectory(test)
add_subdirectory(launch)