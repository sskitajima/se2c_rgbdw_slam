#include "se2c_rgbdw_slam/data/landmark.h"
#include "se2c_rgbdw_slam/data/keyframe.h"
#include "se2c_rgbdw_slam/data/map_database.h"
#include "se2c_rgbdw_slam/publish/map_publisher.h"

#include <spdlog/spdlog.h>

namespace openvslam {
namespace publish {

map_publisher::map_publisher(const std::shared_ptr<config>& cfg, data::map_database* map_db)
    : cfg_(cfg), map_db_(map_db) {
    spdlog::debug("CONSTRUCT: publish::map_publisher");
}

map_publisher::~map_publisher() {
    spdlog::debug("DESTRUCT: publish::map_publisher");
}

void map_publisher::set_current_cam_pose(const Mat44_t& cam_pose_cw) {
    // spdlog::debug("map_publisher::set_current_cam_pose start");
    std::lock_guard<std::mutex> lock(mtx_cam_pose_);
    // cam_pose_cw_ = cam_pose_cw;
    cam_pose_cw_(0,0) = cam_pose_cw(0,0);
    cam_pose_cw_(0,1) = cam_pose_cw(0,1);
    cam_pose_cw_(0,2) = cam_pose_cw(0,2);
    cam_pose_cw_(0,3) = cam_pose_cw(0,3);
    cam_pose_cw_(1,0) = cam_pose_cw(1,0);
    cam_pose_cw_(1,1) = cam_pose_cw(1,1);
    cam_pose_cw_(1,2) = cam_pose_cw(1,2);
    cam_pose_cw_(1,3) = cam_pose_cw(1,3);
    cam_pose_cw_(2,0) = cam_pose_cw(2,0);
    cam_pose_cw_(2,1) = cam_pose_cw(2,1);
    cam_pose_cw_(2,2) = cam_pose_cw(2,2);
    cam_pose_cw_(2,3) = cam_pose_cw(2,3);
    cam_pose_cw_(3,0) = cam_pose_cw(3,0);
    cam_pose_cw_(3,1) = cam_pose_cw(3,1);
    cam_pose_cw_(3,2) = cam_pose_cw(3,2);
    cam_pose_cw_(3,3) = cam_pose_cw(3,3);
}

Mat44_t map_publisher::get_current_cam_pose() {
    std::lock_guard<std::mutex> lock(mtx_cam_pose_);
    return cam_pose_cw_;
}

unsigned int map_publisher::get_keyframes(std::vector<data::keyframe*>& all_keyfrms) {
    all_keyfrms = map_db_->get_all_keyframes();
    return map_db_->get_num_keyframes();
}

unsigned int map_publisher::get_landmarks(std::vector<data::landmark*>& all_landmarks,
                                          std::set<data::landmark*>& local_landmarks) {
    all_landmarks = map_db_->get_all_landmarks();
    const auto _local_landmarks = map_db_->get_local_landmarks();
    local_landmarks = std::set<data::landmark*>(_local_landmarks.begin(), _local_landmarks.end());
    return map_db_->get_num_landmarks();
}

} // namespace publish
} // namespace openvslam
