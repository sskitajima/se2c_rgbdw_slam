#ifndef OPENVSLAM_MODULE_FRAME_TRACKER_H
#define OPENVSLAM_MODULE_FRAME_TRACKER_H

#include "se2c_rgbdw_slam/type.h"
#include "se2c_rgbdw_slam/optimize/pose_optimizer.h"

namespace openvslam {

namespace camera {
class base;
} // namespace camera

namespace data {
class frame;
class keyframe;
} // namespace data

namespace module {

class frame_tracker {
public:
    explicit frame_tracker(camera::base* camera, const unsigned int num_matches_thr = 20);

    bool motion_based_track(data::frame& curr_frm, const data::frame& last_frm, const Mat44_t& velocity) const;

    bool bow_match_based_track(data::frame& curr_frm, const data::frame& last_frm, data::keyframe* ref_keyfrm) const;

    bool robust_match_based_track(data::frame& curr_frm, const data::frame& last_frm, data::keyframe* ref_keyfrm) const;

    void set_se2c_wrappers(std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper>& plane_wrapper, std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper>& odom_wrapper);

private:
    unsigned int discard_outliers(data::frame& curr_frm) const;

    const camera::base* camera_;
    const unsigned int num_matches_thr_;

    optimize::pose_optimizer pose_optimizer_;
};

} // namespace module
} // namespace openvslam

#endif // OPENVSLAM_MODULE_FRAME_TRACKER_H
