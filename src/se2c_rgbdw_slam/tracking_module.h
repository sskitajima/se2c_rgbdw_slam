#ifndef OPENVSLAM_TRACKING_MODULE_H
#define OPENVSLAM_TRACKING_MODULE_H

#include "se2c_rgbdw_slam/type.h"
#include "se2c_rgbdw_slam/data/frame.h"
#include "se2c_rgbdw_slam/module/initializer.h"
#include "se2c_rgbdw_slam/module/relocalizer.h"
#include "se2c_rgbdw_slam/module/keyframe_inserter.h"
#include "se2c_rgbdw_slam/module/frame_tracker.h"

#include "se2c_rgbdw_slam/optimize/g2o/se2constraint/plane_constraint_wrapper.h"
#include "se2c_rgbdw_slam/optimize/g2o/se2constraint/odometry_constraint_wrapper.h"

#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace openvslam {

class system;
class mapping_module;
class global_optimization_module;

namespace util
{
class coordinate_transformer;
}

namespace data {
class map_database;
class bow_database;
} // namespace data

namespace feature {
class orb_extractor;
} // namespace feature

class tracking_module {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Constructor
    tracking_module(const std::shared_ptr<config>& cfg, system* system, data::map_database* map_db,
                    data::bow_vocabulary* bow_vocab, data::bow_database* bow_db);

    //! Destructor
    ~tracking_module();

    //! Set the mapping module
    void set_mapping_module(mapping_module* mapper);

    //! Set the global optimization module
    void set_global_optimization_module(global_optimization_module* global_optimizer);

    //-----------------------------------------
    // interfaces

    //! Set mapping module status
    void set_mapping_module_status(const bool mapping_is_enabled);

    //! Get mapping module status
    bool get_mapping_module_status() const;

    //! Get the keypoints of the initial frame
    std::vector<cv::KeyPoint> get_initial_keypoints() const;

    //! Get the keypoint matches between the initial frame and the current frame
    std::vector<int> get_initial_matches() const;

    Mat44_t visual_encoder_track(const cv::Mat& img, const cv::Mat& depthmap, const double timestamp, bool& is_keyframe, const Mat44_t& odom_from_ref_kf, const Mat66_t& cov_from_ref_kf, const cv::Mat& mask = cv::Mat{});

    //!! Proposed
    // set current frame pose as given
    void set_reference_frame_pose(const Mat44_t &pose_mat_cw);

    //!! Proposed
    Mat44_t get_reference_frame_pose() const;

    void set_depthmap_to_keyfrm(const cv::Mat& depthmap);

    void set_coord_transformer(std::shared_ptr<util::coordinate_transformer>& coord_transformer);
    void set_se2c_wrappers(std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper>& plane_wrapper, std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper>& odom_wrapper);

    //-----------------------------------------
    // management for reset process

    //! Reset the databases
    void reset();

    //-----------------------------------------
    // management for pause process

    //! Request to pause the tracking module
    void request_pause();

    //! Check if the pause of the tracking module is requested or not
    bool pause_is_requested() const;

    //! Check if the tracking module is paused or not
    bool is_paused() const;

    //! Resume the tracking module
    void resume();

    //-----------------------------------------
    // variables

    //! config
    const std::shared_ptr<config> cfg_;

    //! camera model (equals to cfg_->camera_)
    camera::base* camera_;

    //! latest tracking state
    tracker_state_t tracking_state_ = tracker_state_t::NotInitialized;
    //! last tracking state
    tracker_state_t last_tracking_state_ = tracker_state_t::NotInitialized;

    //! current frame and its image
    data::frame curr_frm_;
    //! image of the current frame
    cv::Mat img_gray_;

    //! elapsed microseconds for each tracking
    double elapsed_ms_ = 0.0;

    //! Proposed
    Mat44_t estimated_transform_irc_ = Mat44_t::Identity();

protected:
    //-----------------------------------------
    // tracking processes

    //! Try to initialize with the current frame
    bool initialize();

    //! Track the current frame
    bool track_current_frame();

    //! Update the motion model using the current and last frames
    void update_motion_model();

    //! Replace the landmarks if the `replaced` member has the valid pointer
    void apply_landmark_replace();

    //! Update the camera pose of the last frame
    void update_last_frame();

    //! Optimize the camera pose of the current frame
    bool optimize_current_frame_with_local_map();

    //! Update the local map
    void update_local_map();

    //! Update the local keyframes
    void update_local_keyframes();

    //! Update the local landmarks
    void update_local_landmarks();

    //! Acquire more 2D-3D matches using initial camera pose estimation
    void search_local_landmarks();

    //! Check the new keyframe is needed or not
    bool new_keyframe_is_needed() const;

    //! Insert the new keyframe derived from the current frame
    void insert_new_keyframe();

    //! system
    system* system_ = nullptr;
    //! mapping module
    mapping_module* mapper_ = nullptr;
    //! global optimization module
    global_optimization_module* global_optimizer_ = nullptr;

    // ORB extractors
    //! ORB extractor for left/monocular image
    feature::orb_extractor* extractor_left_ = nullptr;
    //! ORB extractor for right image
    feature::orb_extractor* extractor_right_ = nullptr;
    //! ORB extractor only when used in initializing
    feature::orb_extractor* ini_extractor_left_ = nullptr;

    //! map_database
    data::map_database* map_db_ = nullptr;

    // Bag of Words
    //! BoW vocabulary
    data::bow_vocabulary* bow_vocab_ = nullptr;
    //! BoW database
    data::bow_database* bow_db_ = nullptr;

    //! initializer
    module::initializer initializer_;

    //! frame tracker for current frame
    // const module::frame_tracker frame_tracker_;
    module::frame_tracker frame_tracker_;

    //! relocalizer
    module::relocalizer relocalizer_;

    //! pose optimizer
    // const optimize::pose_optimizer pose_optimizer_;
    optimize::pose_optimizer pose_optimizer_;

    //! keyframe inserter
    module::keyframe_inserter keyfrm_inserter_;

    //! reference keyframe
    data::keyframe* ref_keyfrm_ = nullptr;
    //! Proposed: last keyframe (used to associate odometry measurement from last keyframe. It can be different from ref_keyfrm_. )
    data::keyframe* last_keyfrm_ = nullptr;
    //! local keyframes
    std::vector<data::keyframe*> local_keyfrms_;
    //! local landmarks
    std::vector<data::landmark*> local_landmarks_;

    //! the number of tracked keyframes in the current keyframe
    unsigned int num_tracked_lms_ = 0;

    //! last frame
    data::frame last_frm_;

    //! latest frame ID which succeeded in relocalization
    unsigned int last_reloc_frm_id_ = 0;

    //! motion model
    Mat44_t velocity_;
    //! motion model is valid or not
    bool velocity_is_valid_ = false;

    //! current camera pose from reference keyframe
    //! (to update last camera pose at the beginning of each tracking)
    Mat44_t last_cam_pose_from_ref_keyfrm_;

    //-----------------------------------------
    // mapping module status

    //! mutex for mapping module status
    mutable std::mutex mtx_mapping_;

    //! mapping module is enabled or not
    bool mapping_is_enabled_ = true;

    //-----------------------------------------
    // management for pause process

    //! mutex for pause process
    mutable std::mutex mtx_pause_;

    //! Check the request frame and pause the tracking module
    bool check_and_execute_pause();

    //! the tracking module is paused or not
    bool is_paused_ = false;

    //! Pause of the tracking module is requested or not
    bool pause_is_requested_ = false;

    //-----------------------------------------
    // Proposed System variable

    Mat44_t reference_frame_pose_cw_;

    bool is_wheel_based_matching_;

    Mat44_t extrinsic_param_cr_ = Mat44_t::Identity();
    Mat44_t extrinsic_param_rc_ = Mat44_t::Identity();

    double odom_info_factor_xy_;
    double odom_info_factor_th_;
};

} // namespace openvslam

#endif // OPENVSLAM_TRACKING_MODULE_H
