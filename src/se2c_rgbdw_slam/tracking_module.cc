#include "se2c_rgbdw_slam/system.h"
#include "se2c_rgbdw_slam/tracking_module.h"
#include "se2c_rgbdw_slam/mapping_module.h"
#include "se2c_rgbdw_slam/global_optimization_module.h"


#include "se2c_rgbdw_slam/config.h"
#include "se2c_rgbdw_slam/camera/base.h"
#include "se2c_rgbdw_slam/data/landmark.h"
#include "se2c_rgbdw_slam/data/map_database.h"
#include "se2c_rgbdw_slam/data/bow_database.h"
#include "se2c_rgbdw_slam/feature/orb_extractor.h"
#include "se2c_rgbdw_slam/match/projection.h"
#include "se2c_rgbdw_slam/util/image_converter.h"
#include "se2c_rgbdw_slam/util/coordinate_transformer.h"
#include "se2c_rgbdw_slam/config_se2_constraint.h"


#include <chrono>
#include <unordered_map>

#include <spdlog/spdlog.h>

namespace openvslam {

tracking_module::tracking_module(const std::shared_ptr<config>& cfg, system* system, data::map_database* map_db,
                                 data::bow_vocabulary* bow_vocab, data::bow_database* bow_db)
    : cfg_(cfg), camera_(cfg->camera_), system_(system), map_db_(map_db), bow_vocab_(bow_vocab), bow_db_(bow_db),
      initializer_(cfg->camera_->setup_type_, map_db, bow_db, cfg->yaml_node_),
      frame_tracker_(camera_, 10), relocalizer_(bow_db_), pose_optimizer_(),
      keyfrm_inserter_(cfg_->camera_->setup_type_, cfg_->true_depth_thr_, map_db, bow_db, 0, cfg_->camera_->fps_)
{
    spdlog::debug("CONSTRUCT: tracking_module");

    extractor_left_ = new feature::orb_extractor(cfg_->orb_params_);
    if (camera_->setup_type_ == camera::setup_type_t::Monocular) {
        ini_extractor_left_ = new feature::orb_extractor(cfg_->orb_params_);
        ini_extractor_left_->set_max_num_keypoints(ini_extractor_left_->get_max_num_keypoints() * 2);
    }
    if (camera_->setup_type_ == camera::setup_type_t::Stereo) {
        extractor_right_ = new feature::orb_extractor(cfg_->orb_params_);
    }

    is_wheel_based_matching_ = cfg_->yaml_node_["Method.wheel_based_matching"].as<bool>(false);
    odom_info_factor_xy_ = cfg_->yaml_node_["OdometryConstraint.factor_xy"].as<double>(1000);
    odom_info_factor_th_ = cfg_->yaml_node_["OdometryConstraint.factor_th"].as<double>(1000);
}

tracking_module::~tracking_module() {
    delete extractor_left_;
    extractor_left_ = nullptr;
    delete extractor_right_;
    extractor_right_ = nullptr;
    delete ini_extractor_left_;
    ini_extractor_left_ = nullptr;


    spdlog::debug("DESTRUCT: tracking_module");
}

void tracking_module::set_mapping_module(mapping_module* mapper) {
    mapper_ = mapper;
    keyfrm_inserter_.set_mapping_module(mapper);
}

void tracking_module::set_global_optimization_module(global_optimization_module* global_optimizer) {
    global_optimizer_ = global_optimizer;
}

void tracking_module::set_mapping_module_status(const bool mapping_is_enabled) {
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    mapping_is_enabled_ = mapping_is_enabled;
}

bool tracking_module::get_mapping_module_status() const {
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    return mapping_is_enabled_;
}

std::vector<cv::KeyPoint> tracking_module::get_initial_keypoints() const {
    return initializer_.get_initial_keypoints();
}

std::vector<int> tracking_module::get_initial_matches() const {
    return initializer_.get_initial_matches();
}

Mat44_t tracking_module::visual_encoder_track(const cv::Mat& img, const cv::Mat& depthmap, const double timestamp, bool& is_keyframe, const Mat44_t& odom_from_ref_kf, const Mat66_t& cov_from_ref_kf, const cv::Mat& mask)
{
    assert(is_wheel_based_matching_);

    const auto start = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point t_create_frame, t_track_preprocess, t_track, t_update_localmap, t_check_keyframe;


    // color and depth scale conversion
    img_gray_ = img;
    cv::Mat img_depth = depthmap;
    util::convert_to_grayscale(img_gray_, camera_->color_order_);
    util::convert_to_true_depth(img_depth, cfg_->depthmap_factor_);

    is_keyframe = false;


    // 特徴を抽出する
    curr_frm_ = data::frame(img_gray_, img_depth, timestamp, extractor_left_, bow_vocab_, camera_, cfg_->true_depth_thr_, mask);
    
    t_create_frame = std::chrono::system_clock::now();

    // 初期化処理
    if (tracking_state_ == tracker_state_t::NotInitialized) tracking_state_ = tracker_state_t::Initializing;

    last_tracking_state_ = tracking_state_;

    // check if pause is requested
    check_and_execute_pause();
    while (is_paused()) {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    // LOCK the map database
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);


    // 初期化処理
    if (tracking_state_ == tracker_state_t::Initializing) 
    {
        const bool enable_plane_estimate = cfg_->yaml_node_["Initializer.enable_plane_estimate"].as<bool>(false);
        initializer_.set_depth_img(img_depth);

        if (!initialize()) 
        {
            Mat44_t mat_dummy;
            return mat_dummy;
        }

        if(enable_plane_estimate)
        {
                estimated_transform_irc_ = Eigen::PartialPivLU<openvslam::Mat44_t>(curr_frm_.cam_pose_cw_).inverse();
                extrinsic_param_cr_ = Eigen::PartialPivLU<openvslam::Mat44_t>(estimated_transform_irc_).inverse();
                extrinsic_param_rc_ = estimated_transform_irc_;
        }

        // update the reference keyframe, local keyframes, and local landmarks
        update_local_map();


        // pass all of the keyframes to the mapping module
        const auto keyfrms = map_db_->get_all_keyframes();
        for (const auto keyfrm : keyfrms) {
            mapper_->queue_keyframe(keyfrm);
        }

        // state transition to Tracking mode
        tracking_state_ = tracker_state_t::Tracking;

    }   //  end 初期化処理
    else 
    {
        //! Proposed
        Mat66_t odom_info = Eigen::PartialPivLU<openvslam::Mat44_t>(cov_from_ref_kf).inverse();
        odom_info(0,0) *= odom_info_factor_xy_;
        odom_info(1,1) *= odom_info_factor_xy_;
        odom_info(5,5) *= odom_info_factor_th_;
        std::pair<data::keyframe*, std::pair<Mat44_t, Mat66_t>> odom_from_ref = std::make_pair(last_keyfrm_, std::make_pair(odom_from_ref_kf, odom_info));
        curr_frm_.odom_from_ref_kf_ = std::make_shared<std::pair<data::keyframe*, std::pair<Mat44_t, Mat66_t>>>(odom_from_ref);

        set_reference_frame_pose(extrinsic_param_cr_ * Eigen::PartialPivLU<openvslam::Mat44_t>(odom_from_ref_kf).inverse() * extrinsic_param_rc_ * ref_keyfrm_->get_cam_pose());


        // apply replace of landmarks observed in the last frame
        apply_landmark_replace();
        // update the camera pose of the last frame
        // because the mapping module might optimize the camera pose of the last frame's reference keyframe
        update_last_frame();

        // set the reference keyframe of the current frame
        curr_frm_.ref_keyfrm_ = ref_keyfrm_;

        t_track_preprocess = std::chrono::system_clock::now();

        // マッチングを行なう
        auto succeeded = track_current_frame();
        if(!succeeded) spdlog::info("visual feature matching failed.");

        t_track = std::chrono::system_clock::now();
        
        // state transition
        // tracking_state_ = succeeded ? tracker_state_t::Tracking : tracker_state_t::Lost;

        // update the local map and optimize the camera pose of the current frame
        // if (succeeded) {
            update_local_map();
            succeeded = optimize_current_frame_with_local_map();
        // }


        // 位置はだいたいわかっているので、成功しようがしまいが現在姿勢を最適化する
        // if (succeeded) {
            update_motion_model();
        // }

        t_update_localmap = std::chrono::system_clock::now();

        // update the frame statistics
        map_db_->update_frame_statistics(curr_frm_, tracking_state_ == tracker_state_t::Lost);

        // if tracking is failed within 5.0 sec after initialization, reset the system
        constexpr float init_retry_thr = 5.0;
        if (tracking_state_ == tracker_state_t::Lost
            && curr_frm_.id_ - initializer_.get_initial_frame_id() < camera_->fps_ * init_retry_thr) {
            spdlog::info("tracking lost within {} sec after initialization", init_retry_thr);
            system_->request_reset();
            // return;
            return Mat44_t::Identity();
        }

        // show message if tracking has been lost
        if (last_tracking_state_ != tracker_state_t::Lost && tracking_state_ == tracker_state_t::Lost) {
            spdlog::info("tracking lost: frame {}", curr_frm_.id_);
        }

        // check to insert the new keyframe derived from the current frame
        if (succeeded && new_keyframe_is_needed()) {
        // if (new_keyframe_is_needed()) {
            insert_new_keyframe();
            is_keyframe = true;
        }

        //! Proposed
        // visual特徴マッチングによるトラッキングに失敗した場合は必ずKFにする
        if(!succeeded)
        {
            spdlog::info("insert keyframe due to tracking failure start.");
            // トラッキングに成功していないとき、LMの観測状態が異なるので、initialize時をベースにしたKF作成法を使う
            const auto ref_keyfrm = keyfrm_inserter_.insert_reference_keyfrm(curr_frm_);

            if(ref_keyfrm != nullptr)
            {
                spdlog::info("tracking_module::insert_reference_keyframe() num_valid_landmark {}, id: {}", ref_keyfrm->get_valid_landmarks().size(), ref_keyfrm->id_);
                ref_keyfrm_ = ref_keyfrm;
                last_keyfrm_ = ref_keyfrm;
                is_keyframe = true;
            }

            // set the reference keyframe with the new keyframe
            // ref_keyfrm_ = ref_keyfrm ? ref_keyfrm : ref_keyfrm_;
            curr_frm_.ref_keyfrm_ = ref_keyfrm_;

        }
        
        //! Proposed 
        if(is_keyframe) set_depthmap_to_keyfrm(img_depth);

        t_check_keyframe = std::chrono::system_clock::now();

        // tidy up observations
        for (unsigned int idx = 0; idx < curr_frm_.num_keypts_; ++idx) {
            if (curr_frm_.landmarks_.at(idx) && curr_frm_.outlier_flags_.at(idx)) {
                curr_frm_.landmarks_.at(idx) = nullptr;
            }
        }

        // Mat44_t odom_from_last_kf = curr_frm_.odom_from_ref_kf_->second.first;
        // Mat44_t curr_pose_from_last_kf =  extrinsic_param_rc_ * last_keyfrm_->get_cam_pose() *  curr_frm_.cam_pose_cw_.inverse() * extrinsic_param_cr_;
        // // std::cout << "[tracking_module() odom_from_last_kf]\n" << odom_from_last_kf << std::endl << std::endl;
        // // std::cout << "[tracking_module() curr_pose_from_last_kf]\n" << curr_pose_from_last_kf << std::endl;
        // std::cout << (curr_pose_from_last_kf.block(0,3, 3, 1) - odom_from_last_kf.block(0,3,3,1) ).transpose() << std::endl;

        // std::cout << std::endl << std::endl;
    }
   

    // store the relative pose from the reference keyframe to the current frame
    // to update the camera pose at the beginning of the next tracking process
    if (curr_frm_.cam_pose_cw_is_valid_) {
        last_cam_pose_from_ref_keyfrm_ = curr_frm_.cam_pose_cw_ * curr_frm_.ref_keyfrm_->get_cam_pose_inv();
    }

    // update last frame
    last_frm_ = curr_frm_;


    const auto end = std::chrono::system_clock::now();
    const auto time_create_frame = std::chrono::duration_cast<std::chrono::milliseconds>(t_create_frame - start).count();
    const auto time_track = std::chrono::duration_cast<std::chrono::milliseconds>(t_track - t_create_frame).count();
    const auto time_update_localmap = std::chrono::duration_cast<std::chrono::milliseconds>(t_update_localmap - t_track).count();
    const auto time_check_keyframe = std::chrono::duration_cast<std::chrono::milliseconds>(t_check_keyframe - t_update_localmap).count();
    elapsed_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    spdlog::debug("Tracking finished, num local landmark {}", local_landmarks_.size());
    spdlog::debug("tracking time create {}, track {}, localmap {}, checkKF {}, all {}", time_create_frame, time_track, time_update_localmap, time_check_keyframe, elapsed_ms_);

    return curr_frm_.cam_pose_cw_;
}

void tracking_module::reset() {
    spdlog::info("resetting system");

    initializer_.reset();
    keyfrm_inserter_.reset();

    mapper_->request_reset();
    global_optimizer_->request_reset();

    bow_db_->clear();
    map_db_->clear();

    data::frame::next_id_ = 0;
    data::keyframe::next_id_ = 0;
    data::landmark::next_id_ = 0;

    last_reloc_frm_id_ = 0;

    tracking_state_ = tracker_state_t::NotInitialized;
}

//! Proposed
void tracking_module::set_depthmap_to_keyfrm(const cv::Mat& depthmap)
{
    ref_keyfrm_->set_depthmap(depthmap);
}

void tracking_module::set_coord_transformer(std::shared_ptr<util::coordinate_transformer>& coord_transformer)
{
    // coord_transformer_ = coord_transformer;
    extrinsic_param_cr_ = coord_transformer->get_cr();
    extrinsic_param_rc_ = coord_transformer->get_rc();

    initializer_.set_coord_transformer(coord_transformer);
}

void tracking_module::set_se2c_wrappers(std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper>& plane_wrapper, std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper>& odom_wrapper)
{
    pose_optimizer_.set_se2c_wrappers(plane_wrapper, odom_wrapper);    
    frame_tracker_.set_se2c_wrappers(plane_wrapper, odom_wrapper);
}

bool tracking_module::initialize() {
    // try to initialize with the current frame
    initializer_.initialize(curr_frm_, extrinsic_param_cr_);

    // if map building was failed -> reset the map database
    if (initializer_.get_state() == module::initializer_state_t::Wrong) {
        // reset
        system_->request_reset();
        return false;
    }

    // if initializing was failed -> try to initialize with the next frame
    if (initializer_.get_state() != module::initializer_state_t::Succeeded) {
        return false;
    }

    last_keyfrm_ = curr_frm_.ref_keyfrm_;

    // spdlog::debug("tracking_module::initialize() end");

    // succeeded
    return true;
}

bool tracking_module::track_current_frame() {
    bool succeeded = false;


    if (tracking_state_ == tracker_state_t::Tracking) {
        if (velocity_is_valid_ && last_reloc_frm_id_ + 2 < curr_frm_.id_) {
            // if the motion model is valid and more than 2 frames are created since last relocalize
            succeeded = frame_tracker_.motion_based_track(curr_frm_, last_frm_, velocity_);
        }
        if (!succeeded) {
            succeeded = frame_tracker_.bow_match_based_track(curr_frm_, last_frm_, ref_keyfrm_);
        }
        if (!succeeded) {
            succeeded = frame_tracker_.robust_match_based_track(curr_frm_, last_frm_, ref_keyfrm_);
        }
        if (!succeeded)
        {
            curr_frm_.set_cam_pose(reference_frame_pose_cw_);
        }
    }
    else {
        // Lost mode
        // try to relocalize
        succeeded = relocalizer_.relocalize(curr_frm_);

        if (succeeded) {
            last_reloc_frm_id_ = curr_frm_.id_;
        }
    }
    
    return succeeded;
}

void tracking_module::update_motion_model() {
    if (last_frm_.cam_pose_cw_is_valid_) {
        Mat44_t last_frm_cam_pose_wc = Mat44_t::Identity();
        last_frm_cam_pose_wc.block<3, 3>(0, 0) = last_frm_.get_rotation_inv();
        last_frm_cam_pose_wc.block<3, 1>(0, 3) = last_frm_.get_cam_center();
        velocity_is_valid_ = true;
        velocity_ = curr_frm_.cam_pose_cw_ * last_frm_cam_pose_wc;

        // spdlog::debug("velocity print");
        // std::cout << "velocity\n";
        // std::cout << velocity_ << std::endl;     // 4x4 mat
    }
    else {
        velocity_is_valid_ = false;
        velocity_ = Mat44_t::Identity();
    }

}

void tracking_module::apply_landmark_replace() {
    for (unsigned int idx = 0; idx < last_frm_.num_keypts_; ++idx) {
        auto lm = last_frm_.landmarks_.at(idx);
        if (!lm) {
            continue;
        }

        auto replaced_lm = lm->get_replaced();
        if (replaced_lm) {
            last_frm_.landmarks_.at(idx) = replaced_lm;
        }
    }
}

void tracking_module::update_last_frame() {
    auto last_ref_keyfrm = last_frm_.ref_keyfrm_;
    if (!last_ref_keyfrm) {
        return;
    }
    last_frm_.set_cam_pose(last_cam_pose_from_ref_keyfrm_ * last_ref_keyfrm->get_cam_pose());
}

bool tracking_module::optimize_current_frame_with_local_map() {
    // acquire more 2D-3D matches by reprojecting the local landmarks to the current frame
    search_local_landmarks();

    // optimize the pose
    pose_optimizer_.optimize(curr_frm_);


    // count up the number of tracked landmarks
    num_tracked_lms_ = 0;
    for (unsigned int idx = 0; idx < curr_frm_.num_keypts_; ++idx) {
        auto lm = curr_frm_.landmarks_.at(idx);
        if (!lm) {
            continue;
        }

        if (!curr_frm_.outlier_flags_.at(idx)) {
            // the observation has been considered as inlier in the pose optimization
            assert(lm->has_observation());
            // count up
            ++num_tracked_lms_;
            // increment the number of tracked frame
            lm->increase_num_observed();
        }
        else {
            // the observation has been considered as outlier in the pose optimization
            // remove the observation
            curr_frm_.landmarks_.at(idx) = nullptr;
        }
    }

    constexpr unsigned int num_tracked_lms_thr = 20;

    // if recently relocalized, use the more strict threshold
    if (curr_frm_.id_ < last_reloc_frm_id_ + camera_->fps_ && num_tracked_lms_ < 2 * num_tracked_lms_thr) {
        spdlog::debug("local map tracking failed: {} matches < {}", num_tracked_lms_, 2 * num_tracked_lms_thr);
        return false;
    }

    // check the threshold of the number of tracked landmarks
    if (num_tracked_lms_ < num_tracked_lms_thr) {
        spdlog::debug("local map tracking failed: {} matches < {}", num_tracked_lms_, num_tracked_lms_thr);
        return false;
    }

    spdlog::debug("local map tracking succeed: {} matches", num_tracked_lms_);
    return true;
}

void tracking_module::update_local_map() {
    update_local_keyframes();
    update_local_landmarks();

    map_db_->set_local_landmarks(local_landmarks_);

    // spdlog::debug("tracing_module::update_local_map() end");
}

void tracking_module::update_local_keyframes() {
    // spdlog::debug("tracing_module::update_local_keyframes() start");

    constexpr unsigned int max_num_local_keyfrms = 60;

    // count the number of sharing landmarks between the current frame and each of the neighbor keyframes
    // key: keyframe, value: number of sharing landmarks
    std::unordered_map<data::keyframe*, unsigned int> keyfrm_weights;
    for (unsigned int idx = 0; idx < curr_frm_.num_keypts_; ++idx) {
        auto lm = curr_frm_.landmarks_.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            curr_frm_.landmarks_.at(idx) = nullptr;
            continue;
        }

        const auto observations = lm->get_observations();
        for (auto obs : observations) {
            ++keyfrm_weights[obs.first];
        }
    }

    if (keyfrm_weights.empty()) {
        return;
    }

    // set the aforementioned keyframes as local keyframes
    // and find the nearest keyframe
    unsigned int max_weight = 0;
    data::keyframe* nearest_covisibility = nullptr;

    local_keyfrms_.clear();
    // local_keyfrms_.reserve(4 * keyfrm_weights.size());
    local_keyfrms_.reserve(max_num_local_keyfrms);

    for (auto& keyfrm_weight : keyfrm_weights) {
        auto keyfrm = keyfrm_weight.first;
        const auto weight = keyfrm_weight.second;

        if (keyfrm->will_be_erased()) {
            continue;
        }

        local_keyfrms_.push_back(keyfrm);

        // avoid duplication
        keyfrm->local_map_update_identifier = curr_frm_.id_;

        // update the nearest keyframe
        if (max_weight < weight) {
            max_weight = weight;
            nearest_covisibility = keyfrm;
        }
    }

    // add the second-order keyframes to the local landmarks
    auto add_local_keyframe = [this](data::keyframe* keyfrm) {
        if (!keyfrm) {
            return false;
        }
        if (keyfrm->will_be_erased()) {
            return false;
        }
        // avoid duplication
        if (keyfrm->local_map_update_identifier == curr_frm_.id_) {
            return false;
        }
        keyfrm->local_map_update_identifier = curr_frm_.id_;
        local_keyfrms_.push_back(keyfrm);
        return true;
    };
    for (auto iter = local_keyfrms_.cbegin(); iter != local_keyfrms_.cend(); ++iter) {
        if (max_num_local_keyfrms < local_keyfrms_.size()) {
            break;
        }

        auto keyfrm = *iter;

        // covisibilities of the neighbor keyframe
        const auto neighbors = keyfrm->graph_node_->get_top_n_covisibilities(10);
        for (auto neighbor : neighbors) {
            if (add_local_keyframe(neighbor)) {
                break;
            }
        }

        // children of the spanning tree
        const auto spanning_children = keyfrm->graph_node_->get_spanning_children();
        for (auto child : spanning_children) {
            if (add_local_keyframe(child)) {
                break;
            }
        }

        // parent of the spanning tree
        auto parent = keyfrm->graph_node_->get_spanning_parent();
        add_local_keyframe(parent);
    }

    // update the reference keyframe with the nearest one
    if (nearest_covisibility) {
        ref_keyfrm_ = nearest_covisibility;
        curr_frm_.ref_keyfrm_ = ref_keyfrm_;
    }

    // spdlog::debug("tracing_module::update_local_keyframes() end");
}

void tracking_module::update_local_landmarks() {
    // spdlog::debug("tracing_module::update_local_landmarks() start");

    local_landmarks_.clear();

    for (auto keyfrm : local_keyfrms_) {
        const auto lms = keyfrm->get_landmarks();

        for (auto lm : lms) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            // avoid duplication
            if (lm->identifier_in_local_map_update_ == curr_frm_.id_) {
                continue;
            }
            lm->identifier_in_local_map_update_ = curr_frm_.id_;

            local_landmarks_.push_back(lm);
        }
    }
    // spdlog::debug("tracing_module::update_local_landmarks() end");
}

void tracking_module::search_local_landmarks() {
    // select the landmarks which can be reprojected from the ones observed in the current frame
    for (auto lm : curr_frm_.landmarks_) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // this landmark cannot be reprojected
        // because already observed in the current frame
        lm->is_observable_in_tracking_ = false;
        lm->identifier_in_local_lm_search_ = curr_frm_.id_;

        // this landmark is observable from the current frame
        lm->increase_num_observable();
    }

    bool found_proj_candidate = false;
    // temporary variables
    Vec2_t reproj;
    float x_right;
    unsigned int pred_scale_level;
    for (auto lm : local_landmarks_) {
        // avoid the landmarks which cannot be reprojected (== observed in the current frame)
        if (lm->identifier_in_local_lm_search_ == curr_frm_.id_) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // check the observability
        if (curr_frm_.can_observe(lm, 0.5, reproj, x_right, pred_scale_level)) {
            // pass the temporary variables
            lm->reproj_in_tracking_ = reproj;
            lm->x_right_in_tracking_ = x_right;
            lm->scale_level_in_tracking_ = pred_scale_level;

            // this landmark can be reprojected
            lm->is_observable_in_tracking_ = true;

            // this landmark is observable from the current frame
            lm->increase_num_observable();

            found_proj_candidate = true;
        }
        else {
            // this landmark cannot be reprojected
            lm->is_observable_in_tracking_ = false;
        }
    }

    if (!found_proj_candidate) {
        return;
    }

    // acquire more 2D-3D matches by projecting the local landmarks to the current frame
    match::projection projection_matcher(0.8);
    const float margin = (curr_frm_.id_ < last_reloc_frm_id_ + 2)
                             ? 20.0
                             : ((camera_->setup_type_ == camera::setup_type_t::RGBD)
                                    ? 10.0
                                    : 5.0);
    projection_matcher.match_frame_and_landmarks(curr_frm_, local_landmarks_, margin);
}

bool tracking_module::new_keyframe_is_needed() const {
    if (!mapping_is_enabled_) {
        return false;
    }

    // cannnot insert the new keyframe in a second after relocalization
    const auto num_keyfrms = map_db_->get_num_keyframes();
    if (cfg_->camera_->fps_ < num_keyfrms && curr_frm_.id_ < last_reloc_frm_id_ + cfg_->camera_->fps_) {
        return false;
    }

    // check the new keyframe is needed
    return keyfrm_inserter_.new_keyframe_is_needed(curr_frm_, num_tracked_lms_, *ref_keyfrm_/*, tracking_state_*/);
}

void tracking_module::insert_new_keyframe() {
    //! Proposed
    // insert the new keyframe
    const auto ref_keyfrm = keyfrm_inserter_.insert_new_keyframe(curr_frm_/*, tracking_state_*/);

    // set the reference keyframe with the new keyframe
    ref_keyfrm_ = ref_keyfrm ? ref_keyfrm : ref_keyfrm_;
    curr_frm_.ref_keyfrm_ = ref_keyfrm_;
    last_keyfrm_ = ref_keyfrm;

    spdlog::info("tracking_module::insert_new_keyframe() num_valid_landmark {}, id: {}", ref_keyfrm->get_valid_landmarks().size(), ref_keyfrm->id_);

}

void tracking_module::request_pause() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    pause_is_requested_ = true;
}

bool tracking_module::pause_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return pause_is_requested_;
}

bool tracking_module::is_paused() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return is_paused_;
}

void tracking_module::resume() {
    std::lock_guard<std::mutex> lock(mtx_pause_);

    is_paused_ = false;
    pause_is_requested_ = false;

    spdlog::info("resume tracking module");
}

bool tracking_module::check_and_execute_pause() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    if (pause_is_requested_) {
        is_paused_ = true;
        spdlog::info("pause tracking module");
        return true;
    }
    else {
        return false;
    }
}

void tracking_module::set_reference_frame_pose(const Mat44_t &pose_mat_cw)
{
    reference_frame_pose_cw_ = pose_mat_cw;
}

Mat44_t tracking_module::get_reference_frame_pose() const
{
    return reference_frame_pose_cw_;
}

} // namespace openvslam
