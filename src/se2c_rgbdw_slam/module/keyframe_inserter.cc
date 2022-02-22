#include "se2c_rgbdw_slam/mapping_module.h"
#include "se2c_rgbdw_slam/data/landmark.h"
#include "se2c_rgbdw_slam/data/bow_database.h"
#include "se2c_rgbdw_slam/data/map_database.h"
#include "se2c_rgbdw_slam/module/keyframe_inserter.h"
#include "se2c_rgbdw_slam/util/converter.h"

#include <spdlog/spdlog.h>

namespace openvslam {
namespace module {

keyframe_inserter::keyframe_inserter(const camera::setup_type_t setup_type, const float true_depth_thr,
                                     data::map_database* map_db, data::bow_database* bow_db,
                                     const unsigned int min_num_frms, const unsigned int max_num_frms)
    : setup_type_(setup_type), true_depth_thr_(true_depth_thr),
      map_db_(map_db), bow_db_(bow_db),
      min_num_frms_(min_num_frms), max_num_frms_(max_num_frms) {}

void keyframe_inserter::set_mapping_module(mapping_module* mapper) {
    mapper_ = mapper;
}

void keyframe_inserter::reset() {
    frm_id_of_last_keyfrm_ = 0;
}

bool keyframe_inserter::new_keyframe_is_needed(const data::frame& curr_frm, const unsigned int num_tracked_lms,
                                               const data::keyframe& ref_keyfrm/*, const tracker_state_t tracker_state*/) const {
    assert(mapper_);
    // mapping moduleが停止しているときはキーフレームが追加できない
    if (mapper_->is_paused() || mapper_->pause_is_requested()) {
        return false;
    }

    const auto num_keyfrms = map_db_->get_num_keyframes();

    // reference keyframeで観測している3次元点のうち，3視点以上から観測されている3次元点の数を数える
    const unsigned int min_obs_thr = (3 <= num_keyfrms) ? 3 : 2;
    const auto num_reliable_lms = ref_keyfrm.get_num_tracked_landmarks(min_obs_thr);

    // mappingが処理中かどうか
    const bool mapper_is_idle = mapper_->get_keyframe_acceptability();

    // 最新のキーフレームで観測している3次元点数に対する，現在のフレームで観測している3次元点数の割合の閾値
    constexpr unsigned int num_tracked_lms_thr = 15;
    const float lms_ratio_thr = 0.9;

    // // 条件A1: 前回のキーフレーム挿入からmax_num_frames_以上経過していたらキーフレームを追加する
    const bool cond_a1 = frm_id_of_last_keyfrm_ + max_num_frms_ <= curr_frm.id_;
    // // 条件A2: min_num_frames_以上経過していて,mapping moduleが待機状態であればキーフレームを追加する
    // const bool cond_a2 = (frm_id_of_last_keyfrm_ + min_num_frms_ <= curr_frm.id_) && mapper_is_idle;
    // // 条件A3: 前回のキーフレームから視点が移動してたらキーフレームを追加する
    // const bool cond_a3 = num_tracked_lms < num_reliable_lms * 0.25;

    // 条件B: (キーフレーム追加の必要条件)3次元点が閾値以上観測されていて，3次元点との割合が一定割合以下であればキーフレームを追加する
    // const bool cond_b = (num_tracked_lms_thr <= num_tracked_lms) && (num_tracked_lms < num_reliable_lms * lms_ratio_thr);

    // 条件B1: (キーフレーム追加の必要条件)
    //   1. 3次元点が閾値以上観測されている
    //   2. 参照KFにおいて複数地点から観測されている3次元点と、トラッキング中の三次元点との割合が一定割合以下
    // const bool cond_b1 = (num_tracked_lms_thr <= num_tracked_lms) && (num_tracked_lms < num_reliable_lms * lms_ratio_thr);

    // 条件B2: 参照KF上の複数視点から見られているLMが一定数以下である
    const int num_reliable_lms_thr = 50;
    const bool cond_b2 = (num_reliable_lms < num_reliable_lms_thr) || (num_tracked_lms <= num_tracked_lms_thr);

    // 条件B3: オドメトリの観測値が一定以上離れている
    constexpr double distance_sq_thr = 0.01;                 // 0.1cm^2
    constexpr double angle_thr = 10.0 * M_PI / 180.0;
    const Mat44_t odom_from_ref_kf = curr_frm.odom_from_ref_kf_->second.first;
    double x, y, th;
    util::converter::SE3toSE2param(odom_from_ref_kf, x, y, th);
    const double distance_sq = x*x + y*y;
    const bool cond_b3 = (distance_sq_thr < distance_sq) || (angle_thr < abs(th));

    // Bが満たされていなければ追加しない
    // if (!cond_b) {
    //     return false;
    // }

    // B1とB2の両方が満たされていなければ追加しない(KF追加のためには少なくとも片方は満たしている必要がある)
    // if (!cond_a3 && !cond_b2 && !cond_b3) {
    //     return false;
    // }

    if (!cond_a1 && !cond_b2 && !cond_b3) return false;

    // Aのいずれも満たされていなければ追加しない
    // if (!cond_a1 && !cond_a2 && !cond_a3) {
    //     return false;
    // }

    if (mapper_is_idle) {
        // mapping moduleが処理中でなければ，とりあえずkeyframeを追加しておく
        return true;
    }

    // mapping moduleが処理中だったら，local BAを止めてキーフレームを追加する
    if (setup_type_ != camera::setup_type_t::Monocular
        && mapper_->get_num_queued_keyframes() <= 2) {
        mapper_->abort_local_BA();
        return true;
    }

    return false;
}

data::keyframe* keyframe_inserter::insert_new_keyframe(data::frame& curr_frm/*, const tracker_state_t tracking_state*/) {
    // mapping moduleを(強制的に)動かす
    if (!mapper_->set_force_to_run(true)) {
        return nullptr;
    }

    curr_frm.update_pose_params();
    auto keyfrm = new data::keyframe(curr_frm, map_db_, bow_db_);

    //! Proposed
    keyfrm->odom_from_ref_kf_ = curr_frm.odom_from_ref_kf_;

    std::pair<data::keyframe*, std::pair<Mat44_t, Mat66_t>> odom_to_ref = std::make_pair(keyfrm, std::make_pair(curr_frm.odom_from_ref_kf_->second.first, curr_frm.odom_from_ref_kf_->second.second));
    auto previous_keyfrm = curr_frm.ref_keyfrm_;
    previous_keyfrm->odom_to_ref_kf_ = std::make_shared<std::pair<data::keyframe*, std::pair<Mat44_t, Mat66_t>>>(odom_to_ref);

    frm_id_of_last_keyfrm_ = curr_frm.id_;

    // monocularだったらkeyframeをmapping moduleにqueueして終わり
    if (setup_type_ == camera::setup_type_t::Monocular) {
        queue_keyframe(keyfrm);
        return keyfrm;
    }

    // 有効なdepthとそのindexを格納する
    std::vector<std::pair<float, unsigned int>> depth_idx_pairs;
    depth_idx_pairs.reserve(curr_frm.num_keypts_);
    for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
        const auto depth = curr_frm.depths_.at(idx);
        // depthが有効な範囲のものを追加する
        if (0 < depth) {
            depth_idx_pairs.emplace_back(std::make_pair(depth, idx));
        }
    }

    // 有効なdepthがなかったらqueueして終わり
    if (depth_idx_pairs.empty()) {
        queue_keyframe(keyfrm);
        return keyfrm;
    }

    // カメラに近い順に並べ直す
    std::sort(depth_idx_pairs.begin(), depth_idx_pairs.end());

    // depthを使って3次元点を最小min_num_to_create点作る
    constexpr unsigned int min_num_to_create = 100;
    for (unsigned int count = 0; count < depth_idx_pairs.size(); ++count) {
        const auto depth = depth_idx_pairs.at(count).first;
        const auto idx = depth_idx_pairs.at(count).second;

        // 最小閾値以上の点が追加されて，かつdepthの範囲が敷居を超えたら追加をやめる
        if (min_num_to_create < count && true_depth_thr_ < depth) {
            break;
        }

        // idxに対応する3次元点がある場合はstereo triangulationしない
        {
            auto lm = curr_frm.landmarks_.at(idx);
            if (lm) {
                assert(lm->has_observation());
                continue;
            }
        }

        // idxに対応する3次元がなければstereo triangulationで作る
        const Vec3_t pos_w = curr_frm.triangulate_stereo(idx);
        auto lm = new data::landmark(pos_w, keyfrm, map_db_);

        lm->add_observation(keyfrm, idx);
        keyfrm->add_landmark(lm, idx);
        curr_frm.landmarks_.at(idx) = lm;

        lm->compute_descriptor();
        lm->update_normal_and_depth();

        map_db_->add_landmark(lm);
    }

    // std::cout << "keyframe_inserter::insert_new_keyframe() keyframe id: " << keyfrm->id_ << " : num landmarks of this KF: " << keyfrm->get_valid_landmarks().size() << std::endl;

    // keyframeをqueueして終わり
    queue_keyframe(keyfrm);
    return keyfrm;
}

// !! Proposed
data::keyframe* keyframe_inserter::insert_reference_keyfrm(data::frame& curr_frm)
{
    //! from keyframe_inserter::insert_new_keyfrm()
    // mapping moduleを(強制的に)動かす
    if (!mapper_->set_force_to_run(true)) {
        std::cout << "[keyframe_inserter::insert_reference_keyfrm] nullptr\n" << "\n";
        return nullptr;
        // return  0;
    }

    curr_frm.update_pose_params();
    auto curr_keyfrm = new data::keyframe(curr_frm, map_db_, bow_db_);

    //!Proposed
    curr_keyfrm->odom_from_ref_kf_ = curr_frm.odom_from_ref_kf_;

    std::pair<data::keyframe*, std::pair<Mat44_t, Mat66_t>> odom_to_ref = std::make_pair(curr_keyfrm, std::make_pair(curr_frm.odom_from_ref_kf_->second.first, curr_frm.odom_from_ref_kf_->second.second));
    auto previous_keyfrm = curr_frm.ref_keyfrm_;
    previous_keyfrm->odom_to_ref_kf_ = std::make_shared<std::pair<data::keyframe*, std::pair<Mat44_t, Mat66_t>>>(odom_to_ref);

    frm_id_of_last_keyfrm_ = curr_frm.id_;
    
    // monocularだったらkeyframeをmapping moduleにqueueして終わり
    if (setup_type_ == camera::setup_type_t::Monocular) {
        queue_keyframe(curr_keyfrm);
        return curr_keyfrm;
    }

    // 有効なdepthとそのindexを格納する
    std::vector<std::pair<float, unsigned int>> depth_idx_pairs;
    depth_idx_pairs.reserve(curr_frm.num_keypts_);
    for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
        // add a new landmark if tht corresponding depth is valid
        const auto depth = curr_frm.depths_.at(idx);
        if (depth <= 0) {
            continue;
        }
        
        // idxに対応する3次元点がある場合はstereo triangulationしない
        {
            auto lm = curr_frm.landmarks_.at(idx);
            if (lm) {
                // assert(lm->has_observation());       // 無理やりつくるKFなので観測されているとは限らない
                continue;
            }
        }

        // build a landmark
        const Vec3_t pos_w = curr_frm.triangulate_stereo(idx);
        auto lm = new data::landmark(pos_w, curr_keyfrm, map_db_);

        //!! Proposed
        lm->set_referenced_flag(true);

        // set the associations to the new keyframe
        lm->add_observation(curr_keyfrm, idx);
        curr_keyfrm->add_landmark(lm, idx);
        curr_frm.landmarks_.at(idx) = lm;

        // update the descriptor
        lm->compute_descriptor();
        lm->update_normal_and_depth();

        // set the 2D-3D associations to the current frame
        // curr_frm.outlier_flags_.at(idx) = false;

        // add the landmark to the map DB
        map_db_->add_landmark(lm);

    }

    curr_keyfrm->set_referenced_flag(true);

    queue_keyframe(curr_keyfrm);
    return curr_keyfrm;
}

void keyframe_inserter::queue_keyframe(data::keyframe* keyfrm) {
    mapper_->queue_keyframe(keyfrm);
    mapper_->set_force_to_run(false);
}

} // namespace module
} // namespace openvslam
