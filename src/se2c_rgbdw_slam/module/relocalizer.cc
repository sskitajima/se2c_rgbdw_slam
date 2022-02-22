#include "se2c_rgbdw_slam/data/frame.h"
#include "se2c_rgbdw_slam/data/keyframe.h"
#include "se2c_rgbdw_slam/data/landmark.h"
#include "se2c_rgbdw_slam/data/bow_database.h"
#include "se2c_rgbdw_slam/module/relocalizer.h"
#include "se2c_rgbdw_slam/util/fancy_index.h"

#include <spdlog/spdlog.h>

namespace openvslam {
namespace module {

relocalizer::relocalizer(data::bow_database* bow_db,
                         const double bow_match_lowe_ratio, const double proj_match_lowe_ratio,
                         const unsigned int min_num_bow_matches, const unsigned int min_num_valid_obs)
    : bow_db_(bow_db),
      min_num_bow_matches_(min_num_bow_matches), min_num_valid_obs_(min_num_valid_obs),
      bow_matcher_(bow_match_lowe_ratio, true), proj_matcher_(proj_match_lowe_ratio, true),
      pose_optimizer_() {
    spdlog::debug("CONSTRUCT: module::relocalizer");
}

relocalizer::~relocalizer() {
    spdlog::debug("DESTRUCT: module::relocalizer");
}

bool relocalizer::relocalize(data::frame& curr_frm) {
    curr_frm.compute_bow();

    // acquire relocalization candidates
    const auto reloc_candidates = bow_db_->acquire_relocalization_candidates(&curr_frm);
    if (reloc_candidates.empty()) {
        return false;
    }
    const auto num_candidates = reloc_candidates.size();

    std::vector<std::vector<data::landmark*>> matched_landmarks(num_candidates);

    // 各候補について，BoW tree matcherで対応点を求める
    for (unsigned int i = 0; i < num_candidates; ++i) {
        auto keyfrm = reloc_candidates.at(i);
        if (keyfrm->will_be_erased()) {
            continue;
        }

        //!! Proposed
        // referencedのKFは不安定なので、間違う可能性が高い
        if(keyfrm->is_referenced_keyfrm()){
            continue;
        }

        const auto num_matches = bow_matcher_.match_frame_and_keyframe(keyfrm, curr_frm, matched_landmarks.at(i));
        // discard the candidate if the number of 2D-3D matches is less than the threshold
        if (num_matches < min_num_bow_matches_) {
            continue;
        }

        // setup PnP solver with the current 2D-3D matches
        const auto valid_indices = extract_valid_indices(matched_landmarks.at(i));
        auto pnp_solver = setup_pnp_solver(valid_indices, curr_frm.bearings_, curr_frm.keypts_,
                                           matched_landmarks.at(i), curr_frm.scale_factors_);

        // 1. Estimate the camera pose with EPnP(+RANSAC)

        pnp_solver->find_via_ransac(30);
        if (!pnp_solver->solution_is_valid()) {
            continue;
        }

        curr_frm.cam_pose_cw_ = pnp_solver->get_best_cam_pose();
        curr_frm.update_pose_params();

        // 2. Apply pose optimizer

        // get the inlier indices after EPnP+RANSAC
        const auto inlier_indices = util::resample_by_indices(valid_indices, pnp_solver->get_inlier_flags());

        // set 2D-3D matches for the pose optimization
        curr_frm.landmarks_ = std::vector<data::landmark*>(curr_frm.num_keypts_, nullptr);
        std::set<data::landmark*> already_found_landmarks;
        for (const auto idx : inlier_indices) {
            // 有効な3次元点のみをcurrent frameにセット
            curr_frm.landmarks_.at(idx) = matched_landmarks.at(i).at(idx);
            // すでに特徴点と対応した3次元点を記録しておく
            already_found_landmarks.insert(matched_landmarks.at(i).at(idx));
        }

        // pose optimization
        auto num_valid_obs = pose_optimizer_.optimize(curr_frm);
        // discard the candidate if the number of the inliers is less than the threshold
        if (num_valid_obs < min_num_bow_matches_ / 2) {
            continue;
        }

        // reject outliers
        for (unsigned int idx = 0; idx < curr_frm.num_keypts_; idx++) {
            if (!curr_frm.outlier_flags_.at(idx)) {
                continue;
            }
            curr_frm.landmarks_.at(idx) = nullptr;
        }

        // 3. Apply projection match to increase 2D-3D matches

        // projection match based on the pre-optimized camera pose
        auto num_found = proj_matcher_.match_frame_and_keyframe(curr_frm, reloc_candidates.at(i), already_found_landmarks, 10, 100);
        // discard the candidate if the number of the inliers is less than the threshold
        if (num_valid_obs + num_found < min_num_valid_obs_) {
            continue;
        }

        // 4. Re-apply the pose optimizer

        num_valid_obs = pose_optimizer_.optimize(curr_frm);

        // 閾値未満になったら，もう一度projection matchを行う
        if (num_valid_obs < min_num_valid_obs_) {
            // すでに対応がついているものは除く
            already_found_landmarks.clear();
            for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
                if (!curr_frm.landmarks_.at(idx)) {
                    continue;
                }
                already_found_landmarks.insert(curr_frm.landmarks_.at(idx));
            }
            // もう一度projection matchを行う -> 2D-3D対応を設定
            auto num_additional = proj_matcher_.match_frame_and_keyframe(curr_frm, reloc_candidates.at(i), already_found_landmarks, 3, 64);

            // 閾値未満だったら破棄
            if (num_valid_obs + num_additional < min_num_valid_obs_) {
                continue;
            }

            // もう一度最適化
            num_valid_obs = pose_optimizer_.optimize(curr_frm);

            // 閾値未満だったら破棄
            if (num_valid_obs < min_num_valid_obs_) {
                continue;
            }
        }

        // relocalize成功
        spdlog::info("relocalization succeeded");
        // TODO: current frameのreference keyframeをセットする

        // reject outliers
        for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
            if (!curr_frm.outlier_flags_.at(idx)) {
                continue;
            }
            curr_frm.landmarks_.at(idx) = nullptr;
        }

        return true;
    }

    curr_frm.cam_pose_cw_is_valid_ = false;
    return false;
}

std::vector<unsigned int> relocalizer::extract_valid_indices(const std::vector<data::landmark*>& landmarks) const {
    std::vector<unsigned int> valid_indices;
    valid_indices.reserve(landmarks.size());
    for (unsigned int idx = 0; idx < landmarks.size(); ++idx) {
        auto lm = landmarks.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        valid_indices.push_back(idx);
    }
    return valid_indices;
}

std::unique_ptr<solve::pnp_solver> relocalizer::setup_pnp_solver(const std::vector<unsigned int>& valid_indices,
                                                                 const eigen_alloc_vector<Vec3_t>& bearings,
                                                                 const std::vector<cv::KeyPoint>& keypts,
                                                                 const std::vector<data::landmark*>& matched_landmarks,
                                                                 const std::vector<float>& scale_factors) const {
    // resample valid elements
    const auto valid_bearings = util::resample_by_indices(bearings, valid_indices);
    const auto valid_keypts = util::resample_by_indices(keypts, valid_indices);
    const auto valid_assoc_lms = util::resample_by_indices(matched_landmarks, valid_indices);
    eigen_alloc_vector<Vec3_t> valid_landmarks(valid_indices.size());
    for (unsigned int i = 0; i < valid_indices.size(); ++i) {
        valid_landmarks.at(i) = valid_assoc_lms.at(i)->get_pos_in_world();
    }
    // setup PnP solver
    return std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(valid_bearings, valid_keypts, valid_landmarks, scale_factors));
}


bool relocalizer::relocalize_with_ref_pose(data::frame& curr_frm, const Mat44_t &ref_cam_pose_cw, const double THRES_RELOCALIZE_TRANS, const double THRES_RELOCALIZE_ROT)
{
    curr_frm.compute_bow();

    // acquire relocalization candidates
    const auto reloc_candidates = bow_db_->acquire_relocalization_candidates(&curr_frm);
    if (reloc_candidates.empty()) {
        return false;
    }
    const auto num_candidates = reloc_candidates.size();

    std::vector<std::vector<data::landmark*>> matched_landmarks(num_candidates);

    // 各候補について，BoW tree matcherで対応点を求める
    for (unsigned int i = 0; i < num_candidates; ++i) {
        auto keyfrm = reloc_candidates.at(i);
        if (keyfrm->will_be_erased()) {
            continue;
        }


        // different from relocalize()
        //////////////////////////////////////////////
        auto pose_keyfrm = keyfrm->get_cam_pose().inverse();
        const Mat33_t rot_keyfrm_wc = pose_keyfrm.block<3, 3>(0, 0);
        const Vec3_t trans_keyfrm_wc = pose_keyfrm.block<3, 1>(0, 3);

        const Mat33_t rot_ref_wc = ref_cam_pose_cw.block<3, 3>(0, 0);
        const Vec3_t trans_ref_wc = ref_cam_pose_cw.block<3, 1>(0, 3);

        const Vec3_t key_rpy = util::converter::to_angle_axis(rot_keyfrm_wc);
        const Vec3_t ref_rpy = util::converter::to_angle_axis(rot_ref_wc);
        Vec3_t diff_angle = ref_rpy - key_rpy;
        Vec3_t diff_trans = trans_ref_wc - trans_keyfrm_wc;

        spdlog::debug("rpy ref {} {} {}", ref_rpy(0), ref_rpy(1), ref_rpy(2) );
        spdlog::debug("rpy key {} {} {}", key_rpy(0), key_rpy(1), key_rpy(2) );
        spdlog::debug("trans ref {} {} {}", trans_ref_wc(0), trans_ref_wc(1), trans_ref_wc(2) );
        spdlog::debug("trans key {} {} {}", trans_keyfrm_wc(0), trans_keyfrm_wc(1), trans_keyfrm_wc(2) );

        // const double THRES_RELOCALIZE_TRANS = 2.0;
        // const double THRES_RELOCALIZE_ROT = M_PI_2;
        if(diff_trans.norm() > THRES_RELOCALIZE_TRANS)
        {
            spdlog::debug("too far distance for relocalize");
            continue;
        }
        if(diff_angle.norm() > THRES_RELOCALIZE_ROT)
        {
            spdlog::debug("too far angle for relocalize");
            continue;
        }
        //////////////////////////////////////////////
        // different from relocalize() end


        const auto num_matches = bow_matcher_.match_frame_and_keyframe(keyfrm, curr_frm, matched_landmarks.at(i));
        // discard the candidate if the number of 2D-3D matches is less than the threshold
        if (num_matches < min_num_bow_matches_) {
            continue;
        }

        // setup PnP solver with the current 2D-3D matches
        const auto valid_indices = extract_valid_indices(matched_landmarks.at(i));
        auto pnp_solver = setup_pnp_solver(valid_indices, curr_frm.bearings_, curr_frm.keypts_,
                                           matched_landmarks.at(i), curr_frm.scale_factors_);

        // 1. Estimate the camera pose with EPnP(+RANSAC)

        pnp_solver->find_via_ransac(30);
        if (!pnp_solver->solution_is_valid()) {
            continue;
        }

        curr_frm.cam_pose_cw_ = pnp_solver->get_best_cam_pose();
        curr_frm.update_pose_params();

        // 2. Apply pose optimizer

        // get the inlier indices after EPnP+RANSAC
        const auto inlier_indices = util::resample_by_indices(valid_indices, pnp_solver->get_inlier_flags());

        // set 2D-3D matches for the pose optimization
        curr_frm.landmarks_ = std::vector<data::landmark*>(curr_frm.num_keypts_, nullptr);
        std::set<data::landmark*> already_found_landmarks;
        for (const auto idx : inlier_indices) {
            // 有効な3次元点のみをcurrent frameにセット
            curr_frm.landmarks_.at(idx) = matched_landmarks.at(i).at(idx);
            // すでに特徴点と対応した3次元点を記録しておく
            already_found_landmarks.insert(matched_landmarks.at(i).at(idx));
        }

        // pose optimization
        auto num_valid_obs = pose_optimizer_.optimize(curr_frm);
        // discard the candidate if the number of the inliers is less than the threshold
        if (num_valid_obs < min_num_bow_matches_ / 2) {
            continue;
        }

        // reject outliers
        for (unsigned int idx = 0; idx < curr_frm.num_keypts_; idx++) {
            if (!curr_frm.outlier_flags_.at(idx)) {
                continue;
            }
            curr_frm.landmarks_.at(idx) = nullptr;
        }

        // 3. Apply projection match to increase 2D-3D matches

        // projection match based on the pre-optimized camera pose
        auto num_found = proj_matcher_.match_frame_and_keyframe(curr_frm, reloc_candidates.at(i), already_found_landmarks, 10, 100);
        // discard the candidate if the number of the inliers is less than the threshold
        if (num_valid_obs + num_found < min_num_valid_obs_) {
            continue;
        }

        // 4. Re-apply the pose optimizer

        num_valid_obs = pose_optimizer_.optimize(curr_frm);

        // 閾値未満になったら，もう一度projection matchを行う
        if (num_valid_obs < min_num_valid_obs_) {
            // すでに対応がついているものは除く
            already_found_landmarks.clear();
            for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
                if (!curr_frm.landmarks_.at(idx)) {
                    continue;
                }
                already_found_landmarks.insert(curr_frm.landmarks_.at(idx));
            }
            // もう一度projection matchを行う -> 2D-3D対応を設定
            auto num_additional = proj_matcher_.match_frame_and_keyframe(curr_frm, reloc_candidates.at(i), already_found_landmarks, 3, 64);

            // 閾値未満だったら破棄
            if (num_valid_obs + num_additional < min_num_valid_obs_) {
                continue;
            }

            // もう一度最適化
            num_valid_obs = pose_optimizer_.optimize(curr_frm);

            // 閾値未満だったら破棄
            if (num_valid_obs < min_num_valid_obs_) {
                continue;
            }
        }

        // relocalize成功
        spdlog::info("relocalization succeeded");
        // TODO: current frameのreference keyframeをセットする

        // reject outliers
        for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
            if (!curr_frm.outlier_flags_.at(idx)) {
                continue;
            }
            curr_frm.landmarks_.at(idx) = nullptr;
        }

        return true;
    }

    curr_frm.cam_pose_cw_is_valid_ = false;
    return false;
}

} // namespace module
} // namespace openvslam
