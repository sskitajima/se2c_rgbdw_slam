#include "se2c_rgbdw_slam/data/frame.h"
#include "se2c_rgbdw_slam/match/area.h"
#include "se2c_rgbdw_slam/match/angle_checker.h"

namespace openvslam {
namespace match {

unsigned int area::match_in_consistent_area(data::frame& frm_1, data::frame& frm_2, std::vector<cv::Point2f>& prev_matched_pts,
                                            std::vector<int>& matched_indices_2_in_frm_1, int margin) {
    unsigned int num_matches = 0;

    angle_checker<int> angle_checker;

    matched_indices_2_in_frm_1 = std::vector<int>(frm_1.undist_keypts_.size(), -1);

    std::vector<unsigned int> matched_dists_in_frm_2(frm_2.undist_keypts_.size(), MAX_HAMMING_DIST);
    std::vector<int> matched_indices_1_in_frm_2(frm_2.undist_keypts_.size(), -1);

    for (unsigned int idx_1 = 0; idx_1 < frm_1.undist_keypts_.size(); ++idx_1) {
        const auto& undist_keypt_1 = frm_1.undist_keypts_.at(idx_1);
        const auto scale_level_1 = undist_keypt_1.octave;

        // 第0スケールの特徴点のみを用いる
        if (0 < scale_level_1) {
            continue;
        }

        // 一つ前にマッチングした特徴点周辺のcellの特徴点を持ってくる
        const auto indices = frm_2.get_keypoints_in_cell(prev_matched_pts.at(idx_1).x, prev_matched_pts.at(idx_1).y,
                                                         margin, scale_level_1, scale_level_1);
        if (indices.empty()) {
            continue;
        }

        const auto& desc_1 = frm_1.descriptors_.row(idx_1);

        unsigned int best_hamm_dist = MAX_HAMMING_DIST;
        unsigned int second_best_hamm_dist = MAX_HAMMING_DIST;
        int best_idx_2 = -1;

        for (const auto idx_2 : indices) {
            const auto& desc_2 = frm_2.descriptors_.row(idx_2);

            const auto hamm_dist = compute_descriptor_distance_32(desc_1, desc_2);

            // すでにマッチした点のほうが近ければスルーする
            if (matched_dists_in_frm_2.at(idx_2) <= hamm_dist) {
                continue;
            }

            if (hamm_dist < best_hamm_dist) {
                second_best_hamm_dist = best_hamm_dist;
                best_hamm_dist = hamm_dist;
                best_idx_2 = idx_2;
            }
            else if (hamm_dist < second_best_hamm_dist) {
                second_best_hamm_dist = hamm_dist;
            }
        }

        if (HAMMING_DIST_THR_LOW < best_hamm_dist) {
            continue;
        }

        // ratio test
        if (second_best_hamm_dist * lowe_ratio_ < static_cast<float>(best_hamm_dist)) {
            continue;
        }

        // idx_1 - best_idx_2 がベストな対応と推定

        // best_idx_2に対応する点がすでに存在する場合(=prev_idx_1)は，
        // 上書きするため対応するmatched_indices_2_in_frm_1.at(prev_idx_1)の対応情報を削除する必要がある
        // (matched_indices_1_in_frm_2.at(best_idx_2)は上書きするので削除する必要がない)
        const auto prev_idx_1 = matched_indices_1_in_frm_2.at(best_idx_2);
        if (0 <= prev_idx_1) {
            matched_indices_2_in_frm_1.at(prev_idx_1) = -1;
            --num_matches;
        }

        // 互いの対応情報を記録する
        matched_indices_2_in_frm_1.at(idx_1) = best_idx_2;
        matched_indices_1_in_frm_2.at(best_idx_2) = idx_1;
        matched_dists_in_frm_2.at(best_idx_2) = best_hamm_dist;
        ++num_matches;

        if (check_orientation_) {
            const auto delta_angle
                = frm_1.undist_keypts_.at(idx_1).angle - frm_2.undist_keypts_.at(best_idx_2).angle;
            angle_checker.append_delta_angle(delta_angle, idx_1);
        }
    }

    if (check_orientation_) {
        const auto invalid_matches = angle_checker.get_invalid_matches();
        for (const auto invalid_idx_1 : invalid_matches) {
            if (0 <= matched_indices_2_in_frm_1.at(invalid_idx_1)) {
                matched_indices_2_in_frm_1.at(invalid_idx_1) = -1;
                --num_matches;
            }
        }
    }

    // previous matchesを更新する
    for (unsigned int idx_1 = 0; idx_1 < matched_indices_2_in_frm_1.size(); ++idx_1) {
        if (0 <= matched_indices_2_in_frm_1.at(idx_1)) {
            prev_matched_pts.at(idx_1) = frm_2.undist_keypts_.at(matched_indices_2_in_frm_1.at(idx_1)).pt;
        }
    }

    return num_matches;
}

} // namespace match
} // namespace openvslam
