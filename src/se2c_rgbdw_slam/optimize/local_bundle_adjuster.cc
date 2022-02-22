#include "se2c_rgbdw_slam/data/keyframe.h"
#include "se2c_rgbdw_slam/data/landmark.h"
#include "se2c_rgbdw_slam/data/map_database.h"
#include "se2c_rgbdw_slam/optimize/local_bundle_adjuster.h"
#include "se2c_rgbdw_slam/optimize/g2o/landmark_vertex_container.h"
#include "se2c_rgbdw_slam/optimize/g2o/se3/shot_vertex_container.h"
#include "se2c_rgbdw_slam/optimize/g2o/se3/reproj_edge_wrapper.h"
#include "se2c_rgbdw_slam/optimize/g2o/se2constraint/plane_constraint_wrapper.h"
#include "se2c_rgbdw_slam/optimize/g2o/se2constraint/odometry_constraint_wrapper.h"
#include "se2c_rgbdw_slam/util/converter.h"

#include <unordered_map>

#include <Eigen/StdVector>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

namespace openvslam {
namespace optimize {

local_bundle_adjuster::local_bundle_adjuster(const unsigned int num_first_iter,
                                             const unsigned int num_second_iter)
    : num_first_iter_(num_first_iter), num_second_iter_(num_second_iter)
{
}

void local_bundle_adjuster::set_se2c_wrappers(std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper>& plane_wrapper, std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper>& odom_wrapper)
{
    plane_constraint_wrapper_ = plane_wrapper;
    odometry_constraint_wrapper_ = odom_wrapper;
}


void local_bundle_adjuster::optimize(openvslam::data::keyframe* curr_keyfrm, bool* const force_stop_flag) const {
    // 1. local/fixed keyframes, local landmarksを集計する

    // correct local keyframes of the current keyframe
    std::unordered_map<unsigned int, data::keyframe*> local_keyfrms;

    local_keyfrms[curr_keyfrm->id_] = curr_keyfrm;
    const auto curr_covisibilities = curr_keyfrm->graph_node_->get_covisibilities();
    for (auto local_keyfrm : curr_covisibilities) {
        if (!local_keyfrm) {
            continue;
        }
        if (local_keyfrm->will_be_erased()) {
            continue;
        }

        local_keyfrms[local_keyfrm->id_] = local_keyfrm;
    }

    // correct local landmarks seen in local keyframes
    std::unordered_map<unsigned int, data::landmark*> local_lms;

    for (auto local_keyfrm : local_keyfrms) {
        const auto landmarks = local_keyfrm.second->get_landmarks();
        for (auto local_lm : landmarks) {
            if (!local_lm) {
                continue;
            }
            if (local_lm->will_be_erased()) {
                continue;
            }

            // 重複を避ける
            if (local_lms.count(local_lm->id_)) {
                continue;
            }

            local_lms[local_lm->id_] = local_lm;
        }
    }

    // fixed keyframes: keyframes which observe local landmarks but which are NOT in local keyframes
    std::unordered_map<unsigned int, data::keyframe*> fixed_keyfrms;

    for (auto local_lm : local_lms) {
        const auto observations = local_lm.second->get_observations();
        for (auto& obs : observations) {
            auto fixed_keyfrm = obs.first;
            if (!fixed_keyfrm) {
                continue;
            }
            if (fixed_keyfrm->will_be_erased()) {
                continue;
            }

            // local keyframesに属しているときは追加しない
            if (local_keyfrms.count(fixed_keyfrm->id_)) {
                continue;
            }

            // 重複を避ける
            if (fixed_keyfrms.count(fixed_keyfrm->id_)) {
                continue;
            }

            fixed_keyfrms[fixed_keyfrm->id_] = fixed_keyfrm;
        }
    }

    // 2. optimizerを構築

    auto linear_solver = ::g2o::make_unique<::g2o::LinearSolverCSparse<::g2o::BlockSolver_6_3::PoseMatrixType>>();
    auto block_solver = ::g2o::make_unique<::g2o::BlockSolver_6_3>(std::move(linear_solver));
    auto algorithm = new ::g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    ::g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);

    if (force_stop_flag) {
        optimizer.setForceStopFlag(force_stop_flag);
    }

    // 有意水準5%のカイ2乗値
    // 自由度n=2
    constexpr float chi_sq_2D = 5.99146;
    const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);
    // 自由度n=3
    constexpr float chi_sq_3D = 7.81473;
    const float sqrt_chi_sq_3D = std::sqrt(chi_sq_3D);
    const auto sqrt_chi_sq = (curr_keyfrm->camera_->setup_type_ == camera::setup_type_t::Monocular)
                                         ? sqrt_chi_sq_2D
                                         : sqrt_chi_sq_3D;
    // 3. keyframeをg2oのvertexに変換してoptimizerにセットする

    // shot vertexのcontainer
    g2o::se3::shot_vertex_container keyfrm_vtx_container(0, local_keyfrms.size() + fixed_keyfrms.size());
    // vertexに変換されたkeyframesを保存しておく
    std::unordered_map<unsigned int, data::keyframe*> all_keyfrms;

    // あとでSE2Cを追加するために保存しておく、keyframe idとfeature constraintの数のペア
    std::unordered_map<unsigned int, int> keyfrm_id_and_num_reproj_lm;

    // local keyframesをoptimizerにセット
    for (auto& id_local_keyfrm_pair : local_keyfrms) {
        auto local_keyfrm = id_local_keyfrm_pair.second;

        all_keyfrms.emplace(id_local_keyfrm_pair);
        keyfrm_id_and_num_reproj_lm[local_keyfrm->id_] = 0;
        auto keyfrm_vtx = keyfrm_vtx_container.create_vertex(local_keyfrm, local_keyfrm->id_ == 0);
        optimizer.addVertex(keyfrm_vtx);

        //! Proposed
        // #ifdef ENABLE_PLANE_CONSTRAINT
        // auto plane_constraint = plane_constraint_wrapper_->create_plane_constraint(optimizer, local_keyfrm->get_cam_pose(), sqrt_chi_sq, local_keyfrm->id_);
        // optimizer.addEdge(plane_constraint);
        // #endif

        // #ifdef ENABLE_ODOMETRY_CONSTRAINT
        // if(local_keyfrm->odom_from_ref_kf_ == nullptr) continue;

        // data::keyframe* keyfrm_from = local_keyfrm->odom_from_ref_kf_->first;

        // if(keyfrm_from == nullptr)
        // {
        //     std::cout << "[local_bundle_adjuster::optimize()] keyfrm_from is nullptr\n";
        //     continue;
        // }
        // unsigned int keyfrm_from_id = keyfrm_from->id_;
        // auto result = local_keyfrms.find(keyfrm_from_id);
        // if(result == local_keyfrms.end()) continue;

        // auto odometry_constraint 
        //     = odometry_constraint_wrapper_->create_odometry_constraint( optimizer, 
        //                                                                 (local_keyfrm->odom_from_ref_kf_)->second.first, 
        //                                                                 (local_keyfrm->odom_from_ref_kf_)->second.second, 
        //                                                                 sqrt_chi_sq,
        //                                                                 keyfrm_from_id, 
        //                                                                 local_keyfrm->id_ );
        // optimizer.addEdge(odometry_constraint);

        // // std::cout << "[local_bundle_adjuster::optimize()]: this KF id(vId2): " << local_keyfrm->id_ << " from_id(vId1): " << keyfrm_from_id << std::endl;
        // // std::cout << "addVertex id: " << local_keyfrm->id_ << std::endl;

        // #endif
    }

    // fixed keyframesをoptimizerにセット
    for (auto& id_fixed_keyfrm_pair : fixed_keyfrms) {
        auto fixed_keyfrm = id_fixed_keyfrm_pair.second;

        all_keyfrms.emplace(id_fixed_keyfrm_pair);
        keyfrm_id_and_num_reproj_lm[fixed_keyfrm->id_] = 0;
        auto keyfrm_vtx = keyfrm_vtx_container.create_vertex(fixed_keyfrm, true);
        optimizer.addVertex(keyfrm_vtx);

        //! Proposed
        // #ifdef ENABLE_PLANE_CONSTRAINT
        // auto plane_constraint = plane_constraint_wrapper_->create_plane_constraint(optimizer, fixed_keyfrm->get_cam_pose(), sqrt_chi_sq, fixed_keyfrm->id_);
        // optimizer.addEdge(plane_constraint);
        // #endif

        // std::cout << "fixed KF addVertex id: " << fixed_keyfrm->id_ << std::endl;
    }

    // 4. keyframeとlandmarkのvertexをreprojection edgeで接続する

    // landmark vertexのcontainer
    g2o::landmark_vertex_container lm_vtx_container(keyfrm_vtx_container.get_max_vertex_id() + 1, local_lms.size());

    // reprojection edgeのcontainer
    using reproj_edge_wrapper = g2o::se3::reproj_edge_wrapper<data::keyframe>;
    std::vector<reproj_edge_wrapper> reproj_edge_wraps;
    reproj_edge_wraps.reserve(all_keyfrms.size() * local_lms.size());


    for (auto& id_local_lm_pair : local_lms) {
        auto local_lm = id_local_lm_pair.second;

        // landmarkをg2oのvertexに変換してoptimizerにセットする
        auto lm_vtx = lm_vtx_container.create_vertex(local_lm, false);
        optimizer.addVertex(lm_vtx);

        const auto observations = local_lm->get_observations();
        for (const auto& obs : observations) {
            auto keyfrm = obs.first;
            auto idx = obs.second;
            if (!keyfrm) {
                continue;
            }
            if (keyfrm->will_be_erased()) {
                continue;
            }

            const auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
            const auto& undist_keypt = keyfrm->undist_keypts_.at(idx);
            const float x_right = keyfrm->stereo_x_right_.at(idx);
            const float inv_sigma_sq = keyfrm->inv_level_sigma_sq_.at(undist_keypt.octave);

            // 対象のKF idの再投影誤差カウンタを1つ増やす
            keyfrm_id_and_num_reproj_lm[keyfrm->id_]++;

            // const auto sqrt_chi_sq = (keyfrm->camera_->setup_type_ == camera::setup_type_t::Monocular)
            //                              ? sqrt_chi_sq_2D
            //                              : sqrt_chi_sq_3D;
            const unsigned int num_observed = local_lm->num_observations();
            auto reproj_edge_wrap = reproj_edge_wrapper(keyfrm, keyfrm_vtx, local_lm, lm_vtx,
                                                        idx, undist_keypt.pt.x, undist_keypt.pt.y, x_right,
                                                        inv_sigma_sq, sqrt_chi_sq, num_observed);
            reproj_edge_wraps.push_back(reproj_edge_wrap);
            optimizer.addEdge(reproj_edge_wrap.edge_);
        }
    }

    //! Proposed
    // SE2Cを追加
    // std::cout << "local BA\n";
    for(const auto& it : keyfrm_id_and_num_reproj_lm)
    {
        auto keyfrm_id = it.first;
        auto keyfrm = all_keyfrms[keyfrm_id];

        #ifdef ENABLE_PLANE_CONSTRAINT
        auto plane_constraint = plane_constraint_wrapper_->create_plane_constraint(optimizer, keyfrm->get_cam_pose(), sqrt_chi_sq, keyfrm->id_, it.second);
        optimizer.addEdge(plane_constraint);
        #endif

        #ifdef ENABLE_ODOMETRY_CONSTRAINT
        if(keyfrm->odom_from_ref_kf_ == nullptr) continue;

        data::keyframe* keyfrm_from = keyfrm->odom_from_ref_kf_->first;

        if(keyfrm_from == nullptr)
        {
            std::cout << "[local_bundle_adjuster::optimize()] keyfrm_from is nullptr\n";
            continue;
        }
        unsigned int keyfrm_from_id = keyfrm_from->id_;
        auto result = all_keyfrms.find(keyfrm_from_id);
        if(result == all_keyfrms.end()) continue;

        auto odometry_constraint 
            = odometry_constraint_wrapper_->create_odometry_constraint( optimizer, 
                                                                        (keyfrm->odom_from_ref_kf_)->second.first, 
                                                                        (keyfrm->odom_from_ref_kf_)->second.second, 
                                                                        sqrt_chi_sq,
                                                                        keyfrm_from_id, 
                                                                        keyfrm->id_,
                                                                        it.second
                                                                        );
        optimizer.addEdge(odometry_constraint);

        // std::cout << "[local_bundle_adjuster::optimize()]: this KF id(vId2): " << keyfrm->id_ << " from_id(vId1): " << keyfrm_from_id << std::endl;
        // std::cout << "addVertex id: " << keyfrm->id_ << std::endl;
        #endif
    }


    // 5. 1回目の最適化を実行

    if (force_stop_flag) {
        if (*force_stop_flag) {
            return;
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(num_first_iter_);

    // 6. アウトライア除去をして2回目の最適化を実行

    bool run_robust_BA = true;

    if (force_stop_flag) {
        if (*force_stop_flag) {
            run_robust_BA = false;
        }
    }

    if (run_robust_BA) {
        for (auto& reproj_edge_wrap : reproj_edge_wraps) {
            auto edge = reproj_edge_wrap.edge_;

            auto local_lm = reproj_edge_wrap.lm_;
            if (local_lm->will_be_erased()) {
                continue;
            }

            if (reproj_edge_wrap.is_monocular_) {
                if (chi_sq_2D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                    reproj_edge_wrap.set_as_outlier();
                }
            }
            else {
                if (chi_sq_3D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                    reproj_edge_wrap.set_as_outlier();
                }
            }

            edge->setRobustKernel(nullptr);
        }

        optimizer.initializeOptimization();
        optimizer.optimize(num_second_iter_);
    }

    // 7. アウトライアを集計する

    std::vector<std::pair<data::keyframe*, data::landmark*>> outlier_observations;
    outlier_observations.reserve(reproj_edge_wraps.size());

    for (auto& reproj_edge_wrap : reproj_edge_wraps) {
        auto edge = reproj_edge_wrap.edge_;

        auto local_lm = reproj_edge_wrap.lm_;
        if (local_lm->will_be_erased()) {
            continue;
        }

        if (reproj_edge_wrap.is_monocular_) {
            if (chi_sq_2D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                outlier_observations.emplace_back(std::make_pair(reproj_edge_wrap.shot_, reproj_edge_wrap.lm_));
            }
        }
        else {
            if (chi_sq_3D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                outlier_observations.emplace_back(std::make_pair(reproj_edge_wrap.shot_, reproj_edge_wrap.lm_));
            }
        }
    }

    // 8. 情報を更新

    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        if (!outlier_observations.empty()) {
            for (auto& outlier_obs : outlier_observations) {
                auto keyfrm = outlier_obs.first;
                auto lm = outlier_obs.second;
                keyfrm->erase_landmark(lm);
                lm->erase_observation(keyfrm);
            }
        }

        for (auto id_local_keyfrm_pair : local_keyfrms) {
            auto local_keyfrm = id_local_keyfrm_pair.second;

            auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(local_keyfrm);
            local_keyfrm->set_cam_pose(keyfrm_vtx->estimate());
        }

        for (auto id_local_lm_pair : local_lms) {
            auto local_lm = id_local_lm_pair.second;

            auto lm_vtx = lm_vtx_container.get_vertex(local_lm);
            local_lm->set_pos_in_world(lm_vtx->estimate());
            local_lm->update_normal_and_depth();
        }
    }
}

} // namespace optimize
} // namespace openvslam
