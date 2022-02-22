#include "se2c_rgbdw_slam/data/keyframe.h"
#include "se2c_rgbdw_slam/data/landmark.h"
#include "se2c_rgbdw_slam/data/map_database.h"
#include "se2c_rgbdw_slam/optimize/global_bundle_adjuster.h"
#include "se2c_rgbdw_slam/optimize/g2o/landmark_vertex_container.h"
#include "se2c_rgbdw_slam/optimize/g2o/se3/shot_vertex_container.h"
#include "se2c_rgbdw_slam/optimize/g2o/se3/reproj_edge_wrapper.h"
#include "se2c_rgbdw_slam/optimize/g2o/se2constraint/plane_constraint_wrapper.h"
#include "se2c_rgbdw_slam/optimize/g2o/se2constraint/odometry_constraint_wrapper.h"
#include "se2c_rgbdw_slam/util/converter.h"

#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

namespace openvslam {
namespace optimize {

global_bundle_adjuster::global_bundle_adjuster(data::map_database* map_db, const unsigned int num_iter, const bool use_huber_kernel)
    : map_db_(map_db), num_iter_(num_iter), use_huber_kernel_(use_huber_kernel)
    {
    }

void global_bundle_adjuster::set_se2c_wrappers(std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper>& plane_wrapper, std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper>& odom_wrapper)
{
    plane_constraint_wrapper_ = plane_wrapper;
    odometry_constraint_wrapper_ = odom_wrapper;
}

void global_bundle_adjuster::optimize(const unsigned int lead_keyfrm_id_in_global_BA, bool* const force_stop_flag) const {
    // 1. データを集める

    const auto keyfrms = map_db_->get_all_keyframes();
    const auto lms = map_db_->get_all_landmarks();
    std::vector<bool> is_optimized_lm(lms.size(), true);


    // std::cout << "[global_bundle_adjuster::optimize()] num all KF: " << keyfrms.size() << std::endl;

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
    const auto sqrt_chi_sq = ((*keyfrms.begin())->camera_->setup_type_ == camera::setup_type_t::Monocular)
                                         ? sqrt_chi_sq_2D
                                         : sqrt_chi_sq_3D;

    // 3. keyframeをg2oのvertexに変換してoptimizerにセットする

    // shot vertexのcontainer
    g2o::se3::shot_vertex_container keyfrm_vtx_container(0, keyfrms.size());
    int num_vertex = 0;

    std::unordered_map<unsigned int, data::keyframe*> all_keyfrms;
    // あとでSE2Cを追加するために保存しておく、keyframe idとfeature constraintの数のペア
    std::unordered_map<unsigned int, int> keyfrm_id_and_num_reproj_lm;

    // keyframesをoptimizerにセット
    for (const auto keyfrm : keyfrms) {
        if (!keyfrm) {
            continue;
        }
        if (keyfrm->will_be_erased()) {
            continue;
        }

        all_keyfrms[keyfrm->id_] = keyfrm;
        keyfrm_id_and_num_reproj_lm[keyfrm->id_] = 0;
        auto keyfrm_vtx = keyfrm_vtx_container.create_vertex(keyfrm, keyfrm->id_ == 0);
        optimizer.addVertex(keyfrm_vtx);
        num_vertex++;

        // //! Proposed
        // #ifdef ENABLE_PLANE_CONSTRAINT
        // auto plane_constraint = plane_constraint_wrapper_->create_plane_constraint(optimizer, keyfrm->get_cam_pose(), sqrt_chi_sq, keyfrm->id_);
        // optimizer.addEdge(plane_constraint);
        // #endif

        // #ifdef ENABLE_ODOMETRY_CONSTRAINT
        // if(keyfrm->odom_from_ref_kf_ == nullptr)
        // {
        //     // std::cout << "keyfrm->odom_from_ref_kf_ == nullptr " << keyfrm->id_ << std::endl;
        //     continue;
        // } 

        // data::keyframe* keyfrm_from = keyfrm->odom_from_ref_kf_->first;

        // if(keyfrm_from == nullptr)
        // {
        //     std::cout << "[global_bundle_adjuster::optimize()] keyfrm_from is nullptr\n";
        //     continue;
        // }
        
        // auto odometry_constraint = odometry_constraint_wrapper_->create_odometry_constraint( optimizer, 
        //                                                                                      (keyfrm->odom_from_ref_kf_)->second.first, 
        //                                                                                      (keyfrm->odom_from_ref_kf_)->second.second, 
        //                                                                                      sqrt_chi_sq,
        //                                                                                      keyfrm_from->id_, 
        //                                                                                      keyfrm->id_);
        // optimizer.addEdge(odometry_constraint);

        // // std::cout << "[global_bundle_adjuster::optimize()]: this KF id(vId2): " << keyfrm->id_ << " from_id(vId1): " << keyfrm_from->id_ << std::endl;
        // // std::cout << "addVertex id: " << keyfrm->id_ << std::endl;
        // #endif
    }
    // std::cout << "[global_bundle_adjuster::optimize()] num vertex: " << num_vertex << std::endl;

    // 4. keyframeとlandmarkのvertexをreprojection edgeで接続する

    // landmark vertexのcontainer
    g2o::landmark_vertex_container lm_vtx_container(keyfrm_vtx_container.get_max_vertex_id() + 1, lms.size());

    // reprojection edgeのcontainer
    using reproj_edge_wrapper = g2o::se3::reproj_edge_wrapper<data::keyframe>;
    std::vector<reproj_edge_wrapper> reproj_edge_wraps;
    reproj_edge_wraps.reserve(10 * lms.size());

    for (unsigned int i = 0; i < lms.size(); ++i) {
        auto lm = lms.at(i);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        // landmarkをg2oのvertexに変換してoptimizerにセットする
        auto lm_vtx = lm_vtx_container.create_vertex(lm, false);
        optimizer.addVertex(lm_vtx);

        unsigned int num_edges = 0;
        const auto observations = lm->get_observations();
        for (const auto& obs : observations) {
            auto keyfrm = obs.first;
            auto idx = obs.second;
            if (!keyfrm) {
                continue;
            }
            if (keyfrm->will_be_erased()) {
                continue;
            }

            if (!keyfrm_vtx_container.contain(keyfrm)) {
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
            const unsigned int num_observed = lm->num_observations();
            auto reproj_edge_wrap = reproj_edge_wrapper(keyfrm, keyfrm_vtx, lm, lm_vtx,
                                                        idx, undist_keypt.pt.x, undist_keypt.pt.y, x_right,
                                                        inv_sigma_sq, sqrt_chi_sq, num_observed, use_huber_kernel_);
            reproj_edge_wraps.push_back(reproj_edge_wrap);
            optimizer.addEdge(reproj_edge_wrap.edge_);
            ++num_edges;
        }

        if (num_edges == 0) {
            optimizer.removeVertex(lm_vtx);
            is_optimized_lm.at(i) = false;
        }
    }


    //! Proposed
    // SE2Cを追加
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



    // 5. 最適化を実行

    optimizer.initializeOptimization();
    optimizer.optimize(num_iter_);

    if (force_stop_flag && *force_stop_flag) {
        return;
    }

    // 6. 結果を取り出す

    for (auto keyfrm : keyfrms) {
        if (keyfrm->will_be_erased()) {
            continue;
        }
        auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
        const auto cam_pose_cw = util::converter::to_eigen_mat(keyfrm_vtx->estimate());
        if (lead_keyfrm_id_in_global_BA == 0) {
            keyfrm->set_cam_pose(cam_pose_cw);
        }
        else {
            keyfrm->cam_pose_cw_after_loop_BA_ = cam_pose_cw;
            keyfrm->loop_BA_identifier_ = lead_keyfrm_id_in_global_BA;
        }
    }

    for (unsigned int i = 0; i < lms.size(); ++i) {
        if (!is_optimized_lm.at(i)) {
            continue;
        }

        auto lm = lms.at(i);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        auto lm_vtx = lm_vtx_container.get_vertex(lm);
        const Vec3_t pos_w = lm_vtx->estimate();

        if (lead_keyfrm_id_in_global_BA == 0) {
            lm->set_pos_in_world(pos_w);
            lm->update_normal_and_depth();
        }
        else {
            lm->pos_w_after_global_BA_ = pos_w;
            lm->loop_BA_identifier_ = lead_keyfrm_id_in_global_BA;
        }
    }
}

} // namespace optimize
} // namespace openvslam
