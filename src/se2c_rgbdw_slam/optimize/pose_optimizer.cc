#include "se2c_rgbdw_slam/data/frame.h"
#include "se2c_rgbdw_slam/data/keyframe.h"
#include "se2c_rgbdw_slam/data/landmark.h"
#include "se2c_rgbdw_slam/optimize/pose_optimizer.h"
#include "se2c_rgbdw_slam/optimize/g2o/se3/pose_opt_edge_wrapper.h"
#include "se2c_rgbdw_slam/util/converter.h"

#include "se2c_rgbdw_slam/optimize/g2o/se2constraint/plane_constraint_wrapper.h"
#include "se2c_rgbdw_slam/optimize/g2o/se2constraint/odometry_constraint_wrapper.h"
#include "se2c_rgbdw_slam/util/coordinate_transformer.h"

#include <vector>
#include <mutex>

#include <Eigen/StdVector>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

namespace openvslam {
namespace optimize {

pose_optimizer::pose_optimizer(const unsigned int num_trials, const unsigned int num_each_iter)
    : num_trials_(num_trials), num_each_iter_(num_each_iter) 
    {
    }

void pose_optimizer::set_se2c_wrappers(std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper>& plane_wrapper, std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper>& odom_wrapper)
{
    plane_constraint_wrapper_ = plane_wrapper;
    odometry_constraint_wrapper_ = odom_wrapper;
}


unsigned int pose_optimizer::optimize(data::frame& frm) const {
    // 1. optimizerを構築

    auto linear_solver = ::g2o::make_unique<::g2o::LinearSolverEigen<::g2o::BlockSolver_6_3::PoseMatrixType>>();
    auto block_solver = ::g2o::make_unique<::g2o::BlockSolver_6_3>(std::move(linear_solver));
    auto algorithm = new ::g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    ::g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);

    unsigned int num_init_obs = 0;

    // 有意水準5%のカイ2乗値
    // 自由度n=2
    constexpr float chi_sq_2D = 5.99146;
    const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);
    // 自由度n=3
    constexpr float chi_sq_3D = 7.81473;
    const float sqrt_chi_sq_3D = std::sqrt(chi_sq_3D);
    const auto sqrt_chi_sq = (frm.camera_->setup_type_ == camera::setup_type_t::Monocular)
                                ? sqrt_chi_sq_2D
                                : sqrt_chi_sq_3D;

    // 2. frameをg2oのvertexに変換してoptimizerにセットする

    auto frm_vtx = new g2o::se3::shot_vertex();
    frm_vtx->setId(frm.id_);
    frm_vtx->setEstimate(util::converter::to_g2o_SE3(frm.cam_pose_cw_));
    frm_vtx->setFixed(false);
    optimizer.addVertex(frm_vtx);

    const unsigned int num_keypts = frm.num_keypts_;

    // 3. landmarkのvertexをreprojection edgeで接続する

    // reprojection edgeのcontainer
    using pose_opt_edge_wrapper = g2o::se3::pose_opt_edge_wrapper<data::frame>;
    std::vector<pose_opt_edge_wrapper> pose_opt_edge_wraps;
    pose_opt_edge_wraps.reserve(num_keypts);



    for (unsigned int idx = 0; idx < num_keypts; ++idx) {
        auto lm = frm.landmarks_.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        ++num_init_obs;
        frm.outlier_flags_.at(idx) = false;

        const unsigned int num_observed = lm->num_observations();

        // frameのvertexをreprojection edgeで接続する
        const auto& undist_keypt = frm.undist_keypts_.at(idx);
        const float x_right = frm.stereo_x_right_.at(idx);
        const float inv_sigma_sq = frm.inv_level_sigma_sq_.at(undist_keypt.octave);
        // const auto sqrt_chi_sq = (frm.camera_->setup_type_ == camera::setup_type_t::Monocular)
        //                              ? sqrt_chi_sq_2D
        //                              : sqrt_chi_sq_3D;
        auto pose_opt_edge_wrap = pose_opt_edge_wrapper(&frm, frm_vtx, lm->get_pos_in_world(),
                                                        idx, undist_keypt.pt.x, undist_keypt.pt.y, x_right,
                                                        inv_sigma_sq, sqrt_chi_sq, num_observed);
        pose_opt_edge_wraps.push_back(pose_opt_edge_wrap);
        optimizer.addEdge(pose_opt_edge_wrap.edge_);
    }

    if (num_init_obs < 5) {
        return 0;
    }

    // Proposed
    // Frameがse2上にあるという拘束を追加する
    #ifdef ENABLE_PLANE_CONSTRAINT
    // std::cout << "before optimization: cam_pose_cw\n" << frm.cam_pose_cw_ << "\ninv_sigma_sq " << frm.inv_level_sigma_sq_.at(0) << " " << frm.inv_level_sigma_sq_.at(4) << std::endl; 
    optimize::g2o::se2c::plane_constraint_edge* plane_constraint = plane_constraint_wrapper_->create_plane_constraint(optimizer, frm.cam_pose_cw_, sqrt_chi_sq, frm.id_, num_init_obs);
    optimizer.addEdge(plane_constraint);
    // optimizer.setVerbose(true);
    // algorithm->printProperties(std::cout);
    // std::cout << "cam_pose\n" << frm.cam_pose_cw_ << std::endl; 
    // std::cout << "[pose_optimizer::optimize()] num LM edge: " << num_init_obs << std::endl;
    #endif

    #ifdef ENABLE_ODOMETRY_CONSTRAINT
    assert(frm.odom_from_ref_kf_ != nullptr);

    data::keyframe* keyfrm_from = frm.odom_from_ref_kf_->first;
    unsigned int keyfrm_from_id = keyfrm_from->id_;


    // 参照KFをfixして追加
    auto ref_kf_vtx = new g2o::se3::shot_vertex();
    ref_kf_vtx->setId(keyfrm_from_id);
    ref_kf_vtx->setEstimate(util::converter::to_g2o_SE3(keyfrm_from->get_cam_pose()));
    ref_kf_vtx->setFixed(true);
    optimizer.addVertex(ref_kf_vtx);



    // 拘束を追加
    auto odometry_constraint 
        = odometry_constraint_wrapper_->create_odometry_constraint( optimizer, 
                                                                    (frm.odom_from_ref_kf_)->second.first, 
                                                                    (frm.odom_from_ref_kf_)->second.second, 
                                                                    sqrt_chi_sq,
                                                                    keyfrm_from_id, 
                                                                    frm.id_,
                                                                    num_init_obs);
    optimizer.addEdge(odometry_constraint);
    // std::cout << "odom_measurement t-1 to t\n" << (frm.odom_from_ref_kf_)->second.first << std::endl;
    // // std::cout << "odom_info\n" << (frm.odom_from_ref_kf_)->second.second << std::endl;
    // const Mat44_t T_RC = (odometry_constraint_wrapper_->coord_transformer_)->get_rc();
    // std::cout << "current  t-1 to t\n" << T_RC * (keyfrm_from->get_cam_pose() * frm.cam_pose_cw_.inverse() ) * T_RC.inverse() << std::endl;
    #endif

    // optimizer.setVerbose(true);
    // algorithm->printProperties(std::cout);

    // 4. robust BAを実行する

    unsigned int num_bad_obs = 0;
    for (unsigned int trial = 0; trial < num_trials_; ++trial) {
        optimizer.initializeOptimization();
        optimizer.optimize(num_each_iter_);

        num_bad_obs = 0;

        for (auto& pose_opt_edge_wrap : pose_opt_edge_wraps) {
            auto edge = pose_opt_edge_wrap.edge_;

            if (frm.outlier_flags_.at(pose_opt_edge_wrap.idx_)) {
                edge->computeError();
            }

            if (pose_opt_edge_wrap.is_monocular_) {
                if (chi_sq_2D < edge->chi2()) {
                    frm.outlier_flags_.at(pose_opt_edge_wrap.idx_) = true;
                    pose_opt_edge_wrap.set_as_outlier();
                    ++num_bad_obs;
                }
                else {
                    frm.outlier_flags_.at(pose_opt_edge_wrap.idx_) = false;
                    pose_opt_edge_wrap.set_as_inlier();
                }
            }
            else {
                if (chi_sq_3D < edge->chi2()) {
                    frm.outlier_flags_.at(pose_opt_edge_wrap.idx_) = true;
                    pose_opt_edge_wrap.set_as_outlier();
                    ++num_bad_obs;
                }
                else {
                    frm.outlier_flags_.at(pose_opt_edge_wrap.idx_) = false;
                    pose_opt_edge_wrap.set_as_inlier();
                }
            }

            if (trial == num_trials_ - 2) {
                edge->setRobustKernel(nullptr);
            }
        }

        if (num_init_obs - num_bad_obs < 5) {
            break;
        }
    }

    // 5. 情報を更新

    frm.set_cam_pose(frm_vtx->estimate());

    return num_init_obs - num_bad_obs;
}

unsigned int pose_optimizer::optimize_with_plane_constraint(data::frame& frm) const
{
    // 1. optimizerを構築

    auto linear_solver = ::g2o::make_unique<::g2o::LinearSolverEigen<::g2o::BlockSolver_6_3::PoseMatrixType>>();
    auto block_solver = ::g2o::make_unique<::g2o::BlockSolver_6_3>(std::move(linear_solver));
    auto algorithm = new ::g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    ::g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);

    // 有意水準5%のカイ2乗値
    // 自由度n=2
    constexpr float chi_sq_2D = 5.99146;
    const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);
    // 自由度n=3
    constexpr float chi_sq_3D = 7.81473;
    const float sqrt_chi_sq_3D = std::sqrt(chi_sq_3D);
    const auto sqrt_chi_sq = (frm.camera_->setup_type_ == camera::setup_type_t::Monocular)
                                ? sqrt_chi_sq_2D
                                : sqrt_chi_sq_3D;

    // 2. frameをg2oのvertexに変換してoptimizerにセットする

    auto frm_vtx = new g2o::se3::shot_vertex();
    frm_vtx->setId(frm.id_);
    frm_vtx->setEstimate(util::converter::to_g2o_SE3(frm.cam_pose_cw_));
    frm_vtx->setFixed(false);
    optimizer.addVertex(frm_vtx);
    std::cout << "addvertex " << frm_vtx << std::endl;


    // Proposed
    // Frameがse2上にあるという拘束を追加する
    auto plane_constraint = plane_constraint_wrapper_->create_plane_constraint(optimizer, frm.cam_pose_cw_, sqrt_chi_sq, frm.id_);
    optimizer.addEdge(plane_constraint);
    // optimizer.setVerbose(true);
    // algorithm->printProperties(std::cout);

    optimizer.initializeOptimization();
    optimizer.optimize(num_each_iter_);

    frm.set_cam_pose(frm_vtx->estimate());

    return 0;
}


unsigned int pose_optimizer::optimize_with_odometry_constraint(const unsigned fix_frm_id , const Mat44_t& fix_frm_pose_cw , 
                                                               const unsigned& opt_frm_id, Mat44_t& opt_frm_pose_cw,
                                                               const Mat44_t& odom_measurement_bb, const Mat66_t& odom_measurement_info_bb) const
{
    // 1. optimizerを構築

    auto linear_solver = ::g2o::make_unique<::g2o::LinearSolverEigen<::g2o::BlockSolver_6_3::PoseMatrixType>>();
    auto block_solver = ::g2o::make_unique<::g2o::BlockSolver_6_3>(std::move(linear_solver));
    auto algorithm = new ::g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    ::g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);

    // 有意水準5%のカイ2乗値
    // 自由度n=2
    constexpr float chi_sq_2D = 5.99146;
    const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);
    // 自由度n=3
    constexpr float chi_sq_3D = 7.81473;
    const float sqrt_chi_sq_3D = std::sqrt(chi_sq_3D);
    // const auto sqrt_chi_sq = (frm.camera_->setup_type_ == camera::setup_type_t::Monocular)
    //                             ? sqrt_chi_sq_2D
    //                             : sqrt_chi_sq_3D;
    const auto sqrt_chi_sq = sqrt_chi_sq_3D;

    // 2. frameをg2oのvertexに変換してoptimizerにセットする

    // fix frame
    auto fix_frm_vtx = new g2o::se3::shot_vertex();
    fix_frm_vtx->setId(fix_frm_id);
    fix_frm_vtx->setEstimate(util::converter::to_g2o_SE3(fix_frm_pose_cw));
    fix_frm_vtx->setFixed(true);
    optimizer.addVertex(fix_frm_vtx);
    // std::cout << "addvertex " << fix_frm_vtx << std::endl;

    // opt frame
    auto opt_frm_vtx = new g2o::se3::shot_vertex();
    opt_frm_vtx->setId(opt_frm_id);
    opt_frm_vtx->setEstimate(util::converter::to_g2o_SE3(opt_frm_pose_cw));
    opt_frm_vtx->setFixed(false);
    optimizer.addVertex(opt_frm_vtx);
    // std::cout << "addvertex " << opt_frm_vtx << std::endl;

    const Mat44_t T_rc = odometry_constraint_wrapper_->coord_transformer_->get_rc();
    const Mat44_t T_cr = odometry_constraint_wrapper_->coord_transformer_->get_cr();
    std::cout << "before estimate" << std::endl
              << "fix_frame pose" << std::endl
              << fix_frm_pose_cw << std::endl
              << "opt_frame_pose" << std::endl
              << opt_frm_pose_cw << std::endl
              << "displacement in odom coord" << std::endl
              << T_rc * fix_frm_pose_cw * opt_frm_pose_cw.inverse() * T_cr << std::endl;
    std::cout << "odom displacement\n" << odom_measurement_bb << std::endl;


    // Proposed
    // Frameがse2上にあるという拘束を追加する
    constexpr int dummy_int = 100;
    auto odometry_constraint 
        = odometry_constraint_wrapper_->create_odometry_constraint( optimizer, 
                                                                    odom_measurement_bb, 
                                                                    odom_measurement_info_bb, 
                                                                    sqrt_chi_sq,
                                                                    fix_frm_id, 
                                                                    opt_frm_id,
                                                                    dummy_int);
    optimizer.addEdge(odometry_constraint);
    // optimizer.setVerbose(true);
    // algorithm->printProperties(std::cout);

    optimizer.initializeOptimization();
    optimizer.optimize(num_each_iter_);

    // std::cout << "\n\n";

    opt_frm_pose_cw = opt_frm_vtx->estimate().to_homogeneous_matrix();
    std::cout << "after estimate" << std::endl
              << "fix_frame pose" << std::endl
              << fix_frm_pose_cw << std::endl
              << "opt_frame_pose" << std::endl
              << opt_frm_pose_cw << std::endl
              << "displacement in odom coord" << std::endl
              << T_rc * fix_frm_pose_cw * opt_frm_pose_cw.inverse() * T_cr << std::endl;

    std::cout << "odom displacement\n" << odom_measurement_bb << std::endl;
    std::cout << std::endl;

    return 0;
}

} // namespace optimize
} // namespace openvslam
