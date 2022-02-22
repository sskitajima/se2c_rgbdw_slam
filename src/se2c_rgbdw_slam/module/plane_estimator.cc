#include "se2c_rgbdw_slam/module/plane_estimator.h"
#include "se2c_rgbdw_slam/util/random_array.h"

#include <chrono>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

namespace openvslam
{

namespace module
{

plane_estimator::plane_estimator(const YAML::Node& yaml_node)
: thres_inlier_(yaml_node["PlaneEstimate.thres_inlier"].as<double>(0.01)), 
  num_ransac_iter_(yaml_node["PlaneEstimate.num_ransac_iter"].as<unsigned>(100)), 
  fx_(yaml_node["Camera.fx"].as<double>(530)),
  fy_(yaml_node["Camera.fy"].as<double>(530)),
  cx_(yaml_node["Camera.cx"].as<double>(475)),
  cy_(yaml_node["Camera.cy"].as<double>(265)),
  rows_(yaml_node["Camera.rows"].as<int>(540)),
  cols_(yaml_node["Camera.cols"].as<int>(960)),
  cam_dmin_(yaml_node["Octomap.min_depth"].as<double>(0.02)),
  cam_dmax_(yaml_node["Octomap.max_depth"].as<double>(4.5))
{
    spdlog::debug("CONSTRUCT: plane_estimator");

    auto yaml_transform = yaml_node["transform_rc"];
    transform_rc_default_ <<  yaml_transform[0][0].as<double>(0), yaml_transform[0][1].as<double>(0), yaml_transform[0][2].as<double>(1), yaml_transform[0][3].as<double>(0),
                              yaml_transform[1][0].as<double>(-1), yaml_transform[1][1].as<double>(0), yaml_transform[1][2].as<double>(0), yaml_transform[1][3].as<double>(0),
                              yaml_transform[2][0].as<double>(0), yaml_transform[2][1].as<double>(-1), yaml_transform[2][2].as<double>(0), yaml_transform[2][3].as<double>(0),
                              yaml_transform[3][0].as<double>(0), yaml_transform[3][1].as<double>(0), yaml_transform[3][2].as<double>(0), yaml_transform[3][3].as<double>(1);
    R_rc_base_ << 0, 0, 1, 
                 -1, 0, 0,
                 0, -1, 0;
}


plane_estimator::~plane_estimator()
{
    spdlog::debug("DESTRUCT: config");
}

///////////////////////////////////////////////////////////////////
// private

Vec4_t plane_estimator::estimate_ransac(const std::vector<Vec4_t, Eigen::aligned_allocator<Eigen::Vector4d>>& points, std::vector<int>& inlier_idx) const
{
    // double best_score = std::numeric_limits<double>::max();
    // Vec4_t best_parameter;

    std::vector<double> scores(num_ransac_iter_, std::numeric_limits<double>::max());
    std::vector<Vec4_t, Eigen::aligned_allocator<Eigen::Vector4d>> parameters(num_ransac_iter_, Vec4_t::Zero());


    // ransac loop
    #ifdef USE_OPENMP
    #pragma omp parallel for
    #endif
    for(size_t i=0; i<num_ransac_iter_; i++)
    {

        // ランダムに点を取得
        const auto random_indices = openvslam::util::create_random_array(3, 0, static_cast<int>(points.size())-1);
        std::vector<Vec4_t, Eigen::aligned_allocator<Eigen::Vector4d>> sample_points;
        for(size_t i=0; i<3; i++)
        {
            sample_points.push_back(points.at(random_indices.at(i)));
        }

        // 取得した点から平面のパラメータを取得
        auto plane_parameters = plane_estimator::find_plane_params(sample_points);


        // すべての点について、その平面との距離を計算
        // 和をスコアとする
        double score = 0;
        for(auto pt: points)
        {
            double dist = plane_estimator::formula_point_and_plane(pt, plane_parameters);
            score += dist;
        }

        scores[i] = score;
        parameters[i] = plane_parameters;

        // if(score < best_score)
        // {
        //     best_parameter(0) = plane_parameters(0);
        //     best_parameter(1) = plane_parameters(1);
        //     best_parameter(2) = plane_parameters(2);
        //     best_parameter(3) = plane_parameters(3);

        //     best_score = score;

        // }
    }

    size_t best_idx = std::distance(scores.begin(), std::min_element(scores.begin(), scores.end()));
    Vec4_t best_parameter = parameters[best_idx];
    // double best_score = scores[best_idx];

    scores.clear();
    parameters.clear();

    // inlierのindexを計算
    cal_inlier(inlier_idx, best_parameter, points);

    // std::cout << "\n[RANSAC]\n";
    // std::cout << "best parameter a: " << best_parameter.transpose() << std::endl;
    // std::cout << "num points " << points.size() << " best score: " << best_score << std::endl;
    // std::cout << "num inlier " << inlier_idx.size() << std::endl;
    // std::cout << std::endl;


    return best_parameter;
}


Vec4_t plane_estimator::estimate_ls(const std::vector<Vec4_t, Eigen::aligned_allocator<Eigen::Vector4d>>& points, const std::vector<int>& in_inlier_idx, std::vector<int>& out_inlier_idx) const
{
    // 1. 点群の重心を求める
    Vec4_t sum_vec = Eigen::Vector4d::Zero();
    for(auto idx: in_inlier_idx)
    {
        sum_vec += points.at(idx);
    } 

    const Vec4_t g_points = sum_vec / (double)in_inlier_idx.size();


    // 2. 共分散行列Sを作る
    Mat33_t S = Mat33_t::Zero();
    for(auto idx: in_inlier_idx)
    {
        Vec3_t p_ = (points.at(idx) - g_points).block(0,0,3,1);
        S += p_*p_.transpose();
    }

    // cout << "S\n" << S << endl;


    // 3. Sの最小固有値と対応する固有ベクトルを求める
   Eigen::SelfAdjointEigenSolver<Mat33_t> eigensolver(S);
   if (eigensolver.info() != Eigen::Success)
   {
       spdlog::error("[plane_estimator::estimate_ls()] failed to solve eigenvalues");
       abort();    
   } 

    const Mat33_t eigenvectors = eigensolver.eigenvectors();
    // const Vec3_t eigenvalues = eigensolver.eigenvalues();

    // 4. 固有ベクトル（すなわち、平面の法線）の向きで、点群の重心を通る平面のパラメータを決定
    const Vec3_t normal = eigenvectors.block(0,0,3,1);
    const Vec3_t unit_normal = normal / normal.norm();

    const double h = (unit_normal.transpose() * g_points.block(0,0,3,1))(0);

    Vec4_t best_param_ls;
    best_param_ls(0) = unit_normal(0) / -h;
    best_param_ls(1) = unit_normal(1) / -h;
    best_param_ls(2) = unit_normal(2) / -h;
    best_param_ls(3) = 1;

    // 再度inlierを計算
    cal_inlier(out_inlier_idx, best_param_ls, points);

    // std::cout << "\n[LS]\n";
    // std::cout << "best parameter a: " << best_param_ls.transpose() << std::endl;
    // std::cout << "num points " << points.size() /* << " best score: " << best_score */ << std::endl;
    // std::cout << "num inlier " << out_inlier_idx.size() << std::endl;
    std::cout << std::endl;

    return best_param_ls;
}

Vec4_t plane_estimator::recover_points(const double depth, const double ux, const double uy ) const
{
    Vec4_t xyz;
    xyz[0] = ( ux - cx_ ) * depth/fx_;
    xyz[1] = ( uy - cy_ ) * depth/fy_;
    xyz[2] = depth;
    xyz[3] = 1.0;
    return xyz;
}

void plane_estimator::cal_inlier(std::vector<int>& inlier_idx, const Vec4_t& plane_param, const std::vector<Vec4_t, Eigen::aligned_allocator<Eigen::Vector4d>>& points) const
{    
    inlier_idx.clear();
    for(size_t i=0; i<points.size(); i++)
    {
        double dist = plane_estimator::formula_point_and_plane(points[i], plane_param);
        if(dist < thres_inlier_) inlier_idx.push_back(i);
    }
}

void plane_estimator::cal_principal_component(const std::vector<Vec4_t, Eigen::aligned_allocator<Vec4_t>>& points, const std::vector<int>& inlier_idx,
                                             std::vector<Vec3_t, Eigen::aligned_allocator<Vec3_t>>& principal_vectors) const
{
    // 1. 点群の重心を求める
    Vec4_t sum_vec = Eigen::Vector4d::Zero();
    for(auto idx: inlier_idx)
    {
        sum_vec += points.at(idx);
    } 

    const Vec4_t g_points = sum_vec / (double)inlier_idx.size();

    // 2. 共分散行列Sを作る
    Mat33_t S = Mat33_t::Zero();
    for(auto idx: inlier_idx)
    {
        Vec3_t p_ = (points.at(idx) - g_points).block(0,0,3,1);
        S += p_*p_.transpose();
    }

    // 3. Sの固有値と対応する固有ベクトルを求める
   Eigen::SelfAdjointEigenSolver<Mat33_t> eigensolver(S);
   if (eigensolver.info() != Eigen::Success)
   {
       spdlog::error("[plane_estimator::cal_principal_component()] failed to solve eigenvalues");
       abort();    
   } 

    // 固有値が小さい順に出てくる
    // 固有ベクトルは正規化されて出てくる
    const Mat33_t eigenvectors = eigensolver.eigenvectors();
    // const Vec3_t eigenvalues = eigensolver.eigenvalues();

    principal_vectors.clear();
    for(int i=0; i<3; i++)
    {
        Vec3_t normal = eigenvectors.block(0, i, 3, 1);
        principal_vectors.push_back(normal);

        // std::cout << i+1 << ": eigenvalues: " << eigenvalues(i) << " eigenvector: " << eigenvectors.block(0, i, 3, 1).transpose() << std::endl;
    }
}

Mat44_t plane_estimator::estimate_transform_from_base(const std::vector<Vec3_t, Eigen::aligned_allocator<Vec3_t>>& principal_vectors) const
{
    const Mat33_t I = Mat33_t::Identity();
    Mat33_t correlation_matrix = Mat33_t::Zero();
    int axis = 0;
    int sign = 1;
    for(size_t i=0; i<3; i++)
    {
        const Vec3_t normal_src = principal_vectors.at(i).block(0,0,3,1);
        const double max_val = std::max({abs(normal_src(0)), abs(normal_src(1)), abs(normal_src(2))});
        if(max_val==abs(normal_src(0))) axis = 0;
        if(max_val==abs(normal_src(1))) axis = 1;
        if(max_val==abs(normal_src(2))) axis = 2;

        if(normal_src(axis) < 0) sign = -1;
        else sign = 1;

        const Vec3_t normal_dst = sign * I.block(0,axis,3,1);
        // const Vec3_t normal_dst = I.block(0,axis,3,1);
        correlation_matrix += normal_src * normal_dst.transpose();

        // std::cout << "normal_dst " << normal_dst.transpose() << " \nnormal_src "<< normal_src.transpose() << "\nmul\n" << normal_src * normal_dst.transpose() << std::endl;
    }

    // std::cout << "\ncor mat \n" << correlation_matrix << std::endl;

    Eigen::JacobiSVD< Mat33_t > svd(correlation_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat33_t mat;
    mat << 1, 0, 0,
           0, 1, 0,
           0, 0, (svd.matrixV() * svd.matrixU()).determinant();

    const Mat33_t R_perturb = svd.matrixV() * mat * svd.matrixU().transpose();
    const Vec3_t euler_perturb =  (R_perturb.eulerAngles(2,1,0));      // z->y->x order

    const double rot_y = abs(euler_perturb(1)) > M_PI_2 ? M_PI : 0;
    // const double rot_x = abs(euler_perturb(0)) > M_PI_2 ? M_PI : 0;

    Mat33_t R_perturb_zx;
    R_perturb_zx = Eigen::AngleAxisd(euler_perturb(0), Vec3_t::UnitZ())
                * Eigen::AngleAxisd(rot_y            , Vec3_t::UnitY())
                * Eigen::AngleAxisd(euler_perturb(2), Vec3_t::UnitX());

    const Mat33_t est_rotation = R_rc_base_ * R_perturb_zx; 
    Mat44_t est;
    est.block(0,0,3,3) = est_rotation;
    est.block(0,3,3,1) = transform_rc_default_.block(0,3,3,1); 

    // std::cout << "R_perturb\n" << R_perturb << std::endl;
    // std::cout << "R_perturb_zx\n" << R_perturb_zx << std::endl;
    // std::cout << "rot_y " << rot_y << std::endl;
    // std::cout << "euler_perturb\n" << euler_perturb.transpose() << std::endl;
    // std::cout << "euler_perturb_zx\n" << euler_perturb_zx.transpose() << std::endl;
    // std::cout << "\n\nest_rotation\n" << est_rotation << std::endl;
    // std::cout << "\n\nest_rotation_test\n" << est_rotation_test << std::endl;

    return est;
}



///////////////////////////////////////////////////////////////////
// public

void plane_estimator::set_extrinsic_rc(const Mat44_t& transform_rc)
{
    transform_rc_default_ = transform_rc;
}


double plane_estimator::formula_point_and_plane(const Vec4_t& point, const Vec4_t& params)
{
    double num = 0;
    double deno = 0;

    double temp = 0;
    for (size_t i = 0; i < 4; i++)
    {
        temp += point(i) * params(i);
    }
    deno = abs(temp);

    
    num = sqrt(pow(params(0), 2) + pow(params(1), 2) + pow(params(2), 2));

    assert(num != 0);

    double distance = deno / num;

    return distance;
}

Vec4_t plane_estimator::find_plane_params(const std::vector<Vec4_t, Eigen::aligned_allocator<Eigen::Vector4d>>& points)
{
    assert(points.size() == 3);

    // 平面上にあるベクトルを2つ求める
    // 外積方向が法線方向
    const openvslam::Vec3_t point0 = points.at(0).block(0, 0, 3, 1);
    const openvslam::Vec3_t point1 = points.at(1).block(0, 0, 3, 1);
    const openvslam::Vec3_t point2 = points.at(2).block(0, 0, 3, 1);

    const openvslam::Vec3_t vec01 = point1 - point0;
    const openvslam::Vec3_t vec02 = point2 - point0;

    const openvslam::Vec3_t cross_01_02 = vec01.cross(vec02);

    // ax + by + cz + d = 0
    const double d = -1 * (cross_01_02(0) * point0(0) + cross_01_02(1) * point0(1) + cross_01_02(2) * point0(2));

    // 1に正規化
    openvslam::Vec4_t paramter;
    paramter << cross_01_02(0) / d,
        cross_01_02(1) / d,
        cross_01_02(2) / d,
        1;

    return paramter;
}

void plane_estimator::recover_points_from_depth_img(const cv::Mat& depth_img, std::vector<Vec4_t, Eigen::aligned_allocator<Eigen::Vector4d>>& points, const cv::Mat& mask) const
{
    cv::Mat masked_depth;

    // maskが指定されていない場合、画像の下半分を用いる
    if(mask.empty())
    {
        spdlog::debug("[plane_estimator::recover_points_from_depth_img]  mask is empty");
        cv::Mat mask = cv::Mat::zeros(depth_img.rows, depth_img.cols, CV_8UC1);

        cv::Mat depth_img_8UC1;
        depth_img.convertTo(depth_img_8UC1, CV_8UC1);

        cv::Mat binary_img;
        threshold(depth_img_8UC1, binary_img, 0, 255, cv::THRESH_BINARY_INV+cv::THRESH_OTSU);


        cv::Mat mask_half = cv::Mat::zeros(depth_img.rows, depth_img.cols, CV_8UC1);
        cv::rectangle(mask_half, cv::Rect(0, mask_half.rows / 2, mask_half.cols, mask_half.cols / 2), cv::Scalar(255), -1);
        binary_img.copyTo(mask, mask_half);

        depth_img.copyTo(masked_depth, mask);
    }
    else
    {
        depth_img.copyTo(masked_depth, mask);
    }

    for(int i=0; i<masked_depth.cols; i++)
    {
        for(int j=0; j<masked_depth.rows; j++)
        {
            double depth = masked_depth.at<float> ( j,i );

            if ( depth > cam_dmin_ && depth < cam_dmax_ ) {
                const Eigen::Vector4d cp = recover_points(depth, i, j);
                points.push_back ( cp );
            }
        }
    }
    
}


Mat44_t plane_estimator::estimate(const cv::Mat& depth_img) const
{
    using std::chrono::steady_clock;
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;

    std::vector<Vec4_t, Eigen::aligned_allocator<Eigen::Vector4d>> points;
    std::vector<Vec3_t, Eigen::aligned_allocator<Eigen::Vector3d>> principal_vectors;
    std::vector<int> inlier_ransac;
    std::vector<int> inlier_ls;

    spdlog::debug("[plane_estimator::estimate()] start estimation");

    auto st = steady_clock::now();

    // デプス画像から点を復元
    recover_points_from_depth_img(depth_img, points);
    auto t_depth = steady_clock::now();

    // ransacで平面推定
    estimate_ransac(points, inlier_ransac);
    auto t_ransac = steady_clock::now();

    // inlierの点群から主成分を求める
    cal_principal_component(points, inlier_ransac, principal_vectors);
    auto t_principal = steady_clock::now();

    // 求めた主成分ベクトルから、推定するべき回転を求める
    const Mat44_t est_transform_rc = estimate_transform_from_base(principal_vectors);
    auto t_est = steady_clock::now();

    auto duration_recover  = duration_cast<milliseconds>(t_depth - st).count();
    auto duration_ransac   = duration_cast<milliseconds>(t_ransac - t_depth).count();
    auto duration_principal       = duration_cast<milliseconds>(t_principal - t_ransac).count();
    auto duration_est      = duration_cast<milliseconds>(t_est - t_principal).count();


    spdlog::debug("[plane_estimator::estimate()] duration_recover {}", duration_recover);
    spdlog::debug("[plane_estimator::estimate()] duration_ransac  {}", duration_ransac);
    spdlog::debug("[plane_estimator::estimate()] duration_ls      {}", duration_principal);
    spdlog::debug("[plane_estimator::estimate()] duration_est     {}", duration_est);

    return est_transform_rc;

}

} // namespace module


}