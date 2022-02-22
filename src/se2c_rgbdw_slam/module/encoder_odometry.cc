#include "encoder_odometry.h"
#include "se2c_rgbdw_slam/util/converter.h"

#include <spdlog/spdlog.h>

#include <iomanip>
#include <algorithm>
#include <sstream>
#include <fstream>

#include <math.h>

namespace openvslam
{

Mat33_t encoder_odometry::cov_update_encoder(const Mat33_t& last_cov, const Mat22_t& displacement_cov, const double sr, const double sl, const double th)
{
    const double sep2 = 2.0 * wheel_separation_;
    const double alpha = (sr + sl); 
    const double beta = th + (sr - sl) / (sep2);

    const double sinB = sin(beta);
    const double cosB = cos(beta);


    Eigen::Matrix<double, 3, 5> jacobian;
    jacobian(0, 0) = 1;
    jacobian(0, 1) = 0;
    jacobian(0, 2) = - (alpha * sinB ) / 2.0;
    jacobian(0, 3) = (cosB - (alpha * sinB) / sep2) / 2.0;
    jacobian(0, 4) = (cosB + (alpha * sinB) / sep2) / 2.0;

    jacobian(1, 0) = 0;
    jacobian(1, 1) = 1;
    jacobian(1, 2) = (alpha * cosB) / 2.0;
    jacobian(1, 3) = (sinB + (alpha * cosB) / sep2) / 2.0;
    jacobian(1, 4) = (sinB - (alpha * cosB) / sep2) / 2.0;

    jacobian(2, 0) = 0;
    jacobian(2, 1) = 0;
    jacobian(2, 2) = 1;
    jacobian(2, 3) = 1.0/wheel_separation_;
    jacobian(2, 4) = -1.0/wheel_separation_;


    Eigen::Matrix<double, 5, 5> current_cov;
    current_cov.block(0, 0, 3, 3) = last_cov;
    current_cov.block(3, 0, 2, 3) = Eigen::MatrixXd::Zero(2, 3);
    current_cov.block(3, 3, 2, 2) = displacement_cov;
    current_cov.block(0, 3, 3, 2) = Eigen::MatrixXd::Zero(3, 2);
    Mat33_t updated_cov = jacobian * current_cov * jacobian.transpose();

    // std::cout << "current cov\n " << current_cov << std::endl;
    // std::cout << "jacobian\n " << jacobian << std::endl;
    // std::cout << "updated_cov\n " << updated_cov << std::endl;

    return updated_cov;
}


encoder_odometry::encoder_odometry()
{}

encoder_odometry::encoder_odometry(const double wheel_separation, const double pulse_num_per_m, const double displacement_covariance_factor)
    : wheel_separation_(wheel_separation), pulse_num_per_m_(pulse_num_per_m), displacement_covariance_factor_(displacement_covariance_factor),
      is_first_calculation_(true), last_enc_l_(0), last_enc_r_(0)
{}

encoder_odometry::~encoder_odometry()
{}


void encoder_odometry::reset()
{
    {
        std::lock_guard<std::mutex> lock(mtx_encoders_);
        encoders_.clear();
    }

}

void encoder_odometry::set_encoder_parameter(const double wheel_separation, const double pulse_num_per_m, const double displacement_covariance_factor)
{
    wheel_separation_ = wheel_separation;
    pulse_num_per_m_ = pulse_num_per_m;
    displacement_covariance_factor_ = displacement_covariance_factor;
}

void encoder_odometry::add_measurement(encoder enc)
{
    std::lock_guard<std::mutex> lock(mtx_encoders_);
    encoders_.push_back(enc);
    // spdlog::debug("add encoder measurement {}", enc.v_);

}


size_t encoder_odometry::get_num_encoders()
{
    std::lock_guard<std::mutex> lock(mtx_encoders_);
    return encoders_.size();
}

bool encoder_odometry::dump_odometry_trajectory(const std::string& file_path)
{
    std::ofstream f(file_path);

    if(!f)
    {
        std::cerr << "[encoder_odometry::dump_odometry_trajectory()]: failed to open file: " << file_path << std::endl;
    }
    else
    {
        assert( (x_traj_.size() == y_traj_.size()) && (x_traj_.size() == th_traj_.size()) && (x_traj_.size() == time_traj_.size()) );

        size_t traj_num = x_traj_.size();
        for(size_t i=0; i<traj_num; i++)
        {
            Mat44_t pose = util::converter::SE2paramtoSE3(x_traj_[i], y_traj_[i], th_traj_[i]);
            const Mat33_t& rot_wc = pose.block<3, 3>(0, 0);
            const Vec3_t& trans_wc = pose.block<3, 1>(0, 3);
            const Quat_t quat_wc = Quat_t(rot_wc);
            f << std::setprecision(15)
                << time_traj_[i] << " "
                << std::setprecision(9)
                << trans_wc(0) << " " << trans_wc(1) << " " << trans_wc(2) << " "
                << quat_wc.x() << " " << quat_wc.y() << " " << quat_wc.z() << " " << quat_wc.w() << std::endl;

        }

        f.close();
    }



    return true;
}

void encoder_odometry::get_global_odometry(Mat44_t& global_odom)
{
    global_odom = util::converter::SE2paramtoSE3(x_, y_, th_);
}

void encoder_odometry::integrate_measurement_encoder(const Mat44_t& last_pose, Mat44_t& next_pose, const Mat66_t& last_cov, Mat66_t& next_cov, const double frame_time, int& num_processed_encoder_)
{
    double x, y, th;

    // Mat33_t pose = util::converter::SE3toSE2(last_pose);
    Mat33_t cov = util::converter::SE3covtoSE2cov(last_cov);
    util::converter::SE3toSE2param(last_pose, x, y, th);

    // std::cout << "[encoder_odometry::integrate_measurement_encoder()]  x " << x << "  y " << y << "  th " << th << std::endl;

    {
        std::lock_guard<std::mutex> lock(mtx_encoders_);

        if(encoders_.size() == 0)
        {
            spdlog::info("[encoder_odometry::integrate_measurement_encoder()] no encoder measurement");
        }
        else
        {
            if (is_first_calculation_)
            {
                // std::cout << "encoders_size() " << encoders_.size()  << std::endl;
                // std::cout << "(*encoders_.begin())->wheel_l_  " << (*encoders_.begin()).wheel_l_  << std::endl;
                // std::cout << "(*encoders_.begin())->wheel_r_  " << (*encoders_.begin()).wheel_r_  << std::endl;

                last_enc_l_ = (encoders_.front()).wheel_l_;
                last_enc_r_ = (encoders_.front()).wheel_r_;
                is_first_calculation_ = false;

                x_ = 0;
                y_ = 0;
                th_ = 0;
            }

            constexpr int thres_enc_diff = 1000;
            double dl, dr, ds, dth;
            for (auto enc : encoders_)
            {
                if(enc.timestamp_ >= frame_time) break;
                if((abs(enc.wheel_l_ - last_enc_l_) > thres_enc_diff) || (abs(enc.wheel_r_ - last_enc_r_) > thres_enc_diff))
                {
                    last_enc_l_ = enc.wheel_l_;
                    last_enc_r_ = enc.wheel_r_;
                    continue;
                }

                // pose
                dl = (enc.wheel_l_ - last_enc_l_) / pulse_num_per_m_;
                dr = (enc.wheel_r_ - last_enc_r_) / pulse_num_per_m_;

                ds = ((dl + dr) / 2.0);
                dth = ((dr - dl) / (wheel_separation_));

                x += ds * cos(th + dth / 2.0);
                y += ds * sin(th + dth / 2.0);
                th += dth;
                if(th >= M_PI) th -= 2*M_PI;
                else if(th <= -M_PI) th += 2*M_PI;

                // cov
                const double cov_r = displacement_covariance_factor_ * dr * dr;
                const double cov_l = displacement_covariance_factor_ * dl * dl;
                Mat22_t wheel_cov;
                wheel_cov << cov_r, 0,
                             0, cov_l;
                
                cov = cov_update_encoder(cov, wheel_cov, dr, dl, th);

                x_ += ds * cos(th_ + dth / 2.0);
                y_ += ds * sin(th_ + dth / 2.0);
                th_ += dth;
                if(th_ >= M_PI) th_ -= 2*M_PI;
                else if(th_ <= -M_PI) th_ += 2*M_PI;

                x_traj_.push_back(x_);
                y_traj_.push_back(y_);
                th_traj_.push_back(th_);
                time_traj_.push_back(enc.timestamp_);


                // std::cout << std::setprecision(18);
                // std::cout << "enc timestamp " << enc.timestamp_ << std::endl;
                // std::cout << std::setprecision(10);
                // std::cout << "dx " << dx << "  dy " << dy << "  dth " << dth << std::endl;
                // std::cout << "x " << x << "  y " << y << "  th " << th << std::endl;
                // std::cout << "x_ " << x_ << "  y_ " << y_ << "  th_ " << th_ << std::endl;
                // std::cout << "cov\n" << cov << std::endl;
                // std::cout << "enc diff l" << enc.wheel_l_ - last_enc_l_  << " dist " << dl << std::endl;
                // std::cout << "enc diff r" << enc.wheel_r_ - last_enc_r_  << " dist " << dr << std::endl;            
                // std::cout << "enc l " << enc.wheel_l_  << " last enc_l " <<  last_enc_l_ << " dist " << dl << std::endl;
                // std::cout << "enc r " << enc.wheel_r_  << " last enc_r " <<  last_enc_r_ << " dist " << dr << std::endl;
                // std::cout << std::setprecision(10) <<  "delta_t " << enc.delta_t_ << " v: " << enc.v_ << "  omega: " << enc.omega_ << std::endl;

                // std::cout << "transform " << transform << "\n" << std::endl;
                // std::cout << "pose " << pose << "\n" << std::endl;
                // std::cout << std::endl;


                last_enc_l_ = enc.wheel_l_;
                last_enc_r_ = enc.wheel_r_;
                num_processed_encoder_++;
            }
        }

        // std::cout << "[encoder_odometry::integrate_measurement_encoder()]  x " << x << "  y " << y << "  th " << th << std::endl;
        // std::cout << "cov\n" << cov.diagonal() << std::endl;
        next_pose = util::converter::SE2paramtoSE3(x, y, th);
        next_cov = util::converter::SE2covtoSE3cov(cov, 1e18);
        encoders_.erase(std::remove_if(encoders_.begin(),
                                       encoders_.end(),
                                       [&frame_time](const encoder& enc ) { return enc.timestamp_ < frame_time; }),
                        encoders_.end());
    }

}


// void encoder_odometry::integrate_measurement_velocity(const Mat44_t& last_pose, Mat44_t& next_pose, const Mat66_t& last_cov, Mat66_t& next_cov, const double frame_time, int& num_processed_encoder_)
// {

// }


} // napespace openvslam