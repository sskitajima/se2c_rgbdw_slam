#ifndef OPENVSLAM_MODULE_ENCODER_ODOMETRY_H
#define OPENVSLAM_MODULE_ENCODER_ODOMETRY_H

#include "se2c_rgbdw_slam/data/encoder.h"
#include "se2c_rgbdw_slam/type.h"
#include "se2c_rgbdw_slam/config.h"

#include <mutex>
#include <list>

namespace openvslam
{

class encoder_odometry
{
private:
    std::list<encoder> encoders_;
    std::shared_ptr<config> cfg_ptr_;

    mutable std::mutex mtx_encoders_;

    // encoder parameters
    double wheel_separation_;
    double pulse_num_per_m_;
    double displacement_covariance_factor_;

    // variable for calculation
    bool is_first_calculation_ = true;
    double last_enc_l_;
    double last_enc_r_;

    // global position
    double x_;
    double y_;
    double th_;

    std::vector<double> x_traj_;
    std::vector<double> y_traj_;
    std::vector<double> th_traj_;
    std::vector<double> time_traj_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    encoder_odometry();
    encoder_odometry(const double wheel_separation, const double pulse_num_per_m, const double displacement_covariance_factor);
    ~encoder_odometry();

    void set_encoder_parameter(const double wheel_separation, const double pulse_num_per_m, const double displacement_covariance_factor);
    void add_measurement(encoder enc);

    size_t get_num_encoders();
    void get_global_odometry(Mat44_t& global_odom);

    // encoderのカウンタ値を用いるもの
    void integrate_measurement_encoder(const Mat44_t& last_pose, Mat44_t& next_pose, const Mat66_t& last_cov, Mat66_t& next_cov, const double frame_time, int& num_processed_encoder_);

    // 並進速度と角速度を用いるもの
    // void integrate_measurement_velocity(const Mat44_t& last_pose, Mat44_t& next_pose, const Mat66_t& last_cov, Mat66_t& next_cov, const double frame_time, int& num_processed_encoder_);


    void motion_model();
    Mat33_t cov_update_encoder(const Mat33_t& last_cov, const Mat22_t& displacement_cov, const double sr, const double sl, const double th);


    void reset();
    bool dump_odometry_trajectory(const std::string& file_path);

};

}

#endif