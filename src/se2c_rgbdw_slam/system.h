#ifndef OPENVSLAM_SYSTEM_H
#define OPENVSLAM_SYSTEM_H

#include "timer.h"

#include "se2c_rgbdw_slam/type.h"
#include "se2c_rgbdw_slam/data/bow_vocabulary.h"

#include <opencv2/core/core.hpp>

#include <thread>
#include <mutex>
#include <list>
#include <atomic>


namespace openvslam
{

class config;
class tracking_module;
class mapping_module;
class global_optimization_module;

class config_se2_constraint;
class encoder_odometry;
class rgbd_frame_set;
class rgbd_frame;

namespace module
{
class octomap_mapping;
} // namespace module

namespace camera {
class base;
} // namespace camera

namespace data {
class camera_database;
class map_database;
class bow_database;
} // namespace data

namespace publish {
class map_publisher;
class frame_publisher;
} // namespace publish

namespace optimize
{
namespace g2o{
namespace se2c{
class plane_constraint_wrapper;
class odometry_constraint_wrapper;
}
}
}

namespace util
{
class coordinate_transformer;
}

class system
{
private:
    //! Check reset request of the system
    void check_reset_request();

    //! Pause the mapping module and the global optimization module
    void pause_other_threads() const;

    //! Resume the mapping module and the global optimization module
    void resume_other_threads() const;

    //! config
    const std::shared_ptr<config> cfg_;
    //! camera model
    camera::base* camera_ = nullptr;

    //! camera database
    data::camera_database* cam_db_ = nullptr;

    //! map database
    data::map_database* map_db_ = nullptr;

    //! BoW vocabulary
    data::bow_vocabulary* bow_vocab_ = nullptr;

    //! BoW database
    data::bow_database* bow_db_ = nullptr;

    //! tracker
    tracking_module* tracker_ = nullptr;

    //! mapping module
    mapping_module* mapper_ = nullptr;
    //! mapping thread
    std::unique_ptr<std::thread> mapping_thread_ = nullptr;

    //! global optimization module
    global_optimization_module* global_optimizer_ = nullptr;
    //! global optimization thread
    std::unique_ptr<std::thread> global_optimization_thread_ = nullptr;

    //! frame publisher
    std::shared_ptr<publish::frame_publisher> frame_publisher_ = nullptr;
    //! map publisher
    std::shared_ptr<publish::map_publisher> map_publisher_ = nullptr;

    //! system running status flag
    std::atomic<bool> system_is_running_{false};

    //! mutex for reset flag
    mutable std::mutex mtx_reset_;
    //! reset flag
    bool reset_is_requested_ = false;

    //! mutex for terminate flag
    mutable std::mutex mtx_terminate_;
    //! terminate flag
    bool terminate_is_requested_ = false;

    //! mutex for flags of enable/disable mapping module
    mutable std::mutex mtx_mapping_;

    //! mutex for flags of enable/disable loop detector
    mutable std::mutex mtx_loop_detector_;

    mutable std::mutex mtx_track_times_;

    ///////////////////////////////////////////
    //! Proposed
    
    encoder_odometry* encoder_odometry_ = nullptr;
    rgbd_frame_set* rgbd_frame_set_ = nullptr;
    module::octomap_mapping* octomap_mapping_ = nullptr;
    std::shared_ptr<util::coordinate_transformer> coord_transformer_ = nullptr;
    std::shared_ptr<config_se2_constraint> plane_config_ = nullptr;
    std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper> plane_constraint_wrapper_ = nullptr;
    std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper> odometry_constraint_wrapper_ = nullptr;

    void init(const std::string& vocab_file_path);

    std::thread*  main_thread_;

    std::list<Mat44_t> pub_poses_;
    std::list<Mat66_t> pub_covs_;
    std::list<double>  pub_poses_times_;
    std::mutex mtx_pub_poses_;
    std::vector<double> track_times;

    // wheel odometry
    double initial_cov_factor_;
    Mat44_t wheel_odom_from_last_kf_;
    Mat66_t wheel_cov_from_last_kf_;

    //! flag
    bool is_performance_mode_;
    bool use_pulse_encoder_;
    unsigned long cnt_loop_ = 0;

    // counter
    int num_processed_rgbd_ = 0;
    int num_processed_encoder_ = 0;

    // debug
    timer timer_;
    std::string log_dir_;


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    //! Constructor
    system(const std::shared_ptr<config>& cfg, const std::string& vocab_file_path, const std::string& log_dir);

    //! Destructor
    ~system();

    //-----------------------------------------
    // system startup and shutdown

    //! Startup the SLAM system
    void startup(const bool need_initialize = true);

    //! Shutdown the SLAM system
    void shutdown();

    //-----------------------------------------
    // data I/O

    //! Save the frame trajectory in the specified format
    void save_frame_trajectory(const std::string& path, const std::string& format) const;

    //! Save the keyframe trajectory in the specified format
    void save_keyframe_trajectory(const std::string& path, const std::string& format) const;

    //! Load the map database from the MessagePack file
    void load_map_database(const std::string& path) const;

    //! Save the map database to the MessagePack file
    void save_map_database(const std::string& path) const;

    //! Proposed: Save estimated extrinsic parameter
    void save_estimated_param(const std::string& path) const;

    //! Get the map publisher
    const std::shared_ptr<publish::map_publisher> get_map_publisher() const;

    //! Get the frame publisher
    const std::shared_ptr<publish::frame_publisher> get_frame_publisher() const;

    //-----------------------------------------
    // module management

    //! Enable the mapping module
    void enable_mapping_module();

    //! Disable the mapping module
    void disable_mapping_module();

    //! The mapping module is enabled or not
    bool mapping_module_is_enabled() const;

    //! Enable the loop detector
    void enable_loop_detector();

    //! Disable the loop detector
    void disable_loop_detector();

    //! The loop detector is enabled or not
    bool loop_detector_is_enabled() const;

    //! Loop BA is running or not
    bool loop_BA_is_running() const;

    //! Abort the loop BA externally
    void abort_loop_BA();

    //-----------------------------------------
    // management for pause

    //! Pause the tracking module
    void pause_tracker();

    //! The tracking module is paused or not
    bool tracker_is_paused() const;

    //! Resume the tracking module
    void resume_tracker();

    //-----------------------------------------
    // management for reset

    // //! Request to reset the system
    void request_reset();

    // //! Reset of the system is requested or not
    bool reset_is_requested() const;

    //-----------------------------------------
    // management for terminate

    // //! Request to terminate the system
    void request_terminate();

    // //!! Termination of the system is requested or not
    bool terminate_is_requested() const;


    ////////////////////////////////////
    // Proposed

    void set_encoder_param(const double wheel_sep, const double pulse_per_m, const double displacement_covariance_factor);
    void use_pulse_encoder(const bool use_pulse_encoder);
    std::shared_ptr<util::coordinate_transformer> get_coord_transformer() const;
    std::shared_ptr<config_se2_constraint> get_plane_config() const;

    //! add measurement
    void add_encoder_measurement(const double wheel_r, const double wheel_l, const double delta, const double time);
    void add_encoder_measurement(const double v, const double omega, const double delta);
    void add_rgbd_frame(const cv::Mat& rgb_img, const cv::Mat& depth_img, const double time);

    //! get measurement
    rgbd_frame get_next_frames();

    //! mainloop
    void mainloop();

    Mat44_t mainprocess();

    //-----------------------------------------
    // management for publish poses
    void add_publish_poses(const Mat44_t& poses, const Mat66_t& cov, const double time);
    bool get_publish_poses(Mat44_t& poses, Mat66_t& cov, double& time);

    // Octomap
    module::octomap_mapping* get_octomap_mapping_module() const;

};

} // namespace openvslam

#endif