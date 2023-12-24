#include "se2c_rgbdw_slam/data/rgbd_frame_set.h"
#include "se2c_rgbdw_slam/module/encoder_odometry.h"
#include "se2c_rgbdw_slam/config_se2_constraint.h"
#include "se2c_rgbdw_slam/util/coordinate_transformer.h"

#include "se2c_rgbdw_slam/system.h"
#include "se2c_rgbdw_slam/tracking_module.h"
#include "se2c_rgbdw_slam/mapping_module.h"
#include "se2c_rgbdw_slam/global_optimization_module.h"


#include "se2c_rgbdw_slam/config.h"
#include "se2c_rgbdw_slam/camera/base.h"
#include "se2c_rgbdw_slam/data/camera_database.h"
#include "se2c_rgbdw_slam/data/map_database.h"
#include "se2c_rgbdw_slam/data/bow_database.h"
#include "se2c_rgbdw_slam/io/trajectory_io.h"
#include "se2c_rgbdw_slam/io/map_database_io.h"
#include "se2c_rgbdw_slam/publish/map_publisher.h"
#include "se2c_rgbdw_slam/publish/frame_publisher.h"

#include <spdlog/spdlog.h>

namespace openvslam
{

using namespace std;

system::system(const std::shared_ptr<config>& cfg, const std::string& vocab_file_path, const std::string& log_dir)
    : cfg_(cfg), camera_(cfg->camera_),
      initial_cov_factor_(cfg_->yaml_node_["Encoder.minimum_covariance"].as<double>(1e-6)),
      wheel_odom_from_last_kf_(Mat44_t::Identity()),
      wheel_cov_from_last_kf_(Mat66_t::Identity() * initial_cov_factor_),
      is_performance_mode_(cfg_->yaml_node_["Method.performance_mode"].as<bool>(false)),
      log_dir_(log_dir)
{
    spdlog::debug("CONSTRUCT: system");
    
    init(vocab_file_path);

    // show configuration
    std::cout << *cfg_ << std::endl;
    std::cout << *plane_config_ << std::endl;
}

void system::init(const std::string& vocab_file_path)
{
    // load ORB vocabulary
    spdlog::info("loading ORB vocabulary: {}", vocab_file_path);
#ifdef USE_DBOW2
    bow_vocab_ = new data::bow_vocabulary();
    try {
        bow_vocab_->loadFromBinaryFile(vocab_file_path);
    }
    catch (const std::exception& e) {
        spdlog::critical("wrong path to vocabulary");
        delete bow_vocab_;
        bow_vocab_ = nullptr;
        exit(EXIT_FAILURE);
    }
#else
    bow_vocab_ = new fbow::Vocabulary();
    bow_vocab_->readFromFile(vocab_file_path);
    if (!bow_vocab_->isValid()) {
        spdlog::critical("wrong path to vocabulary");
        delete bow_vocab_;
        bow_vocab_ = nullptr;
        exit(EXIT_FAILURE);
    }
#endif

    // database
    cam_db_ = new data::camera_database(camera_);
    map_db_ = new data::map_database();
    bow_db_ = new data::bow_database(bow_vocab_);

    // frame and map publisher
    frame_publisher_ = std::shared_ptr<publish::frame_publisher>(new publish::frame_publisher(cfg_, map_db_));
    map_publisher_ = std::shared_ptr<publish::map_publisher>(new publish::map_publisher(cfg_, map_db_));

    // tracking module
    tracker_ = new tracking_module(cfg_, this, map_db_, bow_vocab_, bow_db_);
    // // mapping module
    mapper_ = new mapping_module(map_db_, camera_->setup_type_ == camera::setup_type_t::Monocular);
    // // global optimization module
    global_optimizer_ = new global_optimization_module(map_db_, bow_db_, bow_vocab_, camera_->setup_type_ != camera::setup_type_t::Monocular);

    // connect modules each other
    tracker_->set_mapping_module(mapper_);
    tracker_->set_global_optimization_module(global_optimizer_);
    mapper_->set_tracking_module(tracker_);
    mapper_->set_global_optimization_module(global_optimizer_);
    global_optimizer_->set_tracking_module(tracker_);
    global_optimizer_->set_mapping_module(mapper_);

    //////////////! Proposed

    encoder_odometry_ = new encoder_odometry();
    rgbd_frame_set_ = new rgbd_frame_set();

    // octomap mapping module
    octomap_mapping_ = new module::octomap_mapping(cfg_, map_db_);
    mapper_->set_octomap_mappoing_module(octomap_mapping_);
    global_optimizer_->set_octomap_mapping_module(octomap_mapping_);

    coord_transformer_ = std::shared_ptr<util::coordinate_transformer>(new openvslam::util::coordinate_transformer(cfg_));
    plane_config_ = std::shared_ptr<config_se2_constraint>(new openvslam::config_se2_constraint(cfg_));
    plane_constraint_wrapper_ = std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper>(new optimize::g2o::se2c::plane_constraint_wrapper(coord_transformer_, plane_config_));
    odometry_constraint_wrapper_ = std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper>(new optimize::g2o::se2c::odometry_constraint_wrapper(coord_transformer_, plane_config_));

    coord_transformer_->set_plane_wrapper(plane_constraint_wrapper_);
    coord_transformer_->set_odometry_wrapper(odometry_constraint_wrapper_);

    tracker_->set_coord_transformer(coord_transformer_);
    octomap_mapping_->set_coord_transformer(coord_transformer_);

    tracker_->set_se2c_wrappers(plane_constraint_wrapper_, odometry_constraint_wrapper_);
    mapper_->set_se2c_wrappers(plane_constraint_wrapper_, odometry_constraint_wrapper_);
    global_optimizer_->set_se2c_wrappers(plane_constraint_wrapper_, odometry_constraint_wrapper_);
}

system::~system()
{
    delete main_thread_;
    main_thread_ = nullptr;

    global_optimization_thread_.reset(nullptr);
    delete global_optimizer_;
    global_optimizer_ = nullptr;

    mapping_thread_.reset(nullptr);
    delete mapper_;
    mapper_ = nullptr;

    delete tracker_;
    tracker_ = nullptr;

    delete bow_db_;
    bow_db_ = nullptr;
    delete map_db_;
    map_db_ = nullptr;
    delete cam_db_;
    cam_db_ = nullptr;
    delete bow_vocab_;
    bow_vocab_ = nullptr;

    delete encoder_odometry_;
    encoder_odometry_ = nullptr;
    delete rgbd_frame_set_;
    rgbd_frame_set_ = nullptr;
    delete octomap_mapping_;
    octomap_mapping_ = nullptr;

    spdlog::debug("DESTRUCT: system");
}

///////////////////////////////////////////////////////////////////////////////////////////
//  system startup and shutdown

void system::startup(const bool need_initialize)
{
    spdlog::info("startup SLAM system");
    system_is_running_ = true;

    if (!need_initialize)
    {
        tracker_->tracking_state_ = tracker_state_t::Lost;
    }

    mapping_thread_ = std::unique_ptr<std::thread>(new std::thread(&openvslam::mapping_module::run, mapper_));
    global_optimization_thread_ = std::unique_ptr<std::thread>(new std::thread(&openvslam::global_optimization_module::run, global_optimizer_));

    main_thread_ = new std::thread(&system::mainloop, this);
}

void system::shutdown()
{
    main_thread_->join();


    //////////////////////////////////////////

    // terminate the other threads
    mapper_->request_terminate();
    global_optimizer_->request_terminate();

    // wait until they stop
    while (!mapper_->is_terminated() || !global_optimizer_->is_terminated() || global_optimizer_->loop_BA_is_running())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    // wait until the threads stop
    mapping_thread_->join();
    global_optimization_thread_->join();


    ///////////////////////////////////////////////
    {
        std::lock_guard<std::mutex> lock(mtx_track_times_);

        // output the trajectories for evaluation
        save_frame_trajectory(log_dir_ + "frame_trajectory.txt", "TUM");
        save_keyframe_trajectory(log_dir_ + "keyframe_trajectory.txt", "TUM");
        encoder_odometry_->dump_odometry_trajectory(log_dir_ + "encoder_trajectory.txt");
        
        // output the tracking times for evaluation
        std::ofstream ofs(log_dir_ + "track_times.txt", std::ios::out);
        if (ofs.is_open())
        {
            for (const auto track_time : track_times)
            {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
        // output the map database
        std::string map_db_path(log_dir_ + "map.db");
        save_map_database(map_db_path);

        // save octomap 
        octomap_mapping_->save_octree(log_dir_ + "octomap.bt");

        if (track_times.size())
        {
            std::sort(track_times.begin(), track_times.end());
            const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
            std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[ms]" << std::endl;
            std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[ms]" << std::endl;
        }
    }

    if(cfg_->yaml_node_["Initializer.enable_plane_estimate"].as<bool>(false))
    {
        save_estimated_param(log_dir_+"estimated_param.txt");
    }


    spdlog::info("num_processed_rgbd_    {}", num_processed_rgbd_);
    spdlog::info("num_processed_encoder_ {}", num_processed_encoder_);

    system_is_running_ = false;
}

///////////////////////////////////////////////////////////////////////////////////////////
// management for reset

void system::request_reset()
{
    std::lock_guard<std::mutex> lock(mtx_reset_);
    reset_is_requested_ = true;
}

bool system::reset_is_requested() const
{
    std::lock_guard<std::mutex> lock(mtx_reset_);
    return reset_is_requested_;
}

///////////////////////////////////////////////////////////////////////////////////////////
// management for terminate

void system::request_terminate()
{
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool system::terminate_is_requested() const
{
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

///////////////////////////////////////////////////////////////////////////////////////////
// other private methods

void system::check_reset_request()
{
    std::lock_guard<std::mutex> lock(mtx_reset_);
    if (reset_is_requested_)
    {
        tracker_->reset();
        reset_is_requested_ = false;
    }
}

void system::pause_other_threads() const
{
    // pause the mapping module
    if (mapper_ && !mapper_->is_terminated())
    {
        mapper_->request_pause();
        while (!mapper_->is_paused() && !mapper_->is_terminated())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    }
    // pause the global optimization module
    if (global_optimizer_ && !global_optimizer_->is_terminated())
    {
        global_optimizer_->request_pause();
        while (!global_optimizer_->is_paused() && !global_optimizer_->is_terminated())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    }
}

void system::resume_other_threads() const
{
    // resume the global optimization module
    if (global_optimizer_)
    {
        global_optimizer_->resume();
    }
    // resume the mapping module
    if (mapper_)
    {
        mapper_->resume();
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// data I/O

void system::save_frame_trajectory(const std::string &path, const std::string &format) const
{
    spdlog::debug("system::save_frame_trajectory()");
    pause_other_threads();
    io::trajectory_io trajectory_io(map_db_);

    trajectory_io.set_coord_transformer(coord_transformer_);
    trajectory_io.save_frame_trajectory(path, format);
    resume_other_threads();
}

void system::save_keyframe_trajectory(const std::string &path, const std::string &format) const
{
    pause_other_threads();
    io::trajectory_io trajectory_io(map_db_);

    trajectory_io.set_coord_transformer(coord_transformer_);
    trajectory_io.save_keyframe_trajectory(path, format);
    resume_other_threads();
}

void system::load_map_database(const std::string &path) const
{
    pause_other_threads();
    io::map_database_io map_db_io(cam_db_, map_db_, bow_db_, bow_vocab_);
    map_db_io.load_message_pack(path);
    resume_other_threads();
}

void system::save_map_database(const std::string &path) const
{
    pause_other_threads();
    io::map_database_io map_db_io(cam_db_, map_db_, bow_db_, bow_vocab_);
    map_db_io.save_message_pack(path);
    resume_other_threads();
}

void system::save_estimated_param(const std::string& path) const
{
    std::ofstream f(path);
    const Mat44_t param = tracker_->estimated_transform_irc_;

    f << std::setprecision(8) << std::scientific <<  std::fixed;
    f << param(0,0) << ", " << param(0,1) << ", " << param(0,2) << ", " << param(0,3) << "\n";
    f << param(1,0) << ", " << param(1,1) << ", " << param(1,2) << ", " << param(1,3) << "\n";
    f << param(2,0) << ", " << param(2,1) << ", " << param(2,2) << ", " << param(2,3) << "\n";
    f << param(3,0) << ", " << param(3,1) << ", " << param(3,2) << ", " << param(3,3) << "\n";
    f << "\n\n";

    const double RAD2DEG = 180.0 / M_PI;
    const Mat33_t rotation = param.block(0,0,3,3);
    const Vec3_t eular_angles = rotation.eulerAngles(2,1,0);
    f << "eular angles: z: " <<  RAD2DEG * eular_angles(2) 
                   << " y: " << RAD2DEG *eular_angles(1) 
                   << " x: " << RAD2DEG *eular_angles(0)
                   << std::endl;

    f.close();
}

const std::shared_ptr<publish::map_publisher> system::get_map_publisher() const
{
    return map_publisher_;
}

const std::shared_ptr<publish::frame_publisher> system::get_frame_publisher() const
{
    return frame_publisher_;
}

module::octomap_mapping* system::get_octomap_mapping_module() const
{
    return octomap_mapping_;
}

std::shared_ptr<util::coordinate_transformer> system::get_coord_transformer() const
{
    return coord_transformer_;
}

std::shared_ptr<config_se2_constraint> system::get_plane_config() const
{
    return plane_config_;
}

///////////////////////////////////////////////////////////////////////////////////////////
// module management

void system::enable_mapping_module()
{
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    if (!system_is_running_)
    {
        spdlog::critical("please call system::enable_mapping_module() after system::startup()");
    }
    // resume the mapping module
    mapper_->resume();
    // inform to the tracking module
    tracker_->set_mapping_module_status(true);
}

void system::disable_mapping_module()
{
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    if (!system_is_running_)
    {
        spdlog::critical("please call system::disable_mapping_module() after system::startup()");
    }
    // pause the mapping module
    mapper_->request_pause();
    // wait until it stops
    while (!mapper_->is_paused())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
    // inform to the tracking module
    tracker_->set_mapping_module_status(false);
}

bool system::mapping_module_is_enabled() const
{
    return !mapper_->is_paused();
}

void system::enable_loop_detector()
{
    std::lock_guard<std::mutex> lock(mtx_loop_detector_);
    global_optimizer_->enable_loop_detector();
}

void system::disable_loop_detector()
{
    std::lock_guard<std::mutex> lock(mtx_loop_detector_);
    global_optimizer_->disable_loop_detector();
}

bool system::loop_detector_is_enabled() const
{
    return global_optimizer_->loop_detector_is_enabled();
}

bool system::loop_BA_is_running() const
{
    return global_optimizer_->loop_BA_is_running();
}

void system::abort_loop_BA()
{
    global_optimizer_->abort_loop_BA();
}

///////////////////////////////////////////////////////////////////////////////////////////
// management for pause

void system::pause_tracker()
{
    tracker_->request_pause();
}

bool system::tracker_is_paused() const
{
    return tracker_->is_paused();
}

void system::resume_tracker()
{
    tracker_->resume();
}

///////////////////////////////////////////////////////////////////////////////////////////
// data input

void system::add_encoder_measurement(const double wheel_r, const double wheel_l, const double delta, const double time)
{
    encoder enc(wheel_r, wheel_l, delta, time);
    encoder_odometry_->add_measurement(enc);
}

void system::add_encoder_measurement(const double v, const double omega, const double delta)
{
    encoder enc(v, omega, delta);
    encoder_odometry_->add_measurement(enc);
}

void system::add_rgbd_frame(const cv::Mat &rgb_img, const cv::Mat &depth_img, const double time)
{
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    rgbd_frame_set_->add_frames(&rgb_img, &depth_img, time);
}

void system::add_publish_poses(const Mat44_t &poses, const Mat66_t &cov, const double time)
{
    std::lock_guard<std::mutex> lock(mtx_pub_poses_);
    pub_poses_.push_back(poses);
    pub_covs_.push_back(cov);
    pub_poses_times_.push_back(time);
}

void system::set_encoder_param(const double wheel_sep, const double pulse_per_m, const double displacement_covariance_factor)
{
    encoder_odometry_->set_encoder_parameter(wheel_sep, pulse_per_m, displacement_covariance_factor);
}

void system::use_pulse_encoder(const bool use_pulse_encoder)
{
    use_pulse_encoder_ = use_pulse_encoder;
}

///////////////////////////////////////////////////////////////////////////////////////////
// data output

bool system::get_publish_poses(Mat44_t &poses, Mat66_t &cov, double& time)
{
    std::lock_guard<std::mutex> lock(mtx_pub_poses_);

    if (pub_poses_.size() != 0 && pub_covs_.size() != 0 && pub_poses_times_.size() != 0)
    {
        openvslam::util::converter::substitute_mat44(pub_poses_.begin()->eval(), poses);
        openvslam::util::converter::substitute_mat66(pub_covs_.begin()->eval(), cov);
        time = *(pub_poses_times_.begin());

        pub_poses_.erase(pub_poses_.begin());
        pub_covs_.erase(pub_covs_.begin());
        pub_poses_times_.erase(pub_poses_times_.begin());

        return true;
    }

    return false;
}

rgbd_frame system::get_next_frames()
{
    rgbd_frame rgbd_frame;
    rgbd_frame_set_->get_next_frames(rgbd_frame);
    num_processed_rgbd_++;
    return rgbd_frame;
}

///////////////////////////////////////////////////////////////////////////////////////////
// main loop

void system::mainloop()
{
    spdlog::info("mainloop start \n");


    while (!terminate_is_requested())
    {
        check_reset_request();
    
        if(rgbd_frame_set_->get_num_frames() != 0 && encoder_odometry_->get_num_encoders() != 0)
        {
            spdlog::debug("[ system::mainloop() ] rgbd_frame_set_->get_num_frames(): {}", rgbd_frame_set_->get_num_frames());
            spdlog::debug("[ system::mainloop() ] encoder_odometry_->get_num_encoders(): {}", encoder_odometry_->get_num_encoders());

            mainprocess();
        }
        else
        {   
            // 5ms
            // spdlog::debug("waiting....");
            usleep(5000);
        }
    }

    spdlog::debug("finish mainloop");
}

Mat44_t system::mainprocess()
{
    bool is_keyframe = false;
    std::string state;
    double ms;

    timer_.start();

 
    // RGBD画像を取ってくる
    const auto rgbd_frame = get_next_frames();

// Encoder Integration
    if(use_pulse_encoder_)
    {
        encoder_odometry_->integrate_measurement_encoder(wheel_odom_from_last_kf_, wheel_odom_from_last_kf_, wheel_cov_from_last_kf_, wheel_cov_from_last_kf_, rgbd_frame.timestamp_, num_processed_encoder_);
    }
    else
    {
        // TODO
        // encoder_odometry_->integrate_measurement_velocity(wheel_odom_from_last_kf_, wheel_odom_from_last_kf_, wheel_cov_from_last_kf_, wheel_cov_from_last_kf_, rgbd_frame.timestamp_, num_processed_encoder_);

        spdlog::error("[system::mainprocess()] not supported for wheel odometry for linear and angular velocity.");
        std::abort();
    }

    // std::cout << "wheel_odom_from_last_kf_\n" << wheel_odom_from_last_kf_ << std::endl;
    // std::cout << "wheel_cov_from_last_kf_\n" << wheel_cov_from_last_kf_ << std::endl;
    // std::cout << std::endl;


    const auto t_enc = timer_.lap();

    const Mat44_t cam_pose_cir = tracker_->visual_encoder_track(rgbd_frame.rgb_frame_, rgbd_frame.depth_frame_, rgbd_frame.timestamp_, is_keyframe, wheel_odom_from_last_kf_, wheel_cov_from_last_kf_);   
    Eigen::PartialPivLU<Mat44_t> lu(coord_transformer_->get_rc() * cam_pose_cir);
    const Mat44_t pose_irr_r = lu.inverse();

    const auto t_track = timer_.lap();

    if(is_keyframe)
    {
        // cout << "wheel_odom_from_last_kf_ \n" << wheel_odom_from_last_kf_ << std::endl;

        wheel_odom_from_last_kf_ = Mat44_t::Identity();
        wheel_cov_from_last_kf_ = Mat66_t::Identity() * initial_cov_factor_;
    }

    // cout << "pose_irr_r \n" << pose_irr_r << std::endl;

    if(!is_performance_mode_)
    {
        // 最終的なposeをpublishする
        add_publish_poses(pose_irr_r, wheel_cov_from_last_kf_, rgbd_frame.timestamp_);
        // Mat44_t global_wheel_odom;
        // encoder_odometry_->get_global_odometry(global_wheel_odom);
        // add_publish_poses(global_wheel_odom, wheel_cov_from_last_kf_, rgbd_frame.timestamp_);

        frame_publisher_->update(tracker_);
        if (tracker_->tracking_state_ == tracker_state_t::Tracking) {
            map_publisher_->set_current_cam_pose(cam_pose_cir);
        }
    }

    if(tracker_->tracking_state_ == tracker_state_t::Tracking) state = "Tracking...";
    if(tracker_->tracking_state_ == tracker_state_t::Lost) state = "Lost...";


    timer_.stop();
    ms = timer_.get_time();
    spdlog::info("[ system::mainloop() ] this loop time: {}[ms] ((enc {}, track {}) STATE: {}", ms, t_enc, t_track, state.c_str());
    {
        std::lock_guard<std::mutex> lock(mtx_track_times_);
        track_times.push_back(ms);
    }

    cnt_loop_++;

    return pose_irr_r;
}

} // namespace openvslam