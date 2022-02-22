#include "slam_node.h"
#include "dataset_publisher.h"

#include <spdlog/spdlog.h>
#include <popl.hpp>

using namespace std;

void split(const std::string& str, const char sep, std::vector<std::string>& substr_vec) 
{
    int idx = 0;
    int last = str.find_first_of(sep);
  
    while (idx < str.size()) {
        std::string subStr(str, idx, last - idx);
 
        substr_vec.push_back(subStr);
 
        idx = last + 1;
        last = str.find_first_of(sep, idx);
 
        if (last == std::string::npos) {
            last = str.size();
        }
    }
}

void read_encoder_file(const string& encoder_path, vector<double>& timestamp, vector<double>& enc_r, vector<double>& enc_l)
{

    ifstream f(encoder_path);
    string line;
    const char sep = ',';
    if(f.fail())
    {
        cerr << "file path doesn't exist..." << encoder_path;
        return;
    }


    while (getline(f, line)) {
        // std::cout << "#" << str << std::endl;
        std::vector<std::string> sub_str;
        split(line, sep, sub_str);

        timestamp.push_back(stod(sub_str[0].c_str()));
        enc_l.push_back(stod(sub_str[1].c_str()));
        enc_r.push_back(stod(sub_str[2].c_str()));   

        sub_str.clear();     
    }
    
    // cout << setprecision(15);
    // for(int i=0; i<100; i++)
    // {
    //     cout << timestamp[i] << " " << enc_r[i] << " " << enc_l[i] << endl;
    // }

}


int main(int argc, char* argv[])
{
    ////////////////////////////////////////////////////////////////////////////////////////
    // loading configuration

    ros::init(argc, argv, "slam_from_dump_node");

    // create options
    popl::OptionParser op("Allowed options");
    auto help               = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path    = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto setting_file_path  = op.add<popl::Value<std::string>>("c", "config", "setting file path");
    auto rgb_dir            = op.add<popl::Value<std::string>>("", "rgb-dir", "color image directory path");
    auto depth_dir          = op.add<popl::Value<std::string>>("", "depth-dir", "depth image directory path");
    auto encoder_path       = op.add<popl::Value<std::string>>("", "enc-path", "encoder csv path");
    auto mask_img_path      = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");

    auto debug_mode         = op.add<popl::Switch>("", "debug", "debug mode");
    auto eval_log           = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    
    auto map_db_path        = op.add<popl::Value<std::string>>("", "map-db", "store a map database at this path after SLAM", "");
    auto log_dir_path       = op.add<popl::Value<std::string>>("l", "log-dir", "directory to save log", "");

    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !setting_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(setting_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    const bool performance_mode = cfg->yaml_node_["Method.performance_mode"].as<bool>(true);

    ////////////////////////////////////////////////////////////////////////////////////////
    // reading encoder and rgbd data

    const string rgb_dir_str = rgb_dir->value();
    const string depth_dir_str = depth_dir->value();
    const string encoder_data_path = encoder_path->value();

    openvslam_ros::dataset_publisher dataset_color(rgb_dir_str);
    openvslam_ros::dataset_publisher dataset_depth(depth_dir_str);
    vector<double> enc_timestamps;
    vector<double> enc_r;
    vector<double> enc_l;
    read_encoder_file(encoder_data_path, enc_timestamps, enc_r, enc_l);

    // for(auto enc_timestamp : enc_timestamps) std::cout << enc_timestamp << std::endl;
    // for(auto r : enc_r) std::cout << r << std::endl; 
    // for(auto l : enc_l) std::cout << l << std::endl; 

    ////////////////////////////////////////////////////////////////////////////////////////
    // launch slam node


    openvslam_ros::slam_node* node = new openvslam_ros::slam_node( cfg, 
                                                                       vocab_file_path->value(), 
                                                                       log_dir_path->value(), 
                                                                       "", 
                                                                       "" 
                                                                       );

    // system.mainloop()を使わないため
    node->request_terminate_system();

    ////////////////////////////////////////////////////////////////////////////////////////
    // prepare variables for main process

    auto it_timestamp_enc = enc_timestamps.begin();
    auto it_enc_r = enc_r.begin();
    auto it_enc_l = enc_l.begin();
    double enc_delta_t = 0;
    double enc_last_time = 0;

    constexpr double time_difference_thres = 0.065;          // 1 / 15 Hz / 2

    ////////////////////////////////////////////////////////////////////////////////////////
    // main process

    for(auto color_it=dataset_color.paths.begin(), depth_it=dataset_depth.paths.begin();;)
    {
        auto st = std::chrono::steady_clock::now();

        if(color_it==dataset_color.paths.end() || depth_it==dataset_depth.paths.end()) break;
        if(!ros::ok()) break;
        
        if(!performance_mode)
        {
            spdlog::debug(*color_it);
            spdlog::debug(*depth_it);
        }

        if(openvslam_ros::dataset_publisher::basename(*color_it) != openvslam_ros::dataset_publisher::basename(*depth_it))
        {
            const double time_color = std::stod(openvslam_ros::dataset_publisher::basename(*color_it));
            const double time_depth = std::stod(openvslam_ros::dataset_publisher::basename(*depth_it));
            
            if(abs(time_color - time_depth) >= time_difference_thres)
            {
                time_color > time_depth ? depth_it++ : color_it++;
                continue;
            }
        }

        const double timestamp_frame = std::stod(openvslam_ros::dataset_publisher::basename(*color_it));
        
        // read image
        cv::Mat rgb_img = cv::imread(*color_it, cv::IMREAD_UNCHANGED);
        cv::Mat depth_16UC1_img = cv::imread(*depth_it, cv::IMREAD_UNCHANGED);
        

        if(timestamp_frame > *it_timestamp_enc)
        {
            // add encoder measurement
            while(timestamp_frame > *it_timestamp_enc)
            {
                enc_delta_t = *it_timestamp_enc - enc_last_time;
                node->add_encoder_measurement(*it_enc_r, *it_enc_l, enc_delta_t, *it_timestamp_enc);
                enc_last_time = *it_timestamp_enc;

                // cout << std::setprecision(18) << "frame time " << timestamp_frame << " enc time " << *it_timestamp_enc << std::endl; 

                it_timestamp_enc++;
                it_enc_l++;
                it_enc_r++;
            }

            // add rgbd image
            node->add_rgbd_frame(rgb_img, depth_16UC1_img, timestamp_frame);

            // process
            openvslam::Mat44_t pose_irr_r = node->mainprocess();
            
            if(!performance_mode)
            {
                node->publish_ros();
                auto end = std::chrono::steady_clock::now();

                spdlog::debug("[slam_from_dump_node.cc main()] this loop: {} [ms]\n", std::chrono::duration_cast<std::chrono::milliseconds>(end - st).count());
            } 
        }

        color_it++;
        depth_it++;
    }

    node->request_shutdown_system();

    delete node;

    return EXIT_SUCCESS;
}