#ifndef OPENVSLAM_DATA_RGBD_FRAME_SET_H
#define OPENVSLAM_DATA_RGBD_FRAME_SET_H

#include "rgbd_frame.h"
#include "se2c_rgbdw_slam/type.h"
#include "se2c_rgbdw_slam/config.h"


#include <list>
#include <mutex>

namespace openvslam
{

class rgbd_frame_set
{
private:
    std::list<rgbd_frame> frames_;
    std::shared_ptr<config> cfg_ptr_;

    std::mutex mtx_frames_;


public:
    rgbd_frame_set(){};
    ~rgbd_frame_set(){};

    size_t get_num_frames(){return frames_.size();};
    void get_next_frames(rgbd_frame& frames);

    void add_frames(const cv::Mat* rgbd_img, const cv::Mat* depth_img, const double time);

};

}

#endif