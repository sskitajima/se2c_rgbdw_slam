#ifndef OPENVSLAM_DATA_RGBD_FRAME_H
#define OPENVSLAM_DATA_RGBD_FRAME_H

#include <opencv2/core/core.hpp>

namespace openvslam
{
struct rgbd_frame
{
    double timestamp_;
    cv::Mat rgb_frame_;
    cv::Mat depth_frame_;
};

}
#endif
