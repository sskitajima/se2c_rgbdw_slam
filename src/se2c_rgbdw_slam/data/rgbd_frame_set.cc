#include "rgbd_frame_set.h"

#include <iostream>

namespace openvslam 
{

void rgbd_frame_set::add_frames(const cv::Mat* rgbd_img, const cv::Mat* depth_img, const double time) 
{
    rgbd_frame frames;
    frames.rgb_frame_ = rgbd_img->clone();
    frames.depth_frame_ = depth_img->clone();
    frames.timestamp_ = time;
    
    {
        std::lock_guard<std::mutex> lock(mtx_frames_);
        frames_.push_back(frames);
    }
}

void rgbd_frame_set::get_next_frames(rgbd_frame& frameset) 
{
    if(frames_.size() != 0)
    {
        std::lock_guard<std::mutex> lock(mtx_frames_);
        auto it = frames_.begin();

        frameset.timestamp_ = it->timestamp_;
        frameset.rgb_frame_ = it->rgb_frame_.clone();
        frameset.depth_frame_ = it->depth_frame_.clone();

        // フレームレートより処理速度のほうが遅いので、遅延しないよう削除する
        // frames_.erase(it);
        frames_.clear();
    }

}

} // namespace openvslam