#ifndef OPENVSLAM_TIMER_H
#define OPENVSLAM_TIMER_H

#include <iostream>
#include <chrono>
#include <vector>
#include <string>

namespace openvslam
{
class timer
{
private:
    std::chrono::system_clock::time_point start_time_;
    std::vector<std::chrono::system_clock::time_point> lap_;
    std::chrono::system_clock::time_point fin_time_;

    bool mode_;

public:
    timer(const bool mode = true)
        : mode_(mode)
    {
    }

    void start()
    {
        lap_.clear();
        start_time_ = std::chrono::system_clock::now();
        lap_.push_back(start_time_);
    };

    double lap()
    {
        auto now = std::chrono::system_clock::now();
        lap_.push_back(now);

        double ms = std::chrono::duration_cast<std::chrono::milliseconds>(*(lap_.end() - 1) - *(lap_.end() - 2)).count();

        return ms;
    }

    void stop()
    {
        fin_time_ = std::chrono::system_clock::now();
    };

    double get_time()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(fin_time_ - start_time_).count();
    }

    void print(const std::string &func, const std::string message)
    {
        if (mode_)
        {
            std::cout << "[ " << func << " ]:  " << message << std::endl;
        }
    }

    void set_mode(const bool mode)
    {
            mode_ = mode;
        }
    };

}

#endif