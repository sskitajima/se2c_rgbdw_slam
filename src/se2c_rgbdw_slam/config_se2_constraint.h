#ifndef OPENVSLAM_CONFIG_SE2_CONSTRAINT_H
#define OPENVSLAM_CONFIG_SE2_CONSTRAINT_H

#include "se2c_rgbdw_slam/config_se2_constraint.h"
#include "se2c_rgbdw_slam/type.h"
#include "se2c_rgbdw_slam/config.h"

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace openvslam {

class config_se2_constraint
{
private:
    ros::NodeHandle nh_;

public:
    //! Constructor
    config_se2_constraint(const std::string& config_file_path="");
    config_se2_constraint(const std::shared_ptr<openvslam::config>& cfg);

    //! Destructor
    ~config_se2_constraint();

    friend std::ostream& operator<<(std::ostream& os, const config_se2_constraint& cfg);

    //! path to config YAML file
    std::string config_file_path_;

    double info_x_;
    double info_y_;
    double info_z_;
    double info_rx_;
    double info_ry_;
    double info_rz_;

    double odom_info_factor_xy_;
    double odom_info_factor_th_;
};


} // namespace openvslam

#endif