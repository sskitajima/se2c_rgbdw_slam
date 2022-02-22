#ifndef OPENVSLAM_MODULE_OCTOMAP_MAPPING_H
#define OPENVSLAM_MODULE_OCTOMAP_MAPPING_H

#include "se2c_rgbdw_slam/config.h"
#include "se2c_rgbdw_slam/data/keyframe.h"
#include "se2c_rgbdw_slam/camera/perspective.h"
#include "se2c_rgbdw_slam/data/map_database.h"
#include "se2c_rgbdw_slam/util/coordinate_transformer.h"

#include <octomap/octomap.h>

#include <mutex>

namespace openvslam
{
namespace module
{
class octomap_mapping
{
private:    
    const std::shared_ptr<config> cfg_;

    double voxel_size_;
    double occupancy_thres_;
    double prob_hit_;
    double prob_miss_;

    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double rows_;
    double cols_;
    double cam_dmin_;
    double cam_dmax_;

    data::map_database* map_db_ = nullptr;

    mutable std::mutex mtx_octree_;
    octomap::OcTree* octomap_tree_;

    std::shared_ptr<util::coordinate_transformer> coord_transformer_;

    // helper function
    void create_pointcloud(octomap::Pointcloud& cloud, const cv::Mat& depthmap, const Mat44_t& keyfrm_pose_wc) const;
    void insert_pointcloud(const octomap::Pointcloud& cloud, const Mat44_t& cam_pose_wc);
    Eigen::Vector4d recover_points(const double depth, const double ux, const double uy ) const;


    // system status
    bool is_rebuilding_ = false;
   
public:
    // Constructor and Destructor
    octomap_mapping(const std::shared_ptr<config>& cfg, data::map_database* map_db);
    ~octomap_mapping();

    // Setter

    // Getter
    octomap::OcTree* get_octomap_tree() const;
    void set_coord_transformer(std::shared_ptr<util::coordinate_transformer>& coord_transformer);

    // module
    void insert_keyframe(const data::keyframe* key_frm);
    void update_octree();
    
    // Data input and output
    void rebuild_octree_for_all_keyfrms();
    void save_octree(const std::string& save_path);
};


} // namespace module
} // namespace openvslam

#endif
