#include "octomap_mapping.h"

#include "se2c_rgbdw_slam/data/landmark.h"

#include "se2c_rgbdw_slam/type.h"

#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>


namespace openvslam
{
namespace module {

// constructor, destructor
////////////////////////////////////////////////////////////////////
octomap_mapping::octomap_mapping(const std::shared_ptr<config>& cfg, data::map_database* map_db)
: cfg_(cfg),
  voxel_size_(cfg_->yaml_node_["Octomap.voxel_size"].as<double>(0.01)),
  occupancy_thres_(cfg_->yaml_node_["Octomap.occupancy_thres"].as<double>(0.61)),
  prob_hit_(cfg_->yaml_node_["Octomap.prob_hit"].as<double>(0.6)),
  prob_miss_(cfg_->yaml_node_["Octomap.prob_miss"].as<double>(0.45)),
  fx_(cfg_->yaml_node_["Camera.fx"].as<double>(530)),
  fy_(cfg_->yaml_node_["Camera.fy"].as<double>(530)),
  cx_(cfg_->yaml_node_["Camera.cx"].as<double>(475)),
  cy_(cfg_->yaml_node_["Camera.cy"].as<double>(265)),
  rows_(cfg_->yaml_node_["Camera.rows"].as<int>(540)),
  cols_(cfg_->yaml_node_["Camera.cols"].as<int>(960)),
  cam_dmin_(cfg_->yaml_node_["Octomap.min_depth"].as<double>(0.02)),
  cam_dmax_(cfg_->yaml_node_["Octomap.max_depth"].as<double>(4.5)),
  map_db_(map_db)
{
    octomap_tree_ = new octomap::OcTree ( voxel_size_ );
    octomap_tree_->setOccupancyThres ( occupancy_thres_ );
    octomap_tree_->setProbHit ( prob_hit_ );
    octomap_tree_->setProbMiss ( prob_miss_);

    spdlog::debug("octomap_mapping::Construct");
}

octomap_mapping::~octomap_mapping()
{
    spdlog::debug("octomap_mapping::Destruct");

    delete octomap_tree_;
    octomap_tree_ = nullptr;
}

// public
////////////////////////////////////////////////////////////////////

void octomap_mapping::insert_keyframe(const data::keyframe* key_frm)
{
    // spdlog::debug("octomap_mapping::insert_keyframe");

    // keyfrm上の現在位置とlandmarkを取得する
    const Mat44_t keyfrm_pose_irc = key_frm->get_cam_pose_inv();
    const cv::Mat img_depth = key_frm->get_depthmap();

    // pointcloudを作る
    octomap::Pointcloud cloud;
    create_pointcloud(cloud, img_depth, keyfrm_pose_irc);

    // pointcloudを挿入する
    insert_pointcloud(cloud, keyfrm_pose_irc);

    // octreeを更新する
    update_octree();

    // spdlog::debug("octomap_mapping::insert_keyframe finish");
}

octomap::OcTree* octomap_mapping::get_octomap_tree() const
{
    std::lock_guard<std::mutex> lock(mtx_octree_);
    return octomap_tree_;
}

void octomap_mapping::rebuild_octree_for_all_keyfrms()
{
    spdlog::debug("rebuilding octomap start...");
    is_rebuilding_ = true;

    {
        std::lock_guard<std::mutex> lock_full ( mtx_octree_ );
        octomap_tree_->clear();
    }

    std::vector<data::keyframe*> all_keyfrms = map_db_->get_all_keyframes();
    for(const auto keyfrm : all_keyfrms)
    {
        insert_keyframe(keyfrm);
    }

    is_rebuilding_ = false;
    spdlog::debug("rebuilding octomap finish...");

}

void octomap_mapping::save_octree(const std::string& save_path)
{
    std::lock_guard<std::mutex> lock_full ( mtx_octree_ );
	octomap_tree_->writeBinary( save_path );
}

void octomap_mapping::set_coord_transformer(std::shared_ptr<util::coordinate_transformer>& coord_transformer)
{
    coord_transformer_ = coord_transformer;
}


// private
////////////////////////////////////////////////////////////////////

void octomap_mapping::create_pointcloud(octomap::Pointcloud& cloud, const cv::Mat& img_depth, const Mat44_t& keyfrm_pose_irc) const
{
    for(int i=0; i<img_depth.cols; i++)
    {
        for(int j=0; j<img_depth.rows; j++)
        {
            double depth = img_depth.at<float> ( j,i );

            if ( depth > cam_dmin_ && depth < cam_dmax_ ) {
                const Eigen::Vector4d cp = recover_points(depth, i, j);
                const Eigen::Vector4d irp = keyfrm_pose_irc * cp;
                cloud.push_back ( irp[0], irp[1], irp[2] );
            }
        }
    }
}


void octomap_mapping::insert_pointcloud(const octomap::Pointcloud& cloud, const Mat44_t& keyfrm_pose_irc)
{
    // spdlog::debug("octomap_mapping::insert_pointcloud");

    octomap::point3d current_cam_center_3d(
        keyfrm_pose_irc(0,3),
        keyfrm_pose_irc(1,3),
        keyfrm_pose_irc(2,3)
    );

    std::lock_guard<std::mutex> lock(mtx_octree_);
    octomap_tree_->insertPointCloud ( cloud, current_cam_center_3d, -1, true, true );
}

void octomap_mapping::update_octree()
{
    // spdlog::debug("octomap_mapping::update octree");

    std::lock_guard<std::mutex> lock(mtx_octree_);
    octomap_tree_->updateInnerOccupancy();
}

Eigen::Vector4d octomap_mapping::recover_points(const double depth, const double ux, const double uy ) const
{
    Eigen::Vector4d xyz;
    xyz[0] = ( ux - cx_ ) * depth/fx_;
    xyz[1] = ( uy - cy_ ) * depth/fy_;
    xyz[2] = depth;
    xyz[3] = 1.0;
    return xyz;
}

} // namespace module
} // namespace openvslam