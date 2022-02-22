#ifndef OPENVSLAM_OPTIMIZE_GRAPH_OPTIMIZER_H
#define OPENVSLAM_OPTIMIZE_GRAPH_OPTIMIZER_H

#include "se2c_rgbdw_slam/module/type.h"

#include <map>
#include <set>

namespace openvslam {

namespace data {
class keyframe;
class map_database;
} // namespace data

namespace optimize {

namespace g2o{
namespace se2c{
class plane_constraint_wrapper;
class odometry_constraint_wrapper;
}
}

class graph_optimizer {
public:
    /**
     * Constructor
     * @param map_db
     * @param fix_scale
     */
    explicit graph_optimizer(data::map_database* map_db, const bool fix_scale);

    /**
     * Destructor
     */
    virtual ~graph_optimizer() = default;

    /**
     * Perform pose graph optimization
     * @param loop_keyfrm
     * @param curr_keyfrm
     * @param non_corrected_Sim3s
     * @param pre_corrected_Sim3s
     * @param loop_connections
     */
    void optimize(data::keyframe* loop_keyfrm, data::keyframe* curr_keyfrm,
                  const module::keyframe_Sim3_pairs_t& non_corrected_Sim3s,
                  const module::keyframe_Sim3_pairs_t& pre_corrected_Sim3s,
                  const std::map<data::keyframe*, std::set<data::keyframe*>>& loop_connections) const;

    //! Proposed 
    void set_se2c_wrappers(std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper>& plane_wrapper, std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper>& odom_wrapper);

private:
    //! map database
    const data::map_database* map_db_;

    //! SE3 optimization or Sim3 optimization
    const bool fix_scale_;

    //! Proposed
    std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper> plane_constraint_wrapper_ = nullptr;
    std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper> odometry_constraint_wrapper_ = nullptr;
};

} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_GRAPH_OPTIMIZER_H
