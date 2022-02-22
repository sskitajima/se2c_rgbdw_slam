#ifndef OPENVSLAM_OPTIMIZE_POSE_OPTIMIZER_H
#define OPENVSLAM_OPTIMIZE_POSE_OPTIMIZER_H

#include <memory>

namespace openvslam {

namespace util
{
class coordinate_transformer;
}

namespace data {
class frame;
}

namespace optimize {

namespace g2o{
namespace se2c{
class plane_constraint_wrapper;
class odometry_constraint_wrapper;
}
}


class pose_optimizer {
public:
    /**
     * Constructor
     * @param num_trials
     * @param num_each_iter
     */
    explicit pose_optimizer(const unsigned int num_trials = 4, const unsigned int num_each_iter = 10);

    /**
     * Destructor
     */
    virtual ~pose_optimizer() = default;
    
    /**
     * Perform pose optimization
     * @param frm
     * @return
     */
    unsigned int optimize(data::frame& frm) const;

    /**
     * (Debug)Perform optimization using only plane constraint
     * @param frm
     * @return
     */
    unsigned int optimize_with_plane_constraint(data::frame& frm) const;


    /**
     * (Debug)Perform optimization using only odometry constraint
     * @param frm
     * @return
     */
    unsigned int optimize_with_odometry_constraint(const unsigned fix_frm_id , const Mat44_t& fix_frm_pose , 
                                                                   const unsigned& opt_frm_id, Mat44_t& opt_frm_pose,
                                                                   const Mat44_t& odom_measurement_bb, const Mat66_t& odom_measurement_info_bb) const;


    //! Proposed 
    void set_se2c_wrappers(std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper>& plane_wrapper, std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper>& odom_wrapper);

private:
    //! robust optimizationの試行回数
    const unsigned int num_trials_ = 4;

    //! 毎回のoptimizationのiteration回数
    const unsigned int num_each_iter_ = 10;

    std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper> plane_constraint_wrapper_ = nullptr;
    std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper> odometry_constraint_wrapper_ = nullptr;
};

} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZE_POSE_OPTIMIZER_H
