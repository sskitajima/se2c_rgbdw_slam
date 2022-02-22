#ifndef OPENVSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_H
#define OPENVSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_H

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

class local_bundle_adjuster {
public:
    /**
     * Constructor
     * @param map_db
     * @param num_first_iter
     * @param num_second_iter
     */
    explicit local_bundle_adjuster(const unsigned int num_first_iter = 5,
                                   const unsigned int num_second_iter = 10);

    /**
     * Destructor
     */
    virtual ~local_bundle_adjuster() = default;

    /**
     * Perform optimization
     * @param curr_keyfrm
     * @param force_stop_flag
     */
    void optimize(data::keyframe* curr_keyfrm, bool* const force_stop_flag) const;
    
    //! Proposed 
    void set_se2c_wrappers(std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper>& plane_wrapper, std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper>& odom_wrapper);

private:
    //! number of iterations of first optimization
    const unsigned int num_first_iter_;
    //! number of iterations of second optimization
    const unsigned int num_second_iter_;

    std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper> plane_constraint_wrapper_ = nullptr;
    std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper> odometry_constraint_wrapper_ = nullptr;

};

} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_H
