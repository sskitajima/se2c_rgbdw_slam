#ifndef OPENVSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H
#define OPENVSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H

namespace openvslam {

namespace data {
class map_database;
} // namespace data

namespace optimize {

namespace g2o{
namespace se2c{
class plane_constraint_wrapper;
class odometry_constraint_wrapper;
}
}

class global_bundle_adjuster {
public:
    /**
     * Constructor
     * @param map_db
     * @param num_iter
     * @param use_huber_kernel
     */
    explicit global_bundle_adjuster(data::map_database* map_db, const unsigned int num_iter = 10, const bool use_huber_kernel = true);

    /**
     * Destructor
     */
    virtual ~global_bundle_adjuster() = default;

    /**
     * Perform optimization
     * @param lead_keyfrm_id_in_global_BA
     * @param force_stop_flag
     */
    void optimize(const unsigned int lead_keyfrm_id_in_global_BA = 0, bool* const force_stop_flag = nullptr) const;

    //! Proposed 
    void set_se2c_wrappers(std::shared_ptr<optimize::g2o::se2c::plane_constraint_wrapper>& plane_wrapper, std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper>& odom_wrapper);

private:
    //! map database
    const data::map_database* map_db_;

    //! number of iterations of optimization
    unsigned int num_iter_;

    //! use Huber loss or not
    const bool use_huber_kernel_;
    
    //! Proposed
    std::shared_ptr<g2o::se2c::plane_constraint_wrapper> plane_constraint_wrapper_ = nullptr;
    std::shared_ptr<optimize::g2o::se2c::odometry_constraint_wrapper> odometry_constraint_wrapper_ = nullptr;
};

} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H
