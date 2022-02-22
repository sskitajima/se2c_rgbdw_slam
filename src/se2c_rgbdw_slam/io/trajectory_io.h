#ifndef OPENVSLAM_IO_TRAJECTORY_IO_H
#define OPENVSLAM_IO_TRAJECTORY_IO_H

#include "se2c_rgbdw_slam/util/coordinate_transformer.h"

#include <string>
#include <memory>

#include <tf/transform_datatypes.h>

namespace openvslam {

namespace data {
class map_database;
} // namespace data

namespace io {

class trajectory_io {
public:
    /**
     * Constructor
     */
    explicit trajectory_io(data::map_database* map_db);

    /**
     * Destructor
     */
    ~trajectory_io() = default;

    /**
     * Save the frame trajectory in the specified format
     */
    void save_frame_trajectory(const std::string& path, const std::string& format) const;

    /**
     * Save the keyframe trajectory in the specified format
     */
    void save_keyframe_trajectory(const std::string& path, const std::string& format) const;

    void set_coord_transformer(const std::shared_ptr<openvslam::util::coordinate_transformer>& coord_transformer);

private:
    //! map_database
    data::map_database* const map_db_ = nullptr;

    std::shared_ptr<openvslam::util::coordinate_transformer> coord_transformer_ = nullptr;

};

} // namespace io
} // namespace openvslam

#endif // OPENVSLAM_IO_TRAJECTORY_IO_H
