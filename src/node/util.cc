#include "util.h"

namespace openvslam_ros
{
using namespace openvslam;

tf::Transform converter::Eigen2TF(const openvslam::Mat44_t& mat)
{
    return tf::Transform( tf::Matrix3x3(mat(0, 0), mat(0, 1), mat(0, 2),
                                        mat(1, 0), mat(1, 1), mat(1, 2),
                                        mat(2, 0), mat(2, 1), mat(2, 2)),
                           tf::Vector3(mat(0,3), mat(1,3), mat(2, 3))
                        );

}

openvslam::Mat44_t converter::TF2Eigen(const tf::Transform& tf)
{
    openvslam::Mat44_t mat;
    tf::Matrix3x3 rot = tf.getBasis();
    tf::Vector3 trans = tf.getOrigin();
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++) mat(i, j) = rot.getColumn(i)[j];
    }
    for(int j=0; j<3; j++) mat(j, 3) = trans[j];
    mat(3, 3) = 1.0;

    return mat;

}

} // namespace openvslam_ros