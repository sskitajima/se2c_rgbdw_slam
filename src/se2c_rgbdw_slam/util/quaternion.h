#ifndef OPENVSLAM_UITL_QUATERNION_H
#define OPENVSLAM_UITL_QUATERNION_H

namespace openvslam
{

void normalize(double& qx, double& qy, double& qz, double& qw);

// https://www.kazetest.com/vcmemo/quaternion/quaternion.htm
// q0 + iq1 + jq2 + kq3
void QuaternionToEulerAngles(double q0, double q1, double q2, double q3,
                                    double& roll, double& pitch, double& yaw);


// https://www.kazetest.com/vcmemo/quaternion/quaternion.htm
// q0 + iq1 + jq2 + kq3
void EulerAnglesToQuaternion(double roll, double pitch, double yaw,
                                    double& q0, double& q1, double& q2, double& q3);
    

} // namespace openvslam

#endif