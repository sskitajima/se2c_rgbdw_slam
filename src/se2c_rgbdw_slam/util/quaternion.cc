#include "quaternion.h"
#include "math.h"

namespace openvslam
{

void normalize(double& qx, double& qy, double& qz, double& qw)
{
    const double length = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    qx /= length;
    qy /= length;
    qz /= length;
    qw /= length;
}

// https://www.kazetest.com/vcmemo/quaternion/quaternion.htm
// q0 + iq1 + jq2 + kq3
void QuaternionToEulerAngles(double q0, double q1, double q2, double q3,
                                    double& roll, double& pitch, double& yaw)
{
    double q0q0 = q0 * q0;
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    double q0q3 = q0 * q3;
    double q1q1 = q1 * q1;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q3q3 = q3 * q3;
    roll = atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
    pitch = asin(2.0 * (q0q2 - q1q3));
    yaw = atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
}

// https://www.kazetest.com/vcmemo/quaternion/quaternion.htm
// q0 + iq1 + jq2 + kq3
void EulerAnglesToQuaternion(double roll, double pitch, double yaw,
                                    double& q0, double& q1, double& q2, double& q3) {
    double cosRoll = cos(roll / 2.0);
    double sinRoll = sin(roll / 2.0);
    double cosPitch = cos(pitch / 2.0);
    double sinPitch = sin(pitch / 2.0);
    double cosYaw = cos(yaw / 2.0);
    double sinYaw = sin(yaw / 2.0);

    q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

    normalize(q0, q1, q2, q3);
}

} // namespace openvslam
