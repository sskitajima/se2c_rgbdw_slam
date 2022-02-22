#ifndef OPENVSLAM_DATA_ENCODER_H
#define OPENVSLAM_DATA_ENCODER_H

namespace openvslam
{
class encoder
{
private:
public:

    double wheel_r_;
    double wheel_l_;

    double timestamp_;
    double delta_t_;
    double v_;
    double omega_;

    encoder(){};
    encoder(const double r, const double l, const double t, const double time)
    : wheel_r_(r), wheel_l_(l), timestamp_(time), delta_t_(t), v_(-1), omega_(-1)
    {};
    encoder(const double v, const double omega, const double t)
    : wheel_r_(-1), wheel_l_(-1), delta_t_(t), v_(v), omega_(omega)
    {};

    ~encoder(){};


};

}

#endif