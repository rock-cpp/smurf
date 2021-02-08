#pragma once
#include <Eigen/Core>

namespace smurf
{

struct ContactParams
{
    void setZero()
    {
        max_num_contacts = 4;
        erp = 0.2;
        cfm = 1e-05;
        friction1 = 0.8;
        friction2 = 0.8;
        friction_direction1 = 0;
        rolling_friction = 0.001;
        spinning_friction = 0.001;
        motion1 = motion2 = 0;
        fds1 = fds2 = 0;
        bounce = bounce_vel = 0;
        approx_pyramid = 1;
        coll_bitmask = 65535;
        depth_correction = 0.0;
    }

    void contact_params()
    {
        setZero();
    }

    int max_num_contacts;
    double erp, cfm;
    double friction1, friction2;
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> *friction_direction1;
    double rolling_friction, spinning_friction;
    double motion1, motion2;
    double fds1, fds2;
    double bounce, bounce_vel;
    bool approx_pyramid;
    int coll_bitmask;
    double depth_correction;
};

}