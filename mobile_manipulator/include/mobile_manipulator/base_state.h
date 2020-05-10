#ifndef BASE_VEL_PLAN_H
#define BASE_VEL_PLAN_H

#include "nav_msgs/Odometry.h"

class base_state{
    public:
    base_state();
    void base_Callback(const nav_msgs::Odometry& msg);
    std::vector<double> base_vel;
    std::vector<double> base_position;
};

#endif