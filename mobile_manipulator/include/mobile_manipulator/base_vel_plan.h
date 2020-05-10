#ifndef BASE_VEL_PLAN_H
#define BASE_VEL_PLAN_H

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "vector"
typedef geometry_msgs::Pose (*tra_func_p)(double);



class MY_DWA{
    public:
    /* construction function*/
    MY_DWA(int rate,tra_func_p tra);
    /*update current state, input is current base velocity, current base position and current manipulator pose*/
    void state_update(double time,geometry_msgs::Twist base_vel,geometry_msgs::Pose2D base_position,geometry_msgs::Pose mani_pose);
    /*predict the base position after delta time,input is posible velocity*/
    geometry_msgs::Pose2D predict_position(double delta_time,geometry_msgs::Twist posible_vel);
    /*sample in velocity domain, no inputs, result is in sampled_vel*/
    void sample_velocity();
    /*compute the goal function for the predicted base position, inputs are predicted base position and desired mobole manipulator pose in delta time*/
    double compute_goal_function(geometry_msgs::Pose2D predicted_base_position,geometry_msgs::Pose desired_mm_pose);
    /*using all the functions to get the best velocity, return best velocity*/
    geometry_msgs::Twist compute_best_velocity();
    /*clear the sample velocity*/
    void clear_sampled_vel();

    private:
    std::vector<geometry_msgs::Twist> sampled_vel;//all sampled velocity at current time
    int sample_rate;
    double dt;
    geometry_msgs::Twist MAX_VEL;
    geometry_msgs::Twist MAX_ACC;
    geometry_msgs::Twist current_base_velocity;
    geometry_msgs::Pose2D current_base_position;
    geometry_msgs::Pose current_mani_pose;
    double velocity_resolution;
    double current_time;
    double delta_time;
    tra_func_p trajectory;


};

#endif