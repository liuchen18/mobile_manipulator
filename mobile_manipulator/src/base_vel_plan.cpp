#include "mobile_manipulator/base_vel_plan.h"
#include "math.h"

MY_DWA::MY_DWA(int rate,tra_func_p tra){
    sample_rate=rate;
    dt=1/double(sample_rate);
    MAX_VEL.linear.x=1;
    MAX_VEL.linear.y=1;
    MAX_VEL.angular.z=0.5;
    MAX_ACC.linear.x=0.5;
    MAX_ACC.linear.y=0.5;
    MAX_ACC.angular.z=0.5;
    velocity_resolution=0.01;
    delta_time=1;
    trajectory=tra;
}

void MY_DWA::state_update(double time,geometry_msgs::Twist base_vel,geometry_msgs::Pose2D base_position,geometry_msgs::Pose mani_pose){
    current_time=time;
    current_base_velocity=base_vel;
    current_base_position=base_position;
    current_mani_pose=mani_pose;
}

geometry_msgs::Pose2D MY_DWA::predict_position(double delta_time,geometry_msgs::Twist posible_vel){
    geometry_msgs::Pose2D new_pos;
    new_pos.theta=current_base_position.theta+posible_vel.angular.z*dt;
    new_pos.x=current_base_position.x+posible_vel.linear.x*delta_time*cos(new_pos.theta)-posible_vel.linear.y*delta_time*sin(new_pos.theta);
    new_pos.y=current_base_position.y+posible_vel.linear.y*delta_time*cos(new_pos.theta)+posible_vel.linear.x*delta_time*sin(new_pos.theta);
    return new_pos;

}

void MY_DWA::sample_velocity(){
    clear_sampled_vel();
    double cur_max_x_vel =std::min({MAX_VEL.angular.z,current_base_velocity.linear.x+MAX_ACC.linear.x*dt});
    double cur_max_y_vel =std::min({MAX_VEL.angular.z,current_base_velocity.linear.y+MAX_ACC.linear.y*dt});
    double cur_max_theta_vel =std::min({MAX_VEL.angular.z,current_base_velocity.angular.z+MAX_ACC.angular.z*dt});
    double cur_min_x_vel =std::max({-MAX_VEL.angular.z,current_base_velocity.linear.x-MAX_ACC.linear.x*dt});
    double cur_min_y_vel =std::max({-MAX_VEL.angular.z,current_base_velocity.linear.y-MAX_ACC.linear.y*dt});
    double cur_min_theta_vel =std::max({-MAX_VEL.angular.z,current_base_velocity.angular.z-MAX_ACC.angular.z*dt});

    for(int i=0;i<int((cur_max_x_vel-cur_min_x_vel)/velocity_resolution);i++){
        for(int j=0;j<int((cur_max_y_vel-cur_min_y_vel)/velocity_resolution);j++){
            for(int k=0;k<int((cur_max_theta_vel-cur_min_theta_vel)/velocity_resolution);k++){
                geometry_msgs::Twist cur_vel;
                cur_vel.linear.x=cur_min_x_vel+float(i)*velocity_resolution;
                cur_vel.linear.y=cur_min_y_vel+float(j)*velocity_resolution;
                cur_vel.angular.z=cur_min_theta_vel+float(k)*velocity_resolution;
                sampled_vel.push_back(cur_vel);
            }
        }
    }
}

void MY_DWA::clear_sampled_vel(){
    std::vector<geometry_msgs::Twist>().swap(sampled_vel);

}


double MY_DWA::compute_goal_function(geometry_msgs::Pose2D predicted_base_position,geometry_msgs::Pose desired_mm_pose){
    
    double length = sqrt(current_mani_pose.position.x*current_mani_pose.position.x+current_mani_pose.position.y*current_mani_pose.position.y);
    double alpha = atan2(current_mani_pose.position.y,current_mani_pose.position.x);
    double current_end_x=predicted_base_position.x+length*cos(alpha+predicted_base_position.theta);
    double current_end_y=predicted_base_position.y+length*sin(alpha+predicted_base_position.theta);

    double distance_bias=abs(desired_mm_pose.position.x-current_end_x)+abs(desired_mm_pose.position.y-current_end_y);

    double goal_func=distance_bias;
    return goal_func;
}

geometry_msgs::Twist MY_DWA::compute_best_velocity(){
    geometry_msgs::Twist vel_result;
    double goal=10000;
    sample_velocity();

    geometry_msgs::Pose desired_mm_pose=(*trajectory)(current_time+delta_time);

    for(auto vel:sampled_vel){
        geometry_msgs::Pose2D predited_pos=predict_position(delta_time,vel);//delta time is 1s, predict the position after 1s
        double cur_goal_func=compute_goal_function(predited_pos,desired_mm_pose);
        if(goal > cur_goal_func){
            goal=cur_goal_func;
            vel_result=vel;
        }
    }
    return vel_result;
}