
#include "mobile_manipulator/base_vel_plan.h"
#include "ros/ros.h"
#include <Eigen/Dense>
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"

static bool start_to_move = false;

class base_state{
    public:
    base_state(){};
    void base_Callback(const nav_msgs::Odometry& msg);
    geometry_msgs::Twist current_base_velocity;
    geometry_msgs::Pose2D current_base_position;
};

void base_state::base_Callback(const nav_msgs::Odometry& msg){
    current_base_velocity=msg.twist.twist;
    current_base_position.x=msg.pose.pose.position.x;
    current_base_position.y=msg.pose.pose.position.y;
    Eigen::Quaterniond cur_q(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z);
    Eigen::Vector3d rpy=cur_q.matrix().eulerAngles(2,1,0);
    int i=0;
    while(1){
        rpy[0]+=i*M_PI;
        if(abs(current_base_position.theta-rpy[0])<1){
            current_base_position.theta=rpy[0];
            break;
        }
        rpy[0]-=2*i*M_PI;
        if(abs(current_base_position.theta-rpy[0])<1){
            current_base_position.theta=rpy[0];
            break;
        }
        rpy[0]+=i*M_PI;
        i+=1;
    }
}

class manipulator_state{
    public:
    manipulator_state(){};
    void manipulator_Callback(const geometry_msgs::Pose& msg);
    geometry_msgs::Pose current_manipulator_pose;
};

void manipulator_state::manipulator_Callback(const geometry_msgs::Pose& msg){
    current_manipulator_pose=msg;
}

geometry_msgs::Pose desired_trajectory(double time){
    geometry_msgs::Pose d_pose;
    d_pose.position.x=0.2;
    d_pose.position.y=0.2*time + 0.3;
    d_pose.position.z=1.2;
    d_pose.orientation.x=0.707;
    d_pose.orientation.y=0.0;
    d_pose.orientation.z=0.0;
    d_pose.orientation.w=0.707;
    return d_pose;
}

void start_to_move_callback(const std_msgs::Bool& msg){
    start_to_move=msg.data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test");
    MY_DWA base_planner(10,desired_trajectory);
    base_state current_base_state;
    manipulator_state current_manipulator_state;

    ros::NodeHandle nh;
    ros::Subscriber start_sub = nh.subscribe("start_base", 1000, &start_to_move_callback);
    ros::Subscriber base_state_sub =  nh.subscribe("odom",1000,&base_state::base_Callback, &current_base_state);
    ros::Subscriber manipulator_state_sub =  nh.subscribe("manipulator_state",1000,&manipulator_state::manipulator_Callback, &current_manipulator_state);
    ros::Publisher base_cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
     
    ros::Rate loop_rate(10);
    ROS_INFO("waiting for the manipulator signal!");
    while (!start_to_move && ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("start to move the base!");

    double start_time=ros::Time::now().toSec();
    while(ros::ok()){
        base_planner.state_update(ros::Time::now().toSec()-start_time,
                                current_base_state.current_base_velocity,
                                current_base_state.current_base_position,
                                current_manipulator_state.current_manipulator_pose);
        geometry_msgs::Twist vel_cmd=base_planner.compute_best_velocity();
        #ifndef NBEBUG
            ROS_INFO("x_vel: %f, y_vel: %f, theta_vel: %f",vel_cmd.linear.x,vel_cmd.linear.y,vel_cmd.angular.z);
        #endif
        base_cmd_pub.publish(vel_cmd);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}