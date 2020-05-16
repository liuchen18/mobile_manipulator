//#define DEBUG_CONTROL
#define DEBUG_TIME
//#define SHOW_BASE_STATE
//#define DEBUG
//#define DEBUG_POSITION_ERROR
#define DEBUG_ORIENTATION_ERROR
#include "ros/ros.h"
#include <Eigen/Dense>
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryGoal.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <fstream>
#include "math.h"

class manipulator_state{
    public:
    manipulator_state(){};
    void manipulator_Callback(const control_msgs::JointTrajectoryControllerState& msg);

    double current_joint_values[6];
};

/*subscribe the joint state*/
void manipulator_state::manipulator_Callback(const control_msgs::JointTrajectoryControllerState& msg){
    for(int i=0;i<6;i++){
        current_joint_values[i]=msg.desired.positions[i];
    }
}

/*generate trajectory message*/
class trajectory{
    public:

    geometry_msgs::Pose desired_trajectory_pose(double time);
    std::vector<double> desired_trajectory_velocity(double time);
};

geometry_msgs::Pose trajectory::desired_trajectory_pose(double time){
    geometry_msgs::Pose d_pose;
    d_pose.position.x=0.2;
    d_pose.position.y=0.2*time + 0.3;
    d_pose.position.z=1;
    d_pose.orientation.x=0.707;
    d_pose.orientation.y=0.0;
    d_pose.orientation.z=0.0;
    d_pose.orientation.w=0.707;
    return d_pose;
}
std::vector<double> trajectory::desired_trajectory_velocity(double time){
    std::vector<double> vel;
    vel.push_back(0);
    vel.push_back(0.2);
    vel.push_back(0);
    vel.push_back(0);
    vel.push_back(0);
    vel.push_back(0);
    return vel;
}

class base_state{
    public:
    base_state();
    void base_Callback(const nav_msgs::Odometry& msg);
    std::vector<double> base_vel;
    std::vector<double> base_position;
};

base_state::base_state():base_vel(3),base_position(3){}

void base_state::base_Callback(const nav_msgs::Odometry& msg){
    base_vel[0]=msg.twist.twist.linear.x;
    base_vel[1]=msg.twist.twist.linear.y;
    base_vel[2]=msg.twist.twist.angular.z;
    base_position[0]=msg.pose.pose.position.x;
    base_position[1]=msg.pose.pose.position.y;
    Eigen::Quaterniond cur_q(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z);
    Eigen::Vector3d rpy=cur_q.matrix().eulerAngles(2,1,0);
    int i=0;
    while(1){
        rpy[0]+=i*M_PI;
        if(abs(base_position[2]-rpy[0])<1){
            base_position[2]=rpy[0];
            break;
        }
        rpy[0]-=2*i*M_PI;
        if(abs(base_position[2]-rpy[0])<1){
            base_position[2]=rpy[0];
            break;
        }
        rpy[0]+=i*M_PI;
        i+=1;
    }
    #ifdef SHOW_BASE_STATE
        ROS_INFO("quaternion: w: %f, x: %f, y: %f, z: %f",msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z);
        ROS_INFO("current base vel: vel_x: %f, vel_y: %f, vel_theta: %f",base_vel[0],base_vel[1],base_vel[2]);
        ROS_INFO("current base position: position_x: %f, position_y: %f, position_theta: %f",base_position[0],base_position[1],base_position[2]);
    #endif
}

/* compute joint velocity, according to desired cartisian velocity, current pose. desired pose and jacobian matrix*/
std::vector<double> compute_joint_velocity(const std::vector<double>& desired_cartisian_vel,
                                            const geometry_msgs::Pose& current_cartisian_pose,
                                            const geometry_msgs::Pose& desired_cartisian_pose,
                                            const Eigen::MatrixXd& jacobian){
    double K[6]={10,10,2,2,2,10};

    //generate error
    Eigen::Vector3d position_error;
    position_error[0]=desired_cartisian_pose.position.x-current_cartisian_pose.position.x;
    position_error[1]=desired_cartisian_pose.position.y-current_cartisian_pose.position.y;
    position_error[2]=desired_cartisian_pose.position.z-current_cartisian_pose.position.z;
    Eigen::Vector3d qc(current_cartisian_pose.orientation.x, current_cartisian_pose.orientation.y, current_cartisian_pose.orientation.z);
    Eigen::Vector3d qd(desired_cartisian_pose.orientation.x, desired_cartisian_pose.orientation.y, desired_cartisian_pose.orientation.z);
    Eigen::Vector3d orientation_error = current_cartisian_pose.orientation.w*qd-desired_cartisian_pose.orientation.w*qc+qc.cross(qd);

    #ifdef DEBUG_POSITION_ERROR
        ROS_INFO("position x error: %f ",position_error[0]);
        //ROS_INFO("desired x: %f, actual x: %f",desired_cartisian_pose.position.x,current_cartisian_pose.position.x);
        ROS_INFO(" ");
    #endif
    
    #ifdef DEBUG_CONTROL
        ROS_INFO("desired cartisian velocity: %f, %f, %f, %f, %f, %f",desired_cartisian_vel[0],desired_cartisian_vel[1],desired_cartisian_vel[2],desired_cartisian_vel[4],desired_cartisian_vel[4],desired_cartisian_vel[5]);
        ROS_INFO("POSITION ERROR: %f, %f, %f",position_error[0],position_error[1],position_error[2]);
        ROS_INFO("orientation error: %f, %f, %f",orientation_error[0],orientation_error[1],orientation_error[2]);
        ROS_INFO("det of the jacobian: %f",jacobian.determinant());
        //ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
    #endif

    //compute cartisian velocity
    Eigen::Matrix<double,6,1> cartisian_velocity;
    for(int i=0;i<3;i++){
        cartisian_velocity(i,0)=K[i]*position_error[i]+desired_cartisian_vel[i];
    }
    for(int i=0;i<3;i++){
        cartisian_velocity(i+3,0)=K[i+3]*orientation_error[i]+desired_cartisian_vel[i+3];
    }

    #ifdef DEBUG_ORIENTATION_ERROR
        ROS_INFO("current orientation error: R: %f, P: %f, Y: %f",orientation_error[0],orientation_error[1],orientation_error[2]);
        ROS_INFO("desired cartisian velocity: R: %f, P: %f, Y: %f",desired_cartisian_vel[3],desired_cartisian_vel[4],desired_cartisian_vel[5]);
        ROS_INFO("computed cartisian velocity: R: %f, P: %f, Y: %f",cartisian_velocity(3,0),cartisian_velocity(4,0),cartisian_velocity(5,0));
        ROS_INFO(" ");
    #endif

    //compute joint velocity
    Eigen::Matrix<double,6,1> velocity=jacobian.inverse()*cartisian_velocity;
    std::vector<double> joint_velocity;
    for(int i=0;i<6;i++){
        joint_velocity.push_back(velocity[i]);
    }
    #ifdef DEBUG_CONTROL
        ROS_INFO("cartisian velocity: %f, %f, %f, %f, %f, %f",cartisian_velocity(0,0),cartisian_velocity(1,0),cartisian_velocity(2,0),cartisian_velocity(3,0),cartisian_velocity(4,0),cartisian_velocity(5,0));
        ROS_INFO("joint velocity: %f, %f, %f, %f, %f, %f",joint_velocity[0],joint_velocity[1],joint_velocity[2],joint_velocity[3],joint_velocity[4],joint_velocity[5]);
    #endif
    return joint_velocity;
    
}

std::vector<double> compute_desired_manipulator_cartisian_velocity(const geometry_msgs::Pose current_end_effector_pose,
                                                                    const std::vector<double> current_base_position,
                                                                    const std::vector<double> desired_mm_cartiian_velocity,
                                                                    const std::vector<double> current_base_velocity){
    std::vector<double> desired_manipulator_cartisian_velocity(desired_mm_cartiian_velocity);
    double R=sqrt(current_end_effector_pose.position.x*current_end_effector_pose.position.x+current_end_effector_pose.position.y*current_end_effector_pose.position.y);
    double alpha = atan2(current_end_effector_pose.position.y,current_end_effector_pose.position.x);
    double theta = current_base_position[2];

    desired_manipulator_cartisian_velocity[0]=desired_mm_cartiian_velocity[0]*cos(theta)+desired_mm_cartiian_velocity[1]*sin(theta);
    desired_manipulator_cartisian_velocity[1]=-desired_mm_cartiian_velocity[0]*sin(theta)+desired_mm_cartiian_velocity[1]*cos(theta);

    desired_manipulator_cartisian_velocity[0]-=(current_base_velocity[0]-current_base_velocity[2]*R*sin(alpha));
    desired_manipulator_cartisian_velocity[1]-=(current_base_velocity[1]+current_base_velocity[2]*R*cos(alpha));
    desired_manipulator_cartisian_velocity[5]-=current_base_velocity[2];
    #ifdef DEBUG_VEL
        ROS_INFO("mm cartisian velocity: X: %f, Y: %f, theta: %f",desired_mm_cartiian_velocity[0],desired_mm_cartiian_velocity[1],desired_mm_cartiian_velocity[5]);
        ROS_INFO("current base velocity: X: %f, Y: %f, theta: %f",current_base_velocity[0],current_base_velocity[1],current_base_velocity[2]);
        ROS_INFO("manipulator cartisian velocity: X: %f, Y: %f, theta: %f",desired_manipulator_cartisian_velocity[0],desired_manipulator_cartisian_velocity[1],desired_manipulator_cartisian_velocity[5]);
    #endif
    return desired_manipulator_cartisian_velocity;
}

geometry_msgs::Pose compute_desired_manipulator_end_pose(geometry_msgs::Pose desired_mm_pose,const std::vector<double> current_base_position){
    geometry_msgs::Pose desired_mani_end_pose;
    double cartisian_x=desired_mm_pose.position.x-current_base_position[0];
    double cartisian_y=desired_mm_pose.position.y-current_base_position[1];
    desired_mani_end_pose.position.z=desired_mm_pose.position.z;

    double alpha=atan2(cartisian_y,cartisian_x);
    double length=sqrt(cartisian_y*cartisian_y+cartisian_x*cartisian_x);
    desired_mani_end_pose.position.x=length*cos(alpha-current_base_position[2]);
    desired_mani_end_pose.position.y=length*sin(alpha-current_base_position[2]);

    #ifdef DEBUG_POSITION_ERROR
        ROS_INFO("desired mm x: %f, actual base x: %f",desired_mm_pose.position.x,current_base_position[0]);
        ROS_INFO("cartisian_x: %f, cartisian_y: %f",cartisian_x,cartisian_y);
        ROS_INFO("current theta: %f",current_base_position[2]);
        ROS_INFO("desired mani x: %f",desired_mani_end_pose.position.x);
        ROS_INFO("****************************************************88");
    #endif


    Eigen::Quaterniond mm_q(desired_mm_pose.orientation.w,desired_mm_pose.orientation.x,desired_mm_pose.orientation.y,desired_mm_pose.orientation.z);
    Eigen::Vector3d mm_rpy=mm_q.matrix().eulerAngles(2,1,0);
    mm_rpy[0]-=current_base_position[2];
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(mm_rpy(2),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(mm_rpy(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(mm_rpy(0),Eigen::Vector3d::UnitZ()));
 
    Eigen::Quaterniond quaternion;
    quaternion=yawAngle*pitchAngle*rollAngle;

    Eigen::Vector4d nums=quaternion.coeffs();
    desired_mani_end_pose.orientation.x=nums[0];
    desired_mani_end_pose.orientation.y=nums[1];
    desired_mani_end_pose.orientation.z=nums[2];
    desired_mani_end_pose.orientation.w=nums[3];
    return desired_mani_end_pose;
}

/* compute next 0.02s velocity, according to desired joint velocity and delta joint velocity*/
/*std::vector<double> compute_next_joint_velocity(const std::vector<double> desired_joint_velocity,const std::vector<double> delta_joint_velocity){
    std::vector<double> next_joint_velocity;
    for(int i=0;i<6;i++){
        next_joint_velocity.push_back(desired_joint_velocity[i]+delta_joint_velocity[i]);
    }
    #ifdef DEBUG
        ROS_INFO("next joint velocity: %f, %f, %f, %f, %f, %f",next_joint_velocity[0],next_joint_velocity[1],next_joint_velocity[2],next_joint_velocity[3],next_joint_velocity[4],next_joint_velocity[5]);
    #endif
    return next_joint_velocity;
}*/

/*transfor the velocity to joint position in next 0.02s*/
std::vector<double> compute_next_joint_position(const double current_joint_values[],const std::vector<double>& next_joint_velocity){
    std::vector<double> next_joint_position;
    for(int i=0;i<6;i++){
        next_joint_position.push_back(next_joint_velocity[i]*0.02+current_joint_values[i]);
    }
    #ifdef DEBUG
        ROS_INFO("next joint position: %f, %f, %f, %f, %f, %f",next_joint_position[0],next_joint_position[1],next_joint_position[2],next_joint_position[3],next_joint_position[4],next_joint_position[5]);
    #endif
    return next_joint_position;
}

/*drive the manipulator to an init joint value*/
void manipulator_joint_init(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& action_client,std::vector<double> init_value){
    ROS_INFO("trying to initialize the manipulator");
    control_msgs::FollowJointTrajectoryGoal init_goal;
    init_goal.trajectory.header.stamp=ros::Time::now();
    init_goal.trajectory.joint_names={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    init_goal.trajectory.points.resize(1);
    init_goal.trajectory.points[0].time_from_start=ros::Duration(3);
    init_goal.trajectory.points[0].positions.resize(6);
    if(init_value.size()==6){
        for(int i =0;i<6;i++){
            init_goal.trajectory.points[0].positions[i]=init_value[i];
        }
        action_client.sendGoal(init_goal);
        //ROS_INFO("sending action goal!");
        action_client.waitForResult(ros::Duration(6.0));
        if(action_client.getState()==actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("manipulator initialized!");
        }
        else{
            ROS_ERROR("error occered. the manipulator is NOT initialized");
        }
    }
    else{
        ROS_ERROR("the number of init_joint_value should be 6");
    }
}

/*send the next action goal to control the manipulator real time*/
void manipulator_send_goal(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& action_client,std::vector<double> next_joint_values){
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.header.stamp=ros::Time::now();
    goal.trajectory.joint_names={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].time_from_start=ros::Duration(0.02);
    goal.trajectory.points[0].positions.resize(6);
    if(next_joint_values.size()==6){
        for(int i =0;i<6;i++){
            goal.trajectory.points[0].positions[i]=next_joint_values[i];
        }
        action_client.sendGoal(goal);
        //bool succes=action_client.waitForResult(ros::Duration(5.0));
    }
    else{
        ROS_ERROR("the number of next_joint_values should be 6");
    }
}

geometry_msgs::Pose Isometry_to_pose(const Eigen::Isometry3d& state){
    geometry_msgs::Pose pose;
    pose.position.x=state.translation()[0];
    pose.position.y=state.translation()[1];
    pose.position.z=state.translation()[2];
    Eigen::Quaterniond quaternion(state.rotation());
    Eigen::Vector4d nums=quaternion.coeffs();
    pose.orientation.x=nums[0];
    pose.orientation.y=nums[1];
    pose.orientation.z=nums[2];
    pose.orientation.w=nums[3];
    
    return pose;
}


Eigen::Isometry3d pose_to_Isometry(const geometry_msgs::Pose& pose){
    Eigen::Quaterniond q = Eigen::Quaterniond(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z).normalized();
    Eigen::Vector3d t = Eigen::Vector3d(pose.position.x,pose.position.y,pose.position.z);
    Eigen::Isometry3d state = Eigen::Isometry3d::Identity();

    state.rotate(q.toRotationMatrix());
    state.pretranslate(t);
    return state;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "manipulator_control");
    ros::NodeHandle nh;
    manipulator_state current_manipulator_state;
    ros::Subscriber manipulator_sub = nh.subscribe("manipulator_controller/state", 1000, &manipulator_state::manipulator_Callback, &current_manipulator_state);
    base_state current_base_state;
    ros::Subscriber base_sub = nh.subscribe("odom",1000,&base_state::base_Callback, &current_base_state);
    ros::Publisher start_base_pub = nh.advertise<std_msgs::Bool>("start_base", 1000);
    ros::Publisher manipulator_pose_pub = nh.advertise<geometry_msgs::Pose>("manipulator_state", 1000);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client("manipulator_controller/follow_joint_trajectory", true);
    ROS_INFO("waiting for server");
    action_client.waitForServer();
    ROS_INFO("server detected!");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    geometry_msgs::Pose current_cartisian_pose;

    //init the manipualtor using init pose
    geometry_msgs::Pose init_pose;
    init_pose.position.x=0.2;
    init_pose.position.y=0.3;
    init_pose.position.z=1;
    init_pose.orientation.x=0.707;
    init_pose.orientation.y=0;
    init_pose.orientation.z=0;
    init_pose.orientation.w=0.707;
    Eigen::Isometry3d init_state=pose_to_Isometry(init_pose);
    //compute ik
    double timeout = 0.1;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, init_state, timeout);
    std::vector<double> init_joint_values;
    if (found_ik){
        kinematic_state->copyJointGroupPositions(joint_model_group, init_joint_values);
        manipulator_joint_init(action_client,init_joint_values);
    }
    else{
        ROS_ERROR("the given init pose is not availebal");
        exit(1);
    }
    trajectory tra;

    std::ofstream out("state.txt");

    ros::Rate loop_rate(50);
    double start_time = ros::Time::now().toSec();
    while(ros::ok()){
        std_msgs::Bool start_base;
        start_base.data=true;
        start_base_pub.publish(start_base);

        //forward kinematics
        std::vector<double> joint_v(current_manipulator_state.current_joint_values,current_manipulator_state.current_joint_values+6);
        //double v[6]={0,-0.5,-0.5,0,0,1};
        //std::vector<double> joint_v(v,v+6);
        kinematic_state->setJointGroupPositions(joint_model_group,joint_v);
        const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("wrist_3_link");
        geometry_msgs::Pose current_end_effector_pose = Isometry_to_pose(end_effector_state);
        manipulator_pose_pub.publish(current_end_effector_pose);


        out<<ros::Time::now().toSec()-start_time<<" "<<current_base_state.base_position[0]<<" "<<current_base_state.base_position[1]<<" "<<current_base_state.base_position[2]
            <<" "<<current_end_effector_pose.position.x<<" "<<current_end_effector_pose.position.y<<" "<<current_end_effector_pose.position.z
            <<" "<<current_end_effector_pose.orientation.w<<" "<<current_end_effector_pose.orientation.x<<" "<<current_end_effector_pose.orientation.y
            <<" "<<current_end_effector_pose.orientation.z<<"\r\n";

        #ifdef DEBUG
            ROS_INFO("current cartisian position: x: %f, y: %f, z: %f",current_end_effector_pose.position.x,current_end_effector_pose.position.y,current_end_effector_pose.position.z);
        #endif

        //get jacobian
        kinematic_state->getJacobian(joint_model_group,
                                       kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                       reference_point_position, jacobian);
        
        //get desired pose and velocity
        geometry_msgs::Pose desired_mm_end_effector_pose=tra.desired_trajectory_pose(ros::Time::now().toSec()-start_time);
        std::vector<double> desired_mm_cartisian_velocity=tra.desired_trajectory_velocity(ros::Time::now().toSec()-start_time);

        std::vector<double> desired_manipulator_cartisian_velocity=compute_desired_manipulator_cartisian_velocity(current_end_effector_pose,
                                                                                                                    current_base_state.base_position,
                                                                                                                    desired_mm_cartisian_velocity,
                                                                                                                    current_base_state.base_vel);
        geometry_msgs::Pose desired_manipulator_end_pose=compute_desired_manipulator_end_pose(desired_mm_end_effector_pose,current_base_state.base_position);
        //compute next joint velocity
        std::vector<double> joint_velocity=compute_joint_velocity(desired_manipulator_cartisian_velocity,
                                                                    current_end_effector_pose,
                                                                    desired_manipulator_end_pose,
                                                                    jacobian);

        //compute next joint position
        std::vector<double> next_joint_position=compute_next_joint_position(current_manipulator_state.current_joint_values,joint_velocity);

        //send current action goal
        manipulator_send_goal(action_client,next_joint_position);
        #ifdef DEBUG_TIME
            ROS_INFO("current time : %f",ros::Time::now().toSec()-start_time);
        #endif
        ros::spinOnce();
        loop_rate.sleep();
    }
    out.close();

}