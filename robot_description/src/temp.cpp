#include <Eigen/Dense>
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "temp");
    /*
    Eigen::MatrixXd mat;
    Eigen::Matrix3d m;
    m<<1,0,0,0,1,0,0,0,1;
    mat=m;
    ROS_INFO("det of m: %f",m.determinant());
    ROS_INFO("det of mat: %f", mat.determinant());
    */
    Eigen::Vector3d mm_rpy(0.0,1.0,0.5);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(mm_rpy(2),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(mm_rpy(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(mm_rpy(0),Eigen::Vector3d::UnitZ()));
 
    Eigen::Quaterniond quaternion;
    quaternion=yawAngle*pitchAngle*rollAngle;

    Eigen::Vector4d nums=quaternion.coeffs();
    std::cout<<"x: "<<nums[0]<<" y: "<<nums[1]<<" z: "<<nums[2]<<" w: "<<nums[3]<<std::endl;
    return 0;
}
