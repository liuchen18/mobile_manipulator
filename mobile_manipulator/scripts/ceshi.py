#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Pose,Pose2D
from mobile_manipulator.quaternion_euler import *
from mobile_manipulator.trajectory import translate_trajectory

def main():
    current_base_pose=Pose2D()
    current_mani_pose=Pose()
    desired_mm_pose=Pose()
    tra=translate_trajectory(y_vel=0.2)
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/state.txt','r') as f:
        data=list(map(float,f.readlines()[-1].split(' ')))
        desired_mm_pose=tra.compute_desired_pose(data[0])
        current_base_pose.x=data[1]
        current_base_pose.y=data[2]
        current_base_pose.theta=data[3]
        current_mani_pose.position.x=data[4]
        current_mani_pose.position.y=data[5]
        current_mani_pose.position.z=data[6]
        current_mani_pose.orientation.w=data[7]
        current_mani_pose.orientation.x=data[8]
        current_mani_pose.orientation.y=data[9]
        current_mani_pose.orientation.z=data[10]
    
    current_mm_pose=Pose()

    length=math.sqrt(current_mani_pose.position.x**2+current_mani_pose.position.y**2)
    alpha=math.atan2(current_mani_pose.position.y,current_mani_pose.position.x)

    current_mm_pose.position.x=current_base_pose.x+length*math.cos(alpha+current_base_pose.theta)
    current_mm_pose.position.y=current_base_pose.y+length*math.sin(alpha+current_base_pose.theta)
    current_mm_pose.position.z=current_mani_pose.position.z

    R,P,Y=quaternion_to_euler(current_mani_pose.orientation)
    Y+=current_base_pose.theta
    current_mm_pose.orientation=euler_to_quaternion(R,P,Y)
    print('desired_mm_pose:')
    print(desired_mm_pose)
    print('current_mm_pose:')
    print(current_mm_pose)

    c_r,c_p,c_y=quaternion_to_euler(current_mm_pose.orientation)
    d_r,d_p,d_y=quaternion_to_euler(desired_mm_pose.orientation)
    print('current_Y_error: '+str(d_y-c_y))
    
if __name__ == '__main__':
    main()
