#!/usr/bin/env python
import math
from geometry_msgs.msg import Pose

class translate_trajectory():
    def __init__(self,x_vel=0,y_vel=0,theta_vel=0):
        self.x_velocity=x_vel
        self.y_velocity=y_vel
        self.theta_velocity=theta_vel

    def compute_desired_pose(self,time):
        '''
        give the global desired pose at the time according the desired trajectory,return a pose
        '''
        desired_pose=Pose()
        desired_pose.position.x=0.2+self.x_velocity*time
        desired_pose.position.y=time*self.y_velocity+0.3
        desired_pose.position.z=1.0
        desired_pose.orientation.x=0.707
        desired_pose.orientation.y=0.0
        desired_pose.orientation.z=0.0
        desired_pose.orientation.w=0.707
        return desired_pose
