#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from mobile_manipulator.manipulator import manipulator
from mobile_manipulator.trajectory import translate_trajectory
import copy
import sys
import numpy as np



class sample_velocity_planner():
    def __init__(self,tra_path='/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory_goal_func.txt'):
        # Linear velocity limits
        self.MAX_X_VEL = 1.0     # ms^(-1) max speed of each wheel
        self.MAX_X_ACC = 0.3     # ms^(-2) max rate we can change speed of each wheel
        self.MAX_Y_VEL = 1.0     # ms^(-1) max speed of each wheel
        self.MAX_Y_ACC = 0.3     # ms^(-2) max rate we can change speed of each wheel
        # Angular velocity limits
        self.MAX_THETA_VEL = 0.5
        self.MAX_THETA_ACC = 0.3
        # Current linear velocity and angular velocity
        self.current_velocity=Twist()
        # Current positions
        self.current_position=Pose2D()
        # Parameters for prediction trajectory
        self.dt=0.1
        self.vel_resolution=0.01
        self.current_time=0.0
        self.manipulator=manipulator()
        self.trajectory=translate_trajectory(y_vel=0.2)
        self.current_all_sampled_vel=[]
        self.work_space_max_R=1.3
        self.work_space_min_r=0.2
        self.simulation_time=20

        #elf.planned_base_tra=[]
        self.tra_path=tra_path
        with open(self.tra_path,'w') as f:
            f.write('planned vase trajectory: time x y theta x_vel y vel theta_vel'+'\r\n')
    
    def quaternion_to_euler(self,given_oriantation):
        '''
        transfor the given orientation into the ruler angle,return R,P,Y
        '''
        R=math.atan2(2*(given_oriantation.w*given_oriantation.x+given_oriantation.y*given_oriantation.z),1-2*(math.pow(given_oriantation.x,2)+math.pow(given_oriantation.y,2)))
        P=math.asin(2*(given_oriantation.w*given_oriantation.y-given_oriantation.x*given_oriantation.z))
        Y=math.atan2(2*(given_oriantation.w*given_oriantation.z+given_oriantation.x*given_oriantation.y),1-2*(math.pow(given_oriantation.y,2)+math.pow(given_oriantation.z,2)))
        return R,P,Y

    def euler_to_quaternion(self,R,P,Y):
        '''
        transfor the given ruler angle to orientation ,return orientation
        '''
        p=Pose()
        p.orientation.w=math.cos(R/2)*math.cos(P/2)*math.cos(Y/2)+math.sin(R/2)*math.sin(P/2)*math.sin(Y/2)
        p.orientation.x=math.sin(R/2)*math.cos(P/2)*math.cos(Y/2)-math.cos(R/2)*math.sin(P/2)*math.sin(Y/2)
        p.orientation.y=math.cos(R/2)*math.sin(P/2)*math.cos(Y/2)+math.sin(R/2)*math.cos(P/2)*math.sin(Y/2)
        p.orientation.z=math.cos(R/2)*math.cos(P/2)*math.sin(Y/2)-math.sin(R/2)*math.sin(P/2)*math.cos(Y/2)
        return p.orientation
    
    def sample_velocity(self):
        '''this function sample velocity,return a list that contains all posible velocities'''
        max_x=min(self.MAX_X_VEL,self.current_velocity.linear.x+self.MAX_X_ACC*self.dt)
        max_y=min(self.MAX_Y_VEL,self.current_velocity.linear.y+self.MAX_Y_ACC*self.dt)
        max_theta=min(self.MAX_THETA_VEL,self.current_velocity.angular.z+self.MAX_THETA_ACC*self.dt)

        min_x=max(-self.MAX_X_VEL,self.current_velocity.linear.x-self.MAX_X_ACC*self.dt)
        min_y=max(-self.MAX_Y_VEL,self.current_velocity.linear.y-self.MAX_Y_ACC*self.dt)
        min_theta=max(-self.MAX_THETA_VEL,self.current_velocity.angular.z-self.MAX_THETA_ACC*self.dt)

        sample_vel_result=[]
        for i in range(int((max_x-min_x)/self.vel_resolution)):
            for j in range(int((max_y-min_y)/self.vel_resolution)):
                for k in range(int((max_theta-min_theta)/self.vel_resolution)):
                    cur_sam_vel=Twist()
                    cur_sam_vel.linear.x=min_x+i*self.vel_resolution
                    cur_sam_vel.linear.y=min_y+j*self.vel_resolution
                    cur_sam_vel.angular.z=min_theta+k*self.vel_resolution
                    sample_vel_result.append(copy.deepcopy(cur_sam_vel))
        self.current_all_sampled_vel=sample_vel_result[:]
        return sample_vel_result

    def predict_position(self,vel_posible):
        '''this function predict the posible position after delta time using posible velocity,return predicted x,y,theta position'''
        new_pos=Pose2D()
        new_pos.x=self.current_position.x+vel_posible.linear.x*self.dt*math.cos(self.current_position.theta)-vel_posible.linear.y*self.dt*math.sin(self.current_position.theta)
        new_pos.y=self.current_position.y+vel_posible.linear.y*self.dt*math.cos(self.current_position.theta)+vel_posible.linear.x*self.dt*math.sin(self.current_position.theta)
        new_pos.theta=self.current_position.theta+vel_posible.angular.z*self.dt

        next_pos=Pose2D()
        next_pos.theta=new_pos.theta+vel_posible.angular.z*self.dt
        next_pos.x=new_pos.x+vel_posible.linear.x*self.dt*math.cos(new_pos.theta)-vel_posible.linear.y*self.dt*math.sin(new_pos.theta)
        next_pos.y=new_pos.y+vel_posible.linear.y*self.dt*math.cos(new_pos.theta)+vel_posible.linear.x*self.dt*math.sin(new_pos.theta)

        return new_pos,next_pos

    def compute_goal_function(self,current_predicted_position,next_predicted_position,last_desired_pose,current_desired_pose,next_desired_pose):
        '''generate the goal func for every sampled velocity,return a float as goal func'''

        current_desired_mani_pose=self.compute_desired_manipulator_pose(current_predicted_position,current_desired_pose)
        next_desired_mani_pose=self.compute_desired_manipulator_pose(next_predicted_position,next_desired_pose)

        current_work_space_bias=self.compute_min_dustance_to_boundary(current_desired_mani_pose)
        next_work_space_bias=self.compute_min_dustance_to_boundary(next_desired_mani_pose)

        current_manipulability=self.manipulator.compute_manipulability(current_desired_mani_pose)
        next_manipulability=self.manipulator.compute_manipulability(next_desired_mani_pose)
            
        vel_x_bias=0.0
        vel_y_bias=0.0
        constance_x=(self.MAX_X_VEL*self.dt+9*current_desired_pose.position.x-last_desired_pose.position.x)/10
        constance_y=(self.MAX_Y_VEL*self.dt+9*current_desired_pose.position.y-last_desired_pose.position.y)/10
        if current_predicted_position.x-self.current_position.x < current_desired_pose.position.x-last_desired_pose.position.x:
            vel_x_bias=1+1*((current_predicted_position.x-self.current_position.x)-(current_desired_pose.position.x-last_desired_pose.position.x))/2/constance_x
        else:
            vel_x_bias=1-1*((current_predicted_position.x-self.current_position.x)-(current_desired_pose.position.x-last_desired_pose.position.x))/2/constance_x
        if current_predicted_position.y-self.current_position.y < current_desired_pose.position.y-last_desired_pose.position.y:
            vel_y_bias=1+1*((current_predicted_position.y-self.current_position.y)-(current_desired_pose.position.y-last_desired_pose.position.y))/2/constance_y
        else:
            vel_y_bias=1-1*((current_predicted_position.y-self.current_position.y)-(current_desired_pose.position.y-last_desired_pose.position.y))/2/constance_y
        velocity_bias=vel_x_bias+vel_y_bias

        if current_work_space_bias>0 and current_manipulability > 0.003:
            goal_function=0.5*current_work_space_bias + 0.2*next_work_space_bias + current_manipulability + 0.5*next_manipulability+0.2*velocity_bias
            return goal_function
        else:
            return 0


    
    def compute_desired_manipulator_pose(self,base_position,desired_mm_pose):
        '''generate the desired manipulator pose according to the current base position and desired mobole manipulator pose'''

        desired_mani_world_x=desired_mm_pose.position.x-base_position.x
        desired_mani_world_y=desired_mm_pose.position.y-base_position.y
        mani_length=math.sqrt(desired_mani_world_x*desired_mani_world_x+desired_mani_world_y*desired_mani_world_y)
        alpha=math.atan2(desired_mani_world_y,desired_mani_world_x)-base_position.theta

        desired_mani_pose=Pose()
        desired_mani_pose.position.x=mani_length*math.cos(alpha)
        desired_mani_pose.position.y=mani_length*math.sin(alpha)
        desired_mani_pose.position.z=desired_mm_pose.position.z

        R,P,Y=self.quaternion_to_euler(desired_mm_pose.orientation)
        Y-=base_position.theta
        desired_mani_pose.orientation=self.euler_to_quaternion(R,P,Y)

        return desired_mani_pose

    def compute_min_dustance_to_boundary(self,desired_mani_pose):
        ''' compute the min distance from the desired pose to work space boundary'''

        distance_to_base=math.sqrt(desired_mani_pose.position.x*desired_mani_pose.position.x+
                                    desired_mani_pose.position.y*desired_mani_pose.position.y+
                                    desired_mani_pose.position.z*desired_mani_pose.position.z)
        distance_to_max_R=self.work_space_max_R - distance_to_base
        distance_to_min_r=math.sqrt(desired_mani_pose.position.x**2+desired_mani_pose.position.y**2)-self.work_space_min_r

        return min(distance_to_max_R,distance_to_min_r)

    def plan_whole_trajectory(self):
        '''plan the whole trajectory'''
        with open(self.tra_path,'a') as f:
            f.write('0 0 0 0 0 0 0'+'\r\n')
        for i in range(int(self.simulation_time/self.dt)):
            cur_time=self.current_time+self.dt
            print('======current time:'+str(cur_time)+'========total time:'+str(self.simulation_time)+'========='+str(cur_time/self.simulation_time*100)+'% finished')
            self.plan_one_point(cur_time)




    def plan_one_point(self,time):
        '''sample and plan for the desired trajectory at time time'''

        last_desired_mm_pose=self.trajectory.compute_desired_pose(time-self.dt)
        current_desired_mm_pose=self.trajectory.compute_desired_pose(time)
        next_desired_mm_pose=self.trajectory.compute_desired_pose(time+self.dt)

        self.current_all_sampled_vel=[]
        self.sample_velocity()
        max_goal_func=0
        cur_vel=Twist()
        cur_pos=Pose2D()
        for i in range(len(self.current_all_sampled_vel)):
            current_predicted_position, next_predicted_position=self.predict_position(self.current_all_sampled_vel[i])
            cur_goal_func=self.compute_goal_function(current_predicted_position,next_predicted_position,last_desired_mm_pose,current_desired_mm_pose,next_desired_mm_pose)
            if cur_goal_func > max_goal_func:
                max_goal_func=cur_goal_func
                cur_vel=self.current_all_sampled_vel[i]
                cur_pos=current_predicted_position
        print('cur goal func: '+str(max_goal_func))
        if max_goal_func < 0.0001:
            sys.stderr.write(' NO solution to this point at time'+str(time)+'\r\n')
            exit()
        self.current_velocity=cur_vel
        self.current_position=cur_pos
        self.current_time=time
        with open(self.tra_path,'a') as f:
            f.write(str(time)+' '+str(cur_pos.x)+' '+str(cur_pos.y)+' '+str(cur_pos.theta)+' '+str(cur_vel.linear.x)+' '+str(cur_vel.linear.y)+' '+str(cur_vel.angular.z)+'\r\n')
        #self.planned_base_tra.append(copy.deepcopy([time,cur_pos]))
        


