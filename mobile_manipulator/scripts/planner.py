#!/usr/bin/env python
import rospy
import math
import moveit_commander
from moveit_commander import MoveGroupCommander
import numpy as np
from moveit_msgs.srv import GetPositionIK,GetPositionIKRequest,GetPositionIKResponse
from gazebo_msgs.srv import GetLinkState,GetLinkStateRequest,GetLinkStateResponse
from geometry_msgs.msg import Pose
import numpy.matlib
import eigenpy
from geometry_msgs.msg import Twist
import PyKDL as kdl
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class manipulator():
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    def compute_inverse_kinematics(self,end_effector_pose):
        '''
        compute the inverse kinematics of the given end effector pose,return joint values,end_effector_pose should be a pose
        '''
        request=GetPositionIKRequest()
        request.ik_request.group_name=self.group_name
        request.ik_request.ik_link_name = "wrist_3_link"
        request.ik_request.attempts = 10000
        request.ik_request.pose_stamped.header.frame_id = "first_link"
        request.ik_request.pose_stamped.pose.position.x = end_effector_pose.position.x
        request.ik_request.pose_stamped.pose.position.y = end_effector_pose.position.y
        request.ik_request.pose_stamped.pose.position.z = end_effector_pose.position.z
        request.ik_request.pose_stamped.pose.orientation.x = end_effector_pose.orientation.x
        request.ik_request.pose_stamped.pose.orientation.y = end_effector_pose.orientation.y
        request.ik_request.pose_stamped.pose.orientation.z = end_effector_pose.orientation.z
        request.ik_request.pose_stamped.pose.orientation.w = end_effector_pose.orientation.w
        #print(request)
        ik_response=self.compute_ik(request)
        #print(ik_response)
        joint_value=ik_response.solution.joint_state.position
        joint_values=[]
        if len(joint_value) < 6:
            #rospy.logerr('the given end_effector_pose has no results')
            return joint_values
        else:
            for i in range(len(joint_value)):
                joint_values.append(joint_value[i])
            #print(joint_value)
            return joint_values


    def compute_jacobian_matrix(self,joint_value):
        '''
        compute the jacobian matrix of the given joint value,the given joint value should be (1,6) array
        '''
        if len(joint_value) < 6:
            r=[]
            return []
        else:
            jacobian_matrix_m=numpy.matlib.zeros((6,6))
            jacobian_matrix_m=self.group.get_jacobian_matrix(joint_value)
            jacobian_matrix=np.asarray(jacobian_matrix_m)
            return jacobian_matrix

    def compute_manipulability(self,end_effector_pose):
        '''
        compute the manipulability of the given end_effector_pose,return manipulability
        '''
        joint_value=self.compute_inverse_kinematics(end_effector_pose)
        jacobian_matrix=self.compute_jacobian_matrix(joint_value)
        if jacobian_matrix != []:
            manipulability=math.sqrt(np.linalg.det(np.dot(jacobian_matrix,np.transpose(jacobian_matrix))))
        else:
            manipulability=0
        return manipulability
        

class planner():
    def __init__(self):
        self.time_duration=0.5    #this means the min time duration when planning
        self.sim_time=30          #this means the total simulate time
        self.x_acc=0.5
        self.y_acc=0.5
        self.theta_acc=0.2
        self.x_vel=1.0
        self.y_vel=1.0
        self.theta_vel=0.0 #mobile base velocity and acceleration
        self.last_base_vel=Twist() # the velocity of the last time duration
        self.last_base_x=0
        self.last_base_y=0
        self.last_base_theta=0 #the position of the last time duration
        self.min_x_dis=0.01
        self.min_y_dis=0.01
        self.min_theta_dis=0.02 #grid size
        self.arm=manipulator()
        self.way_points=[]

    def desired_end_effector_pose(self,time):
        '''
        give the global desired pose at the time according the desired trajectory,return a pose
        '''
        desired_pose=Pose()
        desired_pose.position.x=0.2
        desired_pose.position.y=time*0.1+0.3
        desired_pose.position.z=1.0
        desired_pose.orientation.x=0.707
        desired_pose.orientation.y=0.0
        desired_pose.orientation.z=0.0
        desired_pose.orientation.w=0.707
        return desired_pose

    def compute_position_area(self):
        '''
        compute the avaible area of the mobile base at this time duration
        '''
        if self.last_base_vel.linear.x >= self.x_vel:
            max_x_dis=self.last_base_vel.linear.x*self.time_duration
            min_x_dis=self.last_base_vel.linear.x*self.time_duration-0.5*self.x_acc*math.pow(self.time_duration,2)
        elif self.last_base_vel.linear.x <= -self.x_vel:
            max_x_dis=self.last_base_vel.linear.x*self.time_duration+0.5*self.x_acc*math.pow(self.time_duration,2)
            min_x_dis=self.last_base_vel.linear.x*self.time_duration
        else:
            max_x_dis=self.last_base_vel.linear.x*self.time_duration+0.5*self.x_acc*math.pow(self.time_duration,2)
            min_x_dis=self.last_base_vel.linear.x*self.time_duration-0.5*self.x_acc*math.pow(self.time_duration,2)
        if self.last_base_vel.linear.y >= self.y_vel:
            max_y_dis=self.last_base_vel.linear.y*self.time_duration
            min_y_dis=self.last_base_vel.linear.y*self.time_duration-0.5*self.y_acc*math.pow(self.time_duration,2)
        elif self.last_base_vel.linear.y <= -self.y_vel:
            max_y_dis=self.last_base_vel.linear.y*self.time_duration+0.5*self.y_acc*math.pow(self.time_duration,2)
            min_y_dis=self.last_base_vel.linear.y*self.time_duration
        else:
            max_y_dis=self.last_base_vel.linear.y*self.time_duration+0.5*self.y_acc*math.pow(self.time_duration,2)
            min_y_dis=self.last_base_vel.linear.y*self.time_duration-0.5*self.y_acc*math.pow(self.time_duration,2)
        if self.last_base_vel.angular.z >= self.theta_vel:
            max_theta_dis=self.last_base_vel.angular.z*self.time_duration
            min_theta_dis=self.last_base_vel.angular.z*self.time_duration-0.5*self.theta_acc*math.pow(self.time_duration,2)
        elif self.last_base_vel.angular.z <= -self.theta_vel:
            max_theta_dis=self.last_base_vel.angular.z*self.time_duration+0.5*self.theta_acc*math.pow(self.time_duration,2)
            min_theta_dis=self.last_base_vel.angular.z*self.time_duration
        else:
            max_theta_dis=self.last_base_vel.angular.z*self.time_duration+0.5*self.theta_acc*math.pow(self.time_duration,2)
            min_theta_dis=self.last_base_vel.angular.z*self.time_duration-0.5*self.theta_acc*math.pow(self.time_duration,2)

        return max_x_dis,min_x_dis,max_y_dis,min_y_dis,max_theta_dis,min_theta_dis

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

    def compute_grid_num(self,max_x_d,min_x_d,max_y_d,min_y_d,max_theta_d,min_theta_d):
        '''
        compute the grid num according the area and the grid size
        '''
        x_num=int(math.floor((max_x_d-min_x_d)/self.min_x_dis)+1)
        y_num=int(math.floor((max_y_d-min_y_d)/self.min_y_dis)+1)
        theta_num=int(math.floor((max_theta_d-min_theta_d)/self.min_theta_dis)+1)
        return x_num,y_num,theta_num


    def compute_max_mani_position(self,desired_pose):
        '''
        according the posible area, compute the manipulability of all the posible position 
        '''
        max_info=[]
        max_manipulability=0
        last_x=self.last_base_x
        last_y=self.last_base_y
        last_theta=self.last_base_theta
        max_x_d,min_x_d,max_y_d,min_y_d,max_theta_d,min_theta_d=self.compute_position_area()
        #print('dis: '+str(max_x_d-min_x_d)+' '+str(max_y_d-min_y_d))
        min_x_dis=self.min_x_dis
        min_y_dis=self.min_y_dis
        min_theta_dis=self.min_theta_dis
        x_num,y_num,theta_num=self.compute_grid_num(max_x_d,min_x_d,max_y_d,min_y_d,max_theta_d,min_theta_d)
        vel=Twist()
        #print('grid_num'+str(x_num)+' '+str(y_num))
        flag_y=-1
        for x in range(x_num):
            for y in range(y_num):
                for theta in range(theta_num):
                    x_dis=min_x_dis*x+min_x_d #this means the distance of the base moves along the direction
                    y_dis=min_y_dis*y+min_y_d
                    #print(y_dis)
                    #theta_dis=min_theta_dis*theta+min_theta_d
                    theta_dis=0.0 # this means theta_vel is always 0
                    global_x,global_y,global_theta=self.compute_current_base_posotion(x_dis,y_dis,theta_dis)
                    #print(global_y)

                    #compute desired pose
                    d_pose=Pose()
                    d_pose.position.x=desired_pose.position.x-global_x
                    d_pose.position.y=desired_pose.position.y-global_y
                    d_pose.position.z=desired_pose.position.z
                    #desired_pose.position.z=1
                    desired_R,desired_P,desired_Y=self.quaternion_to_euler(desired_pose.orientation)
                    desired_Y-=global_theta
                    d_pose.orientation=self.euler_to_quaternion(desired_R,desired_P,desired_Y)


                    manipulability=self.arm.compute_manipulability(d_pose)
                    #print(manipulability)
                    if max_manipulability < manipulability:
                        max_manipulability=manipulability
                        vel.linear.x=x_dis/self.time_duration
                        vel.linear.y=y_dis/self.time_duration
                        vel.angular.z=theta_dis/self.time_duration #this velocity computation method is not good, needed to be improved
                        #print(global_y)
                        max_info=[global_x,global_y,global_theta,manipulability]
                        #print(max_info)

        #print(max_info)
        return max_info,vel

    def compute_current_base_posotion(self,this_x,this_y,this_theta):
        '''
        compute the current mobile base global position,prameters are distance of the base coodinate
        '''
        x=self.last_base_x+this_x*math.cos(self.last_base_theta)-this_y*math.sin(self.last_base_theta)
        y=self.last_base_y+this_y*math.cos(self.last_base_theta)+this_x*math.sin(self.last_base_theta)
        theta=self.last_base_theta+this_theta
        return x,y,theta

    def plan_whole_trajectory(self):
        '''
        plan the whole trajectory,the base trajeacory is in self.way_points
        '''
        for t in range(int(self.sim_time/self.time_duration)):
            desired_pose=self.desired_end_effector_pose(self.time_duration*t)
            max_info,vel=self.compute_max_mani_position(desired_pose)
            if max_info == []:
                rospy.logerr('NO IK solution at this pose')
                exit()
            else:
                with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory.txt','a') as f:
                    f.write(str(max_info[0])+' '+str(max_info[1])+' '+str(max_info[2])+' '+str(max_info[3])+' '+str(t*self.time_duration)+'\r\n')
                self.way_points.append([max_info[0],max_info[1],max_info[2],t])
                self.last_base_vel=vel
                #print(self.last_base_vel)
                self.last_base_x=max_info[0]
                self.last_base_y=max_info[1]
                self.last_base_theta=max_info[2]
                print('current_position:'+str(self.last_base_x)+' '+str(self.last_base_y)+' '+str(self.last_base_theta))
        with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory.txt','a') as f:
            f.write('Done!')


def main():
    eigenpy.switchToNumpyMatrix()
    rospy.init_node('planner')
    while not rospy.get_time() == 0: #wait until the system gets the time
        continue
    rospy.wait_for_service('/compute_ik')
    '''
    #get a end effector pose to test the ik
    rospy.wait_for_service('/gazebo/get_link_state')
    get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    link_state_request=GetLinkStateRequest()
    link_state_request.link_name='wrist_3_link'
    #link_state_request.reference_frame='base_link'
    link_state_reponse=get_link_state(link_state_request)
    end_effector_pose=link_state_reponse.link_state.pose
    #print(end_effector_pose)
    #end_effector_pose.position.z-=0.1
    #end_effector_pose.position.x-=0.2
    '''
    current_planner=planner()
    rate=rospy.Rate(10)
    start_time=rospy.get_time()
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory.txt','w') as f:
        f.write('base trajectory: x y theta manipulability time \r\n')
    current_planner.plan_whole_trajectory()






if __name__ == '__main__':
    main()