#!/usr/bin/env python
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
import numpy as np
from moveit_msgs.srv import GetPositionIK,GetPositionIKRequest,GetPositionIKResponse
from gazebo_msgs.srv import GetLinkState,GetLinkStateRequest,GetLinkStateResponse
from geometry_msgs.msg import Pose
import numpy.matlib
import math
import eigenpy

class manipulator():
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        eigenpy.switchToNumpyMatrix()
    
    def compute_inverse_kinematics(self,end_effector_pose):
        '''
        compute the inverse kinematics of the given end effector pose,return joint values,end_effector_pose should be a pose
        '''
        request=GetPositionIKRequest()
        request.ik_request.group_name=self.group_name
        request.ik_request.ik_link_name = "wrist_3_link"
        request.ik_request.attempts = 1000
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