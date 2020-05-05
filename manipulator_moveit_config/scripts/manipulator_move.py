#!/usr/bin/env python
import rospy
import sys
import moveit_commander
from moveit_commander import MoveGroupCommander
import math
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
import numpy.matlib
import eigenpy
import copy
from moveit_msgs.srv import GetPositionFK,GetPositionFKRequest,GetPositionFKResponse
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped,Pose


class manipulator_control_class():
    def __init__(self):
        self.start_to_sub=False
        rospy.Subscriber('/manipulator_controller/state',JointTrajectoryControllerState,self.callback)
        self.compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        rospy.loginfo('move_group name: manipulator')
        self.current_joint_values=self.group.get_current_joint_values()
        self.current_jacobian_matrix=np.zeros((6,6))
        self.current_jacobian_matrix_det=0
        self.current_end_effector_pose=PoseStamped()
        self.current_state=JointTrajectoryControllerState()
        self.start_time=rospy.get_time()
        self.got_msg=False
        self.current_cartisian_velocity=np.zeros((6,1))
        self.move_type=1 #1 means desired, 2 means noise_added, 3 means controlled
        self.rate_hz=50.0
        self.is_control=False
        self.active_joints=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
        self.move_group_link_names=['first_link','shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link']
        self.K=[0.4,0.4,1,0.6,0.6,0.6]

    def callback(self,msg):
        if self.start_to_sub==True:
            self.current_state=msg
            self.current_joint_values=self.group.get_current_joint_values()
            self.current_jacobian_matrix,self.current_jacobian_matrix_det=self.get_current_jacobian()
            self.current_cartisian_velocity=self.compute_current_cartisian_velocity()
            self.got_msg=True
            self.current_end_effector_pose=self.compute_f_k(self.compute_fk,self.current_joint_values)
            #print(self.current_end_effector_pose)

    def go_to_goal_joint(self,joint1,joint2,joint3,joint4,joint5,joint6):
        rospy.loginfo('trying to go to the goal joint value')
        joint_goal=self.group.get_current_joint_values()
        joint_goal[0] = joint1
        joint_goal[1] = joint2
        joint_goal[2] = joint3
        joint_goal[3] = joint4
        joint_goal[4] = joint5
        joint_goal[5] = joint6
        self.group.go(joint_goal, wait=True)
        self.group.stop()
        return True

    def go_to_home_joint(self):
        rospy.loginfo('trying to go to the home joint value')
        joint_goal=self.group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0
        self.group.go(joint_goal, wait=True)
        self.group.stop()
        return True

    def get_current_jacobian(self):
        '''
        compute current jacobian matrix and the det of the jacobian matrix. the self.current_jacobian_matrix and self.current_jacobian_matrix_det are updated in real time
        '''
        current_joint_values=self.current_joint_values
        current_jacobian_matrix_m=numpy.matlib.zeros((6,6))
        current_jacobian_matrix_m=self.group.get_jacobian_matrix(current_joint_values)
        current_jacobian_matrix_det=np.linalg.det(current_jacobian_matrix_m)
        current_jacobian_matrix=np.asarray(current_jacobian_matrix_m)
        return current_jacobian_matrix,current_jacobian_matrix_det

    def compute_current_cartisian_velocity(self):
        '''
        compute current cartisian velocity,size(6,1),the self.current_cartisian_velocity is updated in real time
        '''
        current_joint_velocity=np.zeros((6,1))
        for i in range(6):
            current_joint_velocity[i,0]=self.current_state.desired.velocities[i]
        current_jacobian_matrix=self.current_jacobian_matrix
        current_cartisian_velocity=np.dot(current_jacobian_matrix,current_joint_velocity)
        #linear_velocity=math.sqrt(math.pow(current_cartisian_velocity[0,0],2)+math.pow(current_cartisian_velocity[1,0],2))
        return current_cartisian_velocity
    
    def compute_jacobian_matrix(self,joint_value):
        '''
        compute the jacobian matrix of the given joint value,the given joint value should be (1,6) array
        '''
        jacobian_matrix_m=numpy.matlib.zeros((6,6))
        jacobian_matrix_m=self.group.get_jacobian_matrix(joint_value)
        jacobian_matrix=np.asarray(jacobian_matrix_m)
        return jacobian_matrix

    def compute_desired_velocity(self,time,latter_joint_value,latter_joint_velocity):
        '''
        compute bpth the desired joint velocity and the desired cartisian velocity,return size(6,1),desired joint value size(1,6)
        '''
        desired_cartisian_velocity=np.zeros((6,1))
        desired_cartisian_velocity[0,0]=-0.167*math.sin(0.5*time)
        desired_cartisian_velocity[1,0]=0.167*math.cos(0.5*time)
        desired_cartisian_velocity[5,0]=0.5
        desired_joint_value=[]
        for i in range(6):
            desired_joint_value.append(latter_joint_value[i]+latter_joint_velocity[i,0]/self.rate_hz)
        jacobian_matrix=self.compute_jacobian_matrix(desired_joint_value)
        #current_cartisian_velocity=np.dot(jacobian_matrix,latter_joint_velocity)
        jacobian_matrix_det=np.linalg.det(jacobian_matrix)
        try:
            if jacobian_matrix_det > 0.001:
                jacobian_matrix_inv=np.linalg.inv(jacobian_matrix)
                desired_joint_velocity=np.dot(jacobian_matrix_inv,desired_cartisian_velocity)
                return desired_cartisian_velocity,desired_joint_velocity,desired_joint_value
            else:
                exit()
        except:
            rospy.logerr('the jacobian matrix is singular')
            exit()

    def add_noise(self,desired_joint_velocity,time):
        '''
        add noise to the desired joint velocity, the noise function is defined below , return size(6,1)
        '''
        joint_velocity_command=copy.deepcopy(desired_joint_velocity)
        joint_velocity_noise=[0.01*math.sin(0.4*time),
                                0.01*math.sin(0.4*time),
                                0.01*math.sin(0.4*time),
                                0.01*math.sin(0.4*time),
                                0.01*math.sin(0.4*time),
                                0.01*math.sin(0.4*time)]
        for i in range(6):
            joint_velocity_command[i,0]=desired_joint_velocity[i,0]+joint_velocity_noise[i]
        #print('desired joint velocity:'+str(desired_joint_velocity.reshape((1,6))))
        #print('noise_added joint velocity'+str(joint_velocity_command.reshape((1,6))))
        return joint_velocity_command

    def controlled_command(self,desired_cartisian_velocity,desired_cartisian_pose,current_cartisian_pose):
        '''
        compute the controlled velocity command of each joint according the desired joint velocity and the given control strategy, return size(6,1)
        '''
        motion_error=self.compute_cartisian_error(desired_cartisian_pose,current_cartisian_pose)
        #print(motion_error)
        controlled_cartisian_velocity=np.zeros((6,1))
        for i in range(6):
            controlled_cartisian_velocity[i][0]=desired_cartisian_velocity[i][0] + self.K[i]*motion_error[i]
        print(controlled_cartisian_velocity.reshape((1,6)))
        print(desired_cartisian_velocity.reshape((1,6)))
        print(motion_error)
        print(str(desired_cartisian_pose.pose.position.z)+' ' + str(current_cartisian_pose.pose.position.z))
        print('*' * 70)
        jacobian_matrix=self.compute_jacobian_matrix(self.current_joint_values)
        jacobian_matrix_inv=np.linalg.inv(jacobian_matrix)
        controlled_joint_velocity=np.dot(jacobian_matrix_inv,controlled_cartisian_velocity)

        return controlled_joint_velocity

    def compute_cartisian_error(self,desired_cartisian_pose,current_cartisian_pose):
        '''
        compute the motion error using the desired and current according the error defination in the paper
        '''
        motion_error=[0 for i in range(6)]
        motion_error[0]=desired_cartisian_pose.pose.position.x-current_cartisian_pose.pose.position.x
        motion_error[1]=desired_cartisian_pose.pose.position.y-current_cartisian_pose.pose.position.y
        motion_error[2]=desired_cartisian_pose.pose.position.z-current_cartisian_pose.pose.position.z
        qc=[[current_cartisian_pose.pose.orientation.x],[current_cartisian_pose.pose.orientation.y],[current_cartisian_pose.pose.orientation.z]]
        nc=current_cartisian_pose.pose.orientation.w
        qd=[[desired_cartisian_pose.pose.orientation.x],[desired_cartisian_pose.pose.orientation.y],[desired_cartisian_pose.pose.orientation.z]]
        nd=desired_cartisian_pose.pose.orientation.w
        qccha=[[0,-qc[2][0],qc[1][0]],
            [qc[2][0],0,-qc[0][0]],
            [-qc[1][0],qc[0][0],0]]
        qd_copy=copy.deepcopy(qd)
        for i in range(3):
            qd_copy[i][0]=qd_copy[i][0]*nc
            qc[i][0]=qc[i][0]*nd
        dot=np.dot(qccha,qd)
        jiaodu=[0,0,0]
        for i in range(3):
            jiaodu[i]=qd_copy[i][0]-qc[i][0]+dot[i][0]
        motion_error[3]=jiaodu[0]
        motion_error[4]=jiaodu[1]
        motion_error[5]=jiaodu[2]
        return motion_error
        

    def compute_f_k(self,compute_fk,joint_values):
        '''
        compute the forward kinematics of the given joint values,the given joint values should be an array of (1,6),this function returns the fk response with the type of PoseStamped
        '''
        fk_request=GetPositionFKRequest()
        links=self.move_group_link_names
        fk_request.fk_link_names=links
        state=RobotState()
        joint_names=self.active_joints
        state.joint_state.name=joint_names
        state.joint_state.position=joint_values
        fk_request.robot_state=state
        fk_response=compute_fk(fk_request)
        end_effector_pose=fk_response.pose_stamped[len(fk_response.pose_stamped)-1]
        return end_effector_pose

    def quaternion_to_euler(self,given_oriantation):
        '''
        transfor the given orientation into the ruler angle,return R,P,Y
        '''
        R=math.atan2(2*(given_oriantation.w*given_oriantation.x+given_oriantation.y*given_oriantation.z),1-2*(math.pow(given_oriantation.x,2)+math.pow(given_oriantation.y,2)))
        P=math.asin(2*(given_oriantation.w*given_oriantation.y-given_oriantation.x*given_oriantation.z))
        Y=math.atan2(2*(given_oriantation.w*given_oriantation.z+given_oriantation.x*given_oriantation.y),1-2*(math.pow(given_oriantation.y,2)+math.pow(given_oriantation.z,2)))
        #print('R:'+str(R))
        #print('P:'+str(P))
        #print('Y:'+str(Y))
        return R,P,Y




def main(move_type):
    eigenpy.switchToNumpyMatrix()
    cmd_pub=rospy.Publisher('/manipulator_controller/command',JointTrajectory,queue_size=10)    
    rospy.loginfo("starting to publish commands")
    '''
    #using moveit to control the arm
    manipulator_control=manipulator_control_class()
    rate=rospy.Rate(10)
    manipulator_control.go_to_goal_joint(math.pi/2,0,0,0,0,0)
    manipulator_control.go_to_home_joint()
    '''
    '''
    # controller type: velocity_controllers/JointVelocityController
    while not rospy.is_shutdown():
        command=Float64MultiArray()
        command.data=[0,0,0,0,0,0]
        if rospy.get_time()-start_time < 5:
            command.data[2]=2
        elif rospy.get_time()-start_time < 10:
            command.data[2]=-2
        else:
            exit()
        cmd_pub.publish(command)
        rate.sleep()    
    '''
    #controller type : velocity_controllers/JointTrajectoryController
    rospy.wait_for_service('/compute_fk')
    manipulator_control=manipulator_control_class()
    manipulator_control.start_to_sub=True
    while rospy.get_time() == 0:              #wait until the rospy gets the system time
        continue
    manipulator_control.start_time=rospy.get_time()
    is_first=True # to help test the time
    if move_type == 'desired':
        manipulator_control.move_type=1
    elif move_type=='noise_added':
        manipulator_control.move_type=2
    elif move_type=='controlled':
        manipulator_control.move_type=3
    else:
        rospy.logerr('invalid arg!plz input desired or noise_added or controlled')
        exit()
    manipulator_control.is_control=True          #false means the command will not be published
    if manipulator_control.is_control == False:
        rospy.loginfo('the trajectory command will not be published')
    rate=rospy.Rate(manipulator_control.rate_hz)
    latter_joint_value=[0,-0.3,-2,0,1,0]         #init joint values of the manipulator
    latter_joint_velocity=np.zeros((6,1))        #init joint velocity of the manipulator
    while not rospy.is_shutdown():
        if manipulator_control.got_msg==True:
            time = rospy.get_time()-manipulator_control.start_time
            if is_first == True and time > 2:
                rospy.logerr('something wrong with the timer, plz restart the node')
                exit()
            is_first=False
            command=JointTrajectory()
            command.joint_names=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint']
            point=JointTrajectoryPoint()
            desired_cartisian_velocity,desired_joint_velocity,desired_joint_value = manipulator_control.compute_desired_velocity(time,latter_joint_value,latter_joint_velocity)
            latter_joint_value = copy.deepcopy(desired_joint_value)
            latter_joint_velocity=copy.deepcopy(desired_joint_velocity)
            #move type of the manipulator
            if manipulator_control.move_type == 1: # the desired trajectory
                joint_velocity_command=copy.deepcopy(desired_joint_velocity)
            elif manipulator_control.move_type ==2: # the desired trajectory with velocity noise
                joint_velocity_command=manipulator_control.add_noise(desired_joint_velocity,time)
            elif manipulator_control.move_type ==3: # the controlled trajectory with velocity noise
                desired_cartisian_pose=manipulator_control.compute_f_k(manipulator_control.compute_fk,desired_joint_value)
                #this desired_cartisian_pose is a PoseStamped
                controlled_joint_velocity=manipulator_control.controlled_command(desired_cartisian_velocity,desired_cartisian_pose,manipulator_control.current_end_effector_pose)
                joint_velocity_command=manipulator_control.add_noise(controlled_joint_velocity,time)
            #publishs the joint velocity command
            for i in range(6):
                point.positions.append(manipulator_control.current_state.desired.positions[i]+joint_velocity_command[i,0])
                point.velocities.append(joint_velocity_command[i,0])
            point.time_from_start=rospy.Duration(1.0)
            command.points.append(point)
            command.header.seq=command.header.seq+1
            command.header.stamp=rospy.Time.now()
            if manipulator_control.is_control == True:
                cmd_pub.publish(command)
        rate.sleep()
        
        



if __name__ == '__main__':
    rospy.init_node('manipulator_move',anonymous=True)
    rospy.loginfo(sys.argv)
    if len(sys.argv) < 2:
        rospy.loginfo('-------moving type : desired---------')
        main('desired')
    else:
        if sys.argv[1]=='noise_added':
            rospy.loginfo('-------moving type : noise_added---------')
            main('noise_added')
        elif sys.argv[1] == 'controlled':
            rospy.loginfo('-------moving type : controlled---------')
            main('controlled')
        elif sys.argv[1] == 'desired':
            rospy.loginfo('-------moving type : desired---------')
            main('desired')
        else:
            rospy.logerr('the input move_type is invalid, plz input desired or noise_added or controlled')
            exit()