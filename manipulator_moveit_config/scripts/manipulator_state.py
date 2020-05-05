#!/usr/bin/env python
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
import eigenpy
from control_msgs.msg import JointTrajectoryControllerState
import numpy.matlib
import sys
import math
import numpy as np
from moveit_msgs.srv import GetPositionFK,GetPositionFKRequest,GetPositionFKResponse
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped



class manipulator_state_class():
    def __init__(self,move_type,write_type):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        #rospy.loginfo('move_group name: manipulator')
        self.compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        self.current_joint_values=self.group.get_current_joint_values()
        self.current_cartisian_velocity=numpy.zeros((6,1))
        self.current_joint_velocity=[]
        self.current_jacobian_matrix=numpy.zeros((6,6))
        self.current_end_effector_pose=PoseStamped()
        self.manipulator_state=JointTrajectoryControllerState()
        self.start_time=rospy.get_time()
        self.got_msg=False
        self.active_joints=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
        self.move_group_link_names=['first_link','shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link']
        self.move_type = move_type
        self.write_type = write_type
        rospy.Subscriber('manipulator_controller/state',JointTrajectoryControllerState,self.callback_sub_robot_state)

    def callback_sub_robot_state(self,state):
        self.manipulator_state=state
        self.current_joint_values=self.manipulator_state.desired.positions
        self.current_cartisian_velocity=self.get_cartisian_velocity
        self.current_joint_velocity=self.manipulator_state.desired.velocities
        self.current_end_effector_pose=self.compute_f_k(self.compute_fk,self.current_joint_values)
        r,p,y=self.quaternion_to_euler(self.current_end_effector_pose.pose.orientation)
        self.got_msg=True
        if self.move_type ==0 and self.write_type == 1:
            with open('/home/chen/ws_chen/src/mm_meta_pkg/manipulator_moveit_config/data/manipulator_state_desired.txt','a') as f:
                f.write(str(rospy.get_time()-self.start_time)+' '+str(self.current_end_effector_pose.pose.position.x)\
                        +' '+str(self.current_end_effector_pose.pose.position.y)+' '+str(self.current_end_effector_pose.pose.position.z)\
                        +' '+str(r)+' '+str(p)+' '+str(y)+'\r\n')
        if self.move_type ==1 and self.write_type == 1:
            with open('/home/chen/ws_chen/src/mm_meta_pkg/manipulator_moveit_config/data/manipulator_state_noise.txt','a') as f:
                f.write(str(rospy.get_time()-self.start_time)+' '+str(self.current_end_effector_pose.pose.position.x)\
                        +' '+str(self.current_end_effector_pose.pose.position.y)+' '+str(self.current_end_effector_pose.pose.position.z)\
                        +' '+str(r)+' '+str(p)+' '+str(y)+'\r\n')
        if self.move_type ==2 and self.write_type == 1:
            with open('/home/chen/ws_chen/src/mm_meta_pkg/manipulator_moveit_config/data/manipulator_state_controlled.txt','a') as f:
                f.write(str(rospy.get_time()-self.start_time)+' '+str(self.current_end_effector_pose.pose.position.x)\
                        +' '+str(self.current_end_effector_pose.pose.position.y)+' '+str(self.current_end_effector_pose.pose.position.z)\
                        +' '+str(r)+' '+str(p)+' '+str(y)+'\r\n')

    def get_jacobian(self):
        current_joint_values=self.group.get_current_joint_values()
        current_jacobian_matrix_m=numpy.matlib.zeros((6,6))
        current_jacobian_matrix_m=self.group.get_jacobian_matrix(current_joint_values)
        current_jacobian_matrix=np.asarray(current_jacobian_matrix_m)
        #rospy.loginfo('current jacobian matrix computed')
        return current_jacobian_matrix

    def get_cartisian_velocity(self):
        current_state=self.manipulator_state
        current_joint_velocity=numpy.zeros((6,1))
        for i in range(6):
            current_joint_velocity[i,0]=current_state.desired.velocities[i]
        self.current_jacobian_matrix=self.get_jacobian()
        #print('actual joint velocity:'+str(current_joint_velocity[0,0])+' '+str(current_joint_velocity[1,0]))
        current_cartisian_velocity=np.dot(self.current_jacobian_matrix,current_joint_velocity)
        #print('end effector cartisian velocity: '+str(math.sqrt(math.pow(current_cartisian_velocity[0,0],2)+math.pow(current_cartisian_velocity[1,0],2))))
        #print(str(current_cartisian_velocity[0,0])+' '+str(current_cartisian_velocity[1,0])+' '+str(current_cartisian_velocity[2,0]))
        #print(current_cartisian_velocity)
        return current_cartisian_velocity

    def compute_f_k(self,compute_fk,joint_values):
        fk_request=GetPositionFKRequest()
        links=self.move_group_link_names
        fk_request.fk_link_names=links
        fk_request.header.frame_id=links[0]
        state=RobotState()
        joint_names=self.active_joints
        state.joint_state.name=joint_names
        state.joint_state.position=joint_values
        fk_request.robot_state=state
        fk_response=compute_fk(fk_request)
        #manipulator_first_link_pose=fk_response.pose_stamped[0]
        #print(manipulator_first_link_pose)
        end_effector_pose=fk_response.pose_stamped[len(fk_response.pose_stamped)-1]
        #print(end_effector_pose)
        return end_effector_pose

    def quaternion_to_euler(self,given_oriantation):
        R=math.atan2(2*(given_oriantation.w*given_oriantation.x+given_oriantation.y*given_oriantation.z),1-2*(math.pow(given_oriantation.x,2)+math.pow(given_oriantation.y,2)))
        P=math.asin(2*(given_oriantation.w*given_oriantation.y-given_oriantation.x*given_oriantation.z))
        Y=math.atan2(2*(given_oriantation.w*given_oriantation.z+given_oriantation.x*given_oriantation.y),1-2*(math.pow(given_oriantation.y,2)+math.pow(given_oriantation.z,2)))
        #print('R:'+str(R))
        #print('P:'+str(P))
        #print('Y:'+str(Y))
        return R,P,Y


def main(move_type,write_type):
    eigenpy.switchToNumpyMatrix()
    rospy.wait_for_service('/compute_fk')
    while rospy.get_time() == 0:              #wait until the rospy gets the system time
        continue
    manipulator=manipulator_state_class(move_type,write_type)
    rate=rospy.Rate(10)
    rospy.loginfo('the manipulator_state node is working!!')
    if manipulator.move_type ==0 and manipulator.write_type == 1:
        with open('/home/chen/ws_chen/src/mm_meta_pkg/manipulator_moveit_config/data/manipulator_state_desired.txt','w') as f:
            f.write('desired robot state:time  position_x  position_y position_z R P Y \r\n ')
    if manipulator.move_type ==1 and manipulator.write_type == 1:
        with open('/home/chen/ws_chen/src/mm_meta_pkg/manipulator_moveit_config/data/manipulator_state_noise.txt','w') as f:
            f.write('noise_added robot state:time  position_x  position_y position_z R P Y\r\n ')
    if manipulator.move_type ==2 and manipulator.write_type == 1:
            with open('/home/chen/ws_chen/src/mm_meta_pkg/manipulator_moveit_config/data/manipulator_state_controlled.txt','w') as f:
                f.write('controlled robot state:time  position_x  position_y position_z R P Y\r\n ')
    while not rospy.is_shutdown():
        '''
        pose=PoseStamped()
        pose.pose.orientation.x=0
        pose.pose.orientation.y=0
        pose.pose.orientation.z=0
        pose.pose.orientation.w=0.866
        manipulator.quaternion_to_euler(pose.pose.orientation)
        '''
        rate.sleep()



if __name__ == "__main__":
    rospy.init_node('manipulator_state_node',anonymous=True)
    if sys.argv[1]=='desired':
        if sys.argv[2]=='write':
            main(0,1)
        elif sys.argv[2]=='not':
            main(0,0)
        else:
            rospy.logerr('please input the right write_arg: write or not')
            exit()
    elif sys.argv[1]=='noise_added':
        if sys.argv[2]=='write':
            main(1,1)
        elif sys.argv[2]=='not':
            main(1,0)
        else:
            rospy.logerr('please input the right write_arg: write or not')
            exit()
    elif sys.argv[1]=='controlled':
        if sys.argv[2]=='write':
            main(2,1)
        elif sys.argv[2]=='not':
            main(2,0)
        else:
            rospy.logerr('please input the right write_arg: write or not')
            exit()
    else:
        rospy.logerr('please input the right move_arg: desired or noise_added or controlled')
        exit()
