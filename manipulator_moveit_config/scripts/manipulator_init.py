#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import *
from control_msgs.msg import JointTrajectoryControllerState
import math
from gazebo_msgs.msg import ModelState
import numpy as np

manipulator_state=JointTrajectoryControllerState()
got_msg = False

def callback(msg):
    global got_msg
    global manipulator_state
    got_msg=True
    manipulator_state=msg

def main():
    rospy.init_node('manipulator_init')
    manipulator_pub=rospy.Publisher('/manipulator_controller/command',JointTrajectory,queue_size=10)
    rospy.Subscriber('/manipulator_controller/state',JointTrajectoryControllerState,callback)
    vel_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model = GetModelStateRequest()
    model.model_name = 'robot'
    robot_state = get_state_service(model)
    set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    model = SetModelStateRequest()
    rate=rospy.Rate(10)
    mani_init=False
    base_init=False
    mani_printed = False
    base_printed = False
    init_position=[0,-0.3,-2,0,1,0] #[0,-0.3,-2,0,1,0]
    while not rospy.is_shutdown():
        global manipulator_state
        current_manipulator_state = manipulator_state
        global got_msg
        if got_msg == True:
            difference=[]
            for i in range(6):
                difference.append(current_manipulator_state.desired.positions[i]-init_position[i])
            if np.linalg.norm(difference) > 0.001:
                command=JointTrajectory()
                command.joint_names=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                'wrist_3_joint']
                command.points=[JointTrajectoryPoint(positions=init_position,velocities=[],time_from_start=rospy.Duration(1.0))]
                command.header.seq=command.header.seq+1
                command.header.stamp=rospy.Time.now()
                manipulator_pub.publish(command)
            else:
                if mani_printed == False:
                    rospy.loginfo('manipulator initialized')
                    mani_printed = True
                mani_init=True
        if robot_state.pose.position.x > 0.001 or robot_state.pose.position.y > 0.001 or robot_state.pose.orientation.z > 0.0001: 
            model = GetModelStateRequest()
            model.model_name = 'robot'
            robot_state = get_state_service(model)
            desired_state=ModelState()
            desired_state.model_name='robot'
            desired_state.pose=robot_state.pose
            desired_state.pose.position.x = 0
            desired_state.pose.position.y = 0
            desired_state.pose.orientation.z = 0
            set_state_service(desired_state)

            '''
            cmd=Twist()
            model = GetModelStateRequest()
            model.model_name = 'robot'
            robot_state = get_state_service(model)
            #cmd.linear.x=-(robot_state.pose.position.x-0.5)
            cmd.linear.x=-robot_state.pose.position.x
            cmd.linear.y=-robot_state.pose.position.y
            cmd.angular.z=-5*(robot_state.pose.orientation.z)
            vel_pub.publish(cmd)
            '''

        else:
            cmd=Twist()
            vel_pub.publish(cmd)
            base_init=True
            if base_printed == False:
                rospy.loginfo('move base initialized')
                base_printed=True
        if mani_init == True and base_init == True:
            exit()
        rate.sleep()

if __name__ == '__main__':
    main()