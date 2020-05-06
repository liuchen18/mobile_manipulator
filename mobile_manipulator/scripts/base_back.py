#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import *
import math
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

def euler_to_quaternion(R,P,Y):
    '''
    transfor the given ruler angle to orientation ,return orientation
    '''
    p=Pose()
    p.orientation.w=math.cos(R/2)*math.cos(P/2)*math.cos(Y/2)+math.sin(R/2)*math.sin(P/2)*math.sin(Y/2)
    p.orientation.x=math.sin(R/2)*math.cos(P/2)*math.cos(Y/2)-math.cos(R/2)*math.sin(P/2)*math.sin(Y/2)
    p.orientation.y=math.cos(R/2)*math.sin(P/2)*math.cos(Y/2)+math.sin(R/2)*math.cos(P/2)*math.sin(Y/2)
    p.orientation.z=math.cos(R/2)*math.cos(P/2)*math.sin(Y/2)-math.sin(R/2)*math.sin(P/2)*math.cos(Y/2)
    return p.orientation

rospy.init_node('init_node')
cmd_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
model = GetModelStateRequest()
model.model_name = 'robot'
robot_state = get_state_service(model)
set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
model = SetModelStateRequest()

rate=rospy.Rate(10)
count=0
while count < 5:
    cmd=Twist()
    cmd_pub.publish(cmd)
    rate.sleep()
    count+=1


desired_position=[0,0,0]

desired_state=ModelState()
desired_state.model_name='robot'
desired_state.pose=robot_state.pose
desired_state.pose.position.x = desired_position[0]
desired_state.pose.position.y = desired_position[1]
desired_state.pose.orientation=euler_to_quaternion(0,0,0)
set_state_service(desired_state)

model = GetModelStateRequest()
model.model_name = 'robot'
robot_state = get_state_service(model)
print('position_x: '+str(robot_state.pose.position.x))
print('position_y: '+str(robot_state.pose.position.y))
print('orientation_z: '+str(robot_state.pose.orientation.z))


rospy.loginfo('robot initialized')
