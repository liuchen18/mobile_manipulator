#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
from gazebo_msgs.srv import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class base_control(object):
    def __init__(self):
        self.base_state= Odometry()
        self.start_base=Bool()
        self.start_base=False
        rospy.Subscriber('start_base',Bool,self.start_callback)
        rospy.Subscriber('odom',Odometry,self.state_callback)

    def state_callback(self,msg):
        self.base_state=msg
    
    def start_callback(self,msg):
        self.start_base=msg.data


def main():
    rospy.init_node('base_move')
    vel_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
    base=base_control()
    x_position_o=[]
    y_position_o=[]
    theta_position_o=[]
    manipulability=[]
    time_o=[]
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory.txt','r') as f:
        for line in f.readlines()[1:]:
            if line == 'Done!':
                break
            x_position_o.append(float(line.split(' ')[0]))
            y_position_o.append(float(line.split(' ')[1]))
            theta_position_o.append(float(line.split(' ')[2]))
            manipulability.append(float(line.split(' ')[3]))
            time_o.append(float(line.split(' ')[4]))

    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    time_duration=time_o[1]-time_o[0]

    while rospy.get_time()==0:
        pass
    r=rospy.Rate(20)
    rospy.loginfo('waiting for the initialization of the manipulator')
    while not base.start_base:
        r.sleep()
    rospy.loginfo('manipualtor initialized. start to move')

    rate=rospy.Rate(1/time_duration)
    start_time=rospy.get_time()
    count=0

    while not rospy.is_shutdown():
        vel=Twist()
        if rospy.get_time()-start_time < time_o[-2]:
            #compute cartisian velocity
            
            cartisian_x_vel=(x_position_o[count+1]-x_position_o[count])/time_duration
            cartisian_y_vel=(y_position_o[count+1]-y_position_o[count])/time_duration
            theta_vel=(theta_position_o[count+1]-theta_position_o[count])/time_duration
            
            print('this x: '+str(x_position_o[count])+' next x: '+str(x_position_o[count+1])+' cartisian vel : '+str(cartisian_x_vel))

            theta_mid=(theta_position_o[count]+theta_position_o[count+1])/2
            vel.linear.x=cartisian_x_vel*math.cos(theta_mid)+cartisian_y_vel*math.sin(theta_mid)
            vel.linear.y=cartisian_y_vel*math.cos(theta_mid)-cartisian_x_vel*math.sin(theta_mid)
            vel.angular.z=theta_vel
            #print(vel)
            count+=1
        elif rospy.get_time()-start_time - time_o[-1] < 5:
            model = GetModelStateRequest()
            model.model_name = 'robot'
            robot_state = get_state_service(model)
            #print('x:'+str(robot_state.pose.position.x)+' y: '+str(robot_state.pose.position.y))
            #print('stopping')
        else:
            count=0
            model = GetModelStateRequest()
            model.model_name = 'robot'
            robot_state = get_state_service(model)
            vel.linear.x=-robot_state.pose.position.x
            vel.linear.y=-robot_state.pose.position.y
            vel.angular.z=-1*(robot_state.pose.orientation.z)
            #print('backing')
        '''
        cartisian_x_vel=(-x_position_o[0]+x_position_o[-1])/(time_o[-1]-time_o[0])
        cartisian_y_vel=(-y_position_o[0]+y_position_o[-1])/(time_o[-1]-time_o[0])
        theta_vel=(theta_position_o[-1]-theta_position_o[0])/(time_o[-1]-time_o[0])
        print('cartisian x vel: '+str(cartisian_x_vel)+' cartisian y vel: '+str(cartisian_y_vel)+' theta vel: '+str(theta_vel))
        theta_mid=(theta_position_o[-1]+theta_position_o[0])/2
        vel.linear.x=cartisian_x_vel*math.cos(theta_mid)+cartisian_y_vel*math.sin(theta_mid)
        vel.linear.y=cartisian_y_vel*math.cos(theta_mid)-cartisian_x_vel*math.sin(theta_mid)
        print('x vel: '+str(vel.linear.x)+' y vel: '+str(vel.linear.y))
        vel.angular.z=theta_vel
        if rospy.get_time()-start_time >time_o[-1]:
            vel.linear.x=0
            vel.linear.y=0
            vel.angular.z=0
        '''
        vel_pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    main()