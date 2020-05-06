#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
from gazebo_msgs.srv import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from mobile_manipulator.dwa import DWA

def quaternion_to_euler(given_oriantation):
    '''
    transfor the given orientation into the ruler angle,return R,P,Y
    '''
    R=math.atan2(2*(given_oriantation.w*given_oriantation.x+given_oriantation.y*given_oriantation.z),1-2*(math.pow(given_oriantation.x,2)+math.pow(given_oriantation.y,2)))
    P=math.asin(2*(given_oriantation.w*given_oriantation.y-given_oriantation.x*given_oriantation.z))
    Y=math.atan2(2*(given_oriantation.w*given_oriantation.z+given_oriantation.x*given_oriantation.y),1-2*(math.pow(given_oriantation.y,2)+math.pow(given_oriantation.z,2)))
    return R,P,Y

class base_control(object):
    def __init__(self):
        self.base_state= Odometry()
        self.start_base=Bool()
        self.base_vel=[0,0,0]
        self.base_position=[0,0,0]
        self.start_base=False
        rospy.Subscriber('start_base',Bool,self.start_callback)
        rospy.Subscriber('odom',Odometry,self.state_callback)

    def state_callback(self,msg):
        self.base_state=msg
        self.base_vel[0]=msg.twist.twist.linear.x
        self.base_vel[1]=msg.twist.twist.linear.y
        self.base_vel[2]=msg.twist.twist.angular.z
        self.base_position[0]=msg.pose.pose.position.x
        self.base_position[1]=msg.pose.pose.position.y
        R,P,Y=quaternion_to_euler(msg.pose.pose.orientation)
        self.base_position[2]=Y

    
    def start_callback(self,msg):
        self.start_base=msg.data


def main():
    rospy.init_node('base_move')
    vel_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
    tra_path='/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory.txt'

    while rospy.get_time()==0:
        pass

    dwa=DWA(file_path=tra_path)
    base=base_control()
    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    r=rospy.Rate(20)
    rospy.loginfo('waiting for the initialization of the manipulator')
    while not base.start_base and not rospy.is_shutdown():
        r.sleep()
    rospy.loginfo('manipualtor initialized. start to move')

    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/actual_base_trajectory.txt','w') as f:
        f.write('base trajectory: x y theta time \r\n')

    rate=rospy.Rate(20)
    start_time=rospy.get_time()
    count=0

    while not rospy.is_shutdown():
        dwa.state_update(rospy.get_time()-start_time,base.base_vel,base.base_position)
        print('current_time:'+str(rospy.get_time()-start_time))

        with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/actual_base_trajectory.txt','a') as f:
            f.write(str(base.base_position[0])+' '+str(base.base_position[1])+' '+str(base.base_position[2])+' '+str(rospy.get_time()-start_time)+'\r\n')

        x_vel,y_vel,theta_vel=dwa.get_best_vel()
        vel=Twist()
        vel.linear.x=x_vel
        vel.linear.y=y_vel
        vel.angular.z=theta_vel
        vel_pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    main()
