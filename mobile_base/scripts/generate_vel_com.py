#!/usr/bin/env python

import rospy
#from sympy import *
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
import math
import sys
from gazebo_msgs.srv import *

class move_base_control_class:
    def __init__(self,k_x=10,k_y=10,k_theta=10):
        self.k_x=k_x
        self.k_y=k_y
        self.k_theta=k_theta
        self.max_y_error=0
    def kinematic_control(self,vel_x_d,vel_y_d,vel_theta_d,pos_x_error,pos_y_error,orien_z_error):
        if self.max_y_error<pos_y_error:
            self.max_y_error=pos_y_error
        vel_ctrl_x=vel_x_d*math.cos(orien_z_error)+self.k_x*pos_x_error
        vel_ctrl_y=vel_y_d*math.cos(orien_z_error)+self.k_y*pos_y_error
        delta=(-2*pos_x_error*vel_y_d+2*pos_y_error*vel_x_d)/math.sqrt(pos_x_error*pos_x_error+pos_y_error*pos_y_error)
        vel_ctrl_theta=vel_theta_d+self.k_theta*math.sqrt(vel_x_d*vel_x_d+vel_y_d*vel_y_d)*math.sin(orien_z_error)+0.005*delta
        #vel_ctrl_theta=vel_theta_d+self.k_theta*math.sin(orien_z_error)+0.005*delta
        return vel_ctrl_x,vel_ctrl_y,vel_ctrl_theta

class move_base_pd_control_class:
    def __init__(self,k_x_p=10,k_x_d=0.2,k_y_p=10,k_y_d=0.2,k_theta_p=1,k_theta_d=0.2):
        self.k_x_p=k_x_p
        self.k_x_d=k_x_d
        self.k_y_p=k_y_p
        self.k_y_d=k_y_d
        self.k_theta_p=k_theta_p
        self.k_theta_d=k_theta_d
    
    def pd_control(self,vel_x_d,vel_y_d,vel_theta_d,pos_x_error,pos_y_error,orien_z_error,vel_x_error,vel_y_error,vel_theta_error):
        vel_ctrl_x=vel_x_d+self.k_x_p*pos_x_error+self.k_x_d*vel_x_error
        vel_ctrl_y=vel_y_d+self.k_y_p*pos_y_error+self.k_y_d*vel_y_error
        vel_ctrl_theta=vel_theta_d+self.k_theta_p*orien_z_error+self.k_theta_d*vel_theta_error
        return vel_ctrl_x,vel_ctrl_y,vel_ctrl_theta

class tra_cmd_class:
    def __init__(self):
        self.ready_to_sub=False
        rospy.init_node("trajectory_command",anonymous=True)
        self.command_pub=rospy.Publisher('/mm/cmd_vel',Twist,queue_size=10)
        self.type_pub=rospy.Publisher('control_type',UInt8,queue_size=10)
        rospy.Subscriber('ready_to_sub', UInt8,self.callback)
        self.start_time=float('inf')
    
    def callback(self,msg):
        self.ready_to_sub = True

    def desired_position(self,t):
        pos_x=2-2*math.cos(t)
        pos_y=2*math.sin(t)
        pos_theta=0
        return pos_x,pos_y,pos_theta
    
    def desired_tra_cmd(self,time):
        vel_x=2*math.sin(time)
        vel_y=2*math.cos(time)
        vel_theta=0
        return vel_x,vel_y,vel_theta

    def vel_error(self,time):
        error_x=0.5*math.cos(2*time)
        error_y=0.5*math.sin(4*time)
        error_theta=0.2*math.sin(time/2)
        return error_x,error_y,error_theta


def main(type):
    try:
        tra_cmd=tra_cmd_class()
        move_base_ctrl=move_base_control_class()
        pd_control=move_base_pd_control_class()
        get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        rate=rospy.Rate(10)
        t=0
        while tra_cmd.ready_to_sub == False and not rospy.is_shutdown():
            print('waiting for sub_robot_state node ready')
            rate.sleep()
        print('trajectory command node got msg : sub_robot_state node is ready')
        #deside the control type : desired or actual or control
        control_type_msg=UInt8()
        if type == 'desired':
            control_type=0
            print('trajectory command node got control type: desired')
        elif type == 'actual':
            control_type=1
            print('trajectory command node got control type: actual')
        elif type == 'control':
            control_type=2
            print('trajectory command node got control type: control')
        elif type == 'pd':
            control_type=3
            print('trajectory command node got control type: PD')
        else:
            print('---------------invalid control type,please input desired or actual or control or pd')
            exit()
        tra_cmd.type_pub.publish(control_type)
        with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/vel_command_d.txt','w') as f:
            f.write('velocity command:\r\n')
        with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/vel_command_a.txt','w') as f:
            f.write('velocity command:\r\n')
        with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/vel_command_c.txt','w') as f:
            f.write('velocity command:\r\n')
        with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/vel_command_pd.txt','w') as f:
            f.write('velocity command:\r\n')
        tra_cmd.start_time=rospy.get_time()
        while not rospy.is_shutdown() and t < 2*math.pi:
            t=rospy.get_time()-tra_cmd.start_time
            cmd=Twist()
            #desired
            if control_type == 0:
                cmd.linear.x,cmd.linear.y,cmd.angular.z=tra_cmd.desired_tra_cmd(t)
                with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/vel_command_d.txt','a') as f:
                    f.write(str(cmd.linear.x))
                    f.write(' ')
                    f.write(str(cmd.linear.y))
                    f.write('\r\n')
            #actual
            elif control_type == 1:
                vel_x,vel_y,vel_theta=tra_cmd.desired_tra_cmd(t)
                error_x,error_y,error_theta=tra_cmd.vel_error(t)
                cmd.linear.x=vel_x+error_x
                cmd.linear.y=vel_y+error_y
                cmd.angular.z=vel_theta+error_theta
                with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/vel_command_a.txt','a') as f:
                    f.write(str(cmd.linear.x))
                    f.write(' ')
                    f.write(str(cmd.linear.y))
                    f.write('\r\n')
            #control
            elif control_type == 2:
                vel_x_d,vel_y_d,vel_theta_d=tra_cmd.desired_tra_cmd(t)
                error_x,error_y,error_theta=tra_cmd.vel_error(t)
                pos_x_d,pos_y_d,pos_theta_d=tra_cmd.desired_position(t)
                model = GetModelStateRequest()
                model.model_name = 'mm'
                robot_state = get_state_service(model)
                pos_x_error=pos_x_d-robot_state.pose.position.x
                pos_y_error=pos_y_d-robot_state.pose.position.y
                pos_theta_error=pos_theta_d-robot_state.pose.orientation.z
                vel_x,vel_y,vel_theta=move_base_ctrl.kinematic_control(vel_x_d,vel_y_d,vel_theta_d,pos_x_error,pos_y_error,pos_theta_error)
                cmd.linear.x=vel_x+error_x
                cmd.linear.y=vel_y+error_y
                cmd.angular.z=vel_theta+error_theta
                with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/vel_command_c.txt','a') as f:
                    f.write(str(cmd.linear.x))
                    f.write(' ')
                    f.write(str(cmd.linear.y))
                    f.write(' ')
                    f.write(str(cmd.angular.z))
                    f.write('\r\n')
            elif control_type==3:
                vel_x_d,vel_y_d,vel_theta_d=tra_cmd.desired_tra_cmd(t)
                error_x,error_y,error_theta=tra_cmd.vel_error(t)
                pos_x_d,pos_y_d,pos_theta_d=tra_cmd.desired_position(t)
                model = GetModelStateRequest()
                model.model_name = 'mm'
                robot_state = get_state_service(model)
                pos_x_error=pos_x_d-robot_state.pose.position.x
                pos_y_error=pos_y_d-robot_state.pose.position.y
                pos_theta_error=pos_theta_d-robot_state.pose.orientation.z
                vel_x_error=vel_x_d-robot_state.twist.linear.x
                vel_y_error=vel_y_d-robot_state.twist.linear.y
                vel_theta_error=vel_theta_d-robot_state.twist.angular.z
                vel_x,vel_y,vel_theta=pd_control.pd_control(vel_x_d,vel_y_d,vel_theta_d,pos_x_error,pos_y_error,pos_theta_error,vel_x_error,vel_y_error,vel_theta_error)
                cmd.linear.x=vel_x+error_x
                cmd.linear.y=vel_y+error_y
                cmd.angular.z=vel_theta+error_theta
                with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/vel_command_pd.txt','a') as f:
                    f.write(str(cmd.linear.x))
                    f.write(' ')
                    f.write(str(cmd.linear.y))
                    f.write(' ')
                    f.write(str(cmd.angular.z))
                    f.write('\r\n')

            tra_cmd.command_pub.publish(cmd)
            #print('msg published')
            rate.sleep()
        cmd=Twist()
        tra_cmd.command_pub.publish(cmd)
        #print('max_y_error:'+str(move_base_ctrl.max_y_error))
    except rospy.ROSInterruptException:
        return

if __name__ == '__main__':
    input_arg=sys.argv[1]
    #print(a)
    main(input_arg)
