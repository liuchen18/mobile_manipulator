#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
from gazebo_msgs.srv import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from mobile_manipulator.dwa import DWA
import copy

def quaternion_to_euler(given_oriantation):
    '''
    transfor the given orientation into the ruler angle,return R,P,Y
    '''
    R=math.atan2(2*(given_oriantation.w*given_oriantation.x+given_oriantation.y*given_oriantation.z),1-2*(math.pow(given_oriantation.x,2)+math.pow(given_oriantation.y,2)))
    P=math.asin(2*(given_oriantation.w*given_oriantation.y-given_oriantation.x*given_oriantation.z))
    Y=math.atan2(2*(given_oriantation.w*given_oriantation.z+given_oriantation.x*given_oriantation.y),1-2*(math.pow(given_oriantation.y,2)+math.pow(given_oriantation.z,2)))
    return R,P,Y

class base_control():
    def __init__(self,k_x=1,k_y=1,k_theta=1):
        self.base_state= Odometry()
        self.start_base=Bool()
        self.base_vel=[0,0,0]
        self.base_position=[0,0,0]
        self.start_base=True
        self.k_x=k_x
        self.k_y=k_y
        self.k_theta=k_theta
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

    def kinematic_control(self,planned_velocity,pos_x_error,pos_y_error,orien_z_error):

        vel_ctrl_x=planned_velocity[0]*math.cos(orien_z_error)+self.k_x*pos_x_error
        vel_ctrl_y=planned_velocity[1]*math.cos(orien_z_error)+self.k_y*pos_y_error
        delta=(-2*pos_x_error*planned_velocity[1]+2*pos_y_error*planned_velocity[0])/math.sqrt(pos_x_error*pos_x_error+pos_y_error*pos_y_error)
        vel_ctrl_theta=planned_velocity[2]+self.k_theta*math.sqrt(planned_velocity[0]**2+planned_velocity[1]**2)*math.sin(orien_z_error)+0.005*delta
        return vel_ctrl_x,vel_ctrl_y,vel_ctrl_theta
    
def compute_position_error(desired_pose2d,current_base_position):

    x_err_cartisian=desired_pose2d[0]-current_base_position[0]
    y_err_cartisian=desired_pose2d[1]-current_base_position[1]
    theta_err=desired_pose2d[2]-current_base_position[2]

    x_err=x_err_cartisian*math.cos(current_base_position[2])+y_err_cartisian*math.sin(current_base_position[2])
    y_err=y_err_cartisian*math.cos(current_base_position[2])-x_err_cartisian*math.sin(current_base_position[2])

    return x_err,y_err,theta_err




def main():
    rospy.init_node('base_move')
    vel_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
    tra_path='/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory_goal_func.txt'
    vel_tra=[]
    pos_tra=[]
    with open(tra_path,'r') as f:
        for line in f.readlines()[1:]:
            data=list(map(float,line.split(' ')))
            time=data[0]
            x_vel=data[4]
            y_vel=data[5]
            theta_vel=data[6]
            vel_tra.append(copy.deepcopy([time,x_vel,y_vel,theta_vel]))
            pos_tra.append(copy.deepcopy([data[1],data[2],data[3]]))

    while rospy.get_time()==0:
        pass
    base=base_control()

    r=rospy.Rate(20)
    rospy.loginfo('waiting for the initialization of the manipulator')
    while not base.start_base and not rospy.is_shutdown():
        r.sleep()
    rospy.loginfo('manipualtor initialized. start to move')

    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/actual_base_trajectory.txt','w') as f:
        f.write('base trajectory: x y theta time \r\n')

    compute_path='/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory_recompute.txt'
    with open(compute_path,'w') as f:
        f.write('computed position trajectory'+'\r\n')

    base_control_rate=int(1/(vel_tra[1][0]-vel_tra[0][0]))
    print('base control rate: '+str(base_control_rate))
    
    rate=rospy.Rate(base_control_rate)
    start_time=rospy.get_time()
    index=0
    dt=vel_tra[1][0]-vel_tra[0][0]

    while not rospy.is_shutdown():
        index+=1
        print('current_time:'+str(rospy.get_time()-start_time))

        with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/actual_base_trajectory.txt','a') as f:
            f.write(str(base.base_position[0])+' '+str(base.base_position[1])+' '+str(base.base_position[2])+' '+str(rospy.get_time()-start_time)+
                    ' '+str(base.base_vel[0])+' '+str(base.base_vel[1])+' '+str(base.base_vel[2])+'\r\n')

        new_theta=base.base_position[2]+vel_tra[index][3]*dt
        new_x=base.base_position[0]+vel_tra[index][1]*dt*math.cos(base.base_position[2])-vel_tra[index][2]*dt*math.sin(base.base_position[2])
        new_y=base.base_position[1]+vel_tra[index][2]*dt*math.cos(base.base_position[2])+vel_tra[index][1]*dt*math.sin(base.base_position[2])

        with open(compute_path,'a') as f:
            f.write(str(vel_tra[index][0])+' '+str(new_x)+' '+str(new_y)+' '+str(new_theta)+'\r\n')

        #x_err,y_err,theta_err=compute_position_error(pos_tra[index],base.base_position)

        #print('x_err: '+str(x_err)+' y_err: '+str(y_err)+' theta_err: '+str(theta_err))

        vel=Twist()
        #vel.linear.x,vel.linear.y,vel.angular.z=base.kinematic_control(vel_tra[index][1:],x_err,y_vel,theta_err)
        vel.linear.x,vel.linear.y,vel.angular.z=vel_tra[index][1:]
        print('current x: '+str(base.base_position[0])+' desired x: '+str(pos_tra[index][0]))

        vel_pub.publish(vel)

        rate.sleep()

if __name__ == '__main__':
    main()
