#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import math
from geometry_msgs.msg import Pose
from mobile_manipulator.quaternion_euler import *
from mobile_manipulator.trajectory import translate_trajectory

def plot_end_effector():
    t=[]
    position_x=[]
    position_y=[]
    position_z=[]
    orientation_w=[]
    orientation_x=[]
    orientation_y=[]
    orientation_z=[]
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/state.txt','r') as f:
        for line in f.readlines()[2:]:
            line=line.split(' ')
            t.append(float(line[0]))
            position_x.append(float(line[4]))
            position_y.append(float(line[5]))
            position_z.append(float(line[6]))
            orientation_w.append(float(line[7]))
            orientation_x.append(float(line[8]))
            orientation_y.append(float(line[9]))
            orientation_z.append(float(line[10]))
    plt.figure(12)
    plt.subplot(2,1,1)
    plt.plot(t,position_y,label='position_y')
    plt.plot(t,position_x,label='position_x')
    plt.plot(t,position_z,label='position_z')
    plt.title('manipulator pose translation')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')

    plt.subplot(2,1,2)
    plt.plot(t,orientation_w,label='orientation_w')
    plt.plot(t,orientation_x,label='orientation_x')
    plt.plot(t,orientation_y,label='orientation_y')
    plt.plot(t,orientation_z,label='orientation_z')
    plt.title('manipulator pose :orientation')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')
    
    plt.show()


def plot_base_position():
    t_d=[]
    position_x_d=[]
    position_y_d=[]
    position_z_d=[]
    t_a=[]
    position_x_a=[]
    position_y_a=[]
    position_z_a=[]
    t_c=[]
    position_x_c=[]
    position_y_c=[]
    position_z_c=[]
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory_goal_func.txt','r') as f:
        for line in f.readlines()[1:]:
            data=list(map(float,line.split(' ')))
            position_x_d.append(data[1])
            position_y_d.append(data[2])
            position_z_d.append(data[3])
            t_d.append(data[0])
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/actual_base_trajectory.txt','r') as f:
        for line in f.readlines()[1:]:
            data=list(map(float,line.split(' ')))
            position_x_a.append(data[0])
            position_y_a.append(data[1])
            position_z_a.append(data[2])
            t_a.append(data[3])
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory_recompute.txt','r') as f:
        for line in f.readlines()[1:]:
            data=list(map(float,line.split(' ')))
            position_x_c.append(data[1])
            position_y_c.append(data[2])
            position_z_c.append(data[3])
            t_c.append(data[0])
    plt.figure(12)
    plt.subplot(3,1,1)
    plt.plot(t_d,position_x_d,label='desired x position')
    plt.plot(t_a,position_x_a,label='actual x position')
    plt.plot(t_c,position_x_c,label='recompute x position')
    plt.title('base x distance')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')

    plt.subplot(3,1,2)
    plt.plot(t_d,position_y_d,label='desired y position')
    plt.plot(t_a,position_y_a,label='actual y position')
    plt.plot(t_c,position_y_c,label='recompute y position')
    plt.title('base y distance')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')

    plt.subplot(3,1,3)
    plt.plot(t_d,position_z_d,label='desired z position')
    plt.plot(t_a,position_z_a,label='actual z position')
    plt.plot(t_c,position_z_c,label='recompute z position')
    plt.title('base z distance')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')
    
    plt.show()

def plot_base_velocity():
    t_d=[]
    velocity_x_d=[]
    velocity_y_d=[]
    velocity_z_d=[]
    t_a=[]
    velocity_x_a=[]
    velocity_y_a=[]
    velocity_z_a=[]
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory_goal_func.txt','r') as f:
        for line in f.readlines()[1:]:
            data=list(map(float,line.split(' ')))
            velocity_x_d.append(data[4])
            velocity_y_d.append(data[5])
            velocity_z_d.append(data[6])
            t_d.append(data[0])
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/actual_base_trajectory.txt','r') as f:
        for line in f.readlines()[1:]:
            data=list(map(float,line.split(' ')))
            t_a.append(data[3])
            velocity_x_a.append(data[4])
            velocity_y_a.append(data[5])
            velocity_z_a.append(data[6])
            
    plt.figure(12)
    plt.subplot(3,1,1)
    plt.plot(t_d,velocity_x_d,label='desired x velocity')
    plt.plot(t_a,velocity_x_a,label='actual x velocity')
    plt.title('base x velocity')
    plt.xlabel('time')
    plt.ylabel('velocity')
    plt.legend(loc='upper left')

    plt.subplot(3,1,2)
    plt.plot(t_d,velocity_y_d,label='desired y velocity')
    plt.plot(t_a,velocity_y_a,label='actual y velocity')
    plt.title('base y velocity')
    plt.xlabel('time')
    plt.ylabel('velocity')
    plt.legend(loc='upper left')

    plt.subplot(3,1,3)
    plt.plot(t_d,velocity_z_d,label='desired z velocity')
    plt.plot(t_a,velocity_z_a,label='actual z velocity')
    plt.title('base z velocity')
    plt.xlabel('time')
    plt.ylabel('velocity')
    plt.legend(loc='upper left')
    
    plt.show()

def plot_base_error():
    t=[]
    vel_x_err=[]
    vel_y_err=[]
    vel_z_err=[]
    pos_x_err=[]
    pos_y_err=[]
    pos_z_err=[]
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/actual_base_error.txt','r') as f:
        for line in f.readlines()[1:]:
            data=list(map(float,line.split(' ')))
            vel_x_err.append(data[4])
            vel_y_err.append(data[5])
            vel_z_err.append(data[6])
            t.append(data[0])
            pos_x_err.append(data[1])
            pos_y_err.append(data[2])
            pos_z_err.append(data[3])

            
    plt.figure(12)
    plt.subplot(2,1,1)
    plt.plot(t,vel_x_err,label='x velocity error')
    plt.plot(t,vel_y_err,label='y velocity error')
    plt.plot(t,vel_z_err,label='z velocity error')
    plt.title('base velocity error')
    plt.xlabel('time')
    plt.ylabel('velocity')
    plt.legend(loc='upper left')

    plt.subplot(2,1,2)
    plt.plot(t,pos_x_err,label='x position error')
    plt.plot(t,pos_y_err,label='y position error')
    plt.plot(t,pos_z_err,label='z position error')
    plt.title('base position error')
    plt.xlabel('time')
    plt.ylabel('position')
    plt.legend(loc='upper left')
    
    plt.show()

def plot_mm_pose():
    t=[]
    position_x=[]
    position_y=[]
    position_z=[]
    orientation_w=[]
    orientation_x=[]
    orientation_y=[]
    orientation_z=[]
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/state.txt','r') as f:
        for line in f.readlines()[2:]:
            line=list(map(float,line.split(' ')))
            t.append(float(line[0]))
            length=math.sqrt(line[4]**2+line[5]**2)
            alpha=math.atan2(line[5],line[4])

            position_x.append(float(line[1])+length*math.cos(alpha+line[3]))
            position_y.append(float(line[2])+length*math.sin(alpha+line[3]))
            position_z.append(float(line[6]))

            cur_p=Pose()
            cur_p.orientation.w=line[7]
            cur_p.orientation.x=line[8]
            cur_p.orientation.y=line[9]
            cur_p.orientation.z=line[10]
            R,P,Y=quaternion_to_euler(cur_p.orientation)
            Y+=line[3]
            quaternion=euler_to_quaternion(R,P,Y)

            orientation_w.append(quaternion.w)
            orientation_x.append(quaternion.x)
            orientation_y.append(quaternion.y)
            orientation_z.append(quaternion.z)
    plt.figure(12)
    plt.subplot(2,1,1)
    plt.plot(t,position_y,label='position_y')
    plt.plot(t,position_x,label='position_x')
    plt.plot(t,position_z,label='position_z')
    plt.title('mm pose: translation')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')

    plt.subplot(2,1,2)
    plt.plot(t,orientation_w,label='orientation_w')
    plt.plot(t,orientation_x,label='orientation_x')
    plt.plot(t,orientation_y,label='orientation_y')
    plt.plot(t,orientation_z,label='orientation_z')
    plt.title('mm pose: orientation')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')
    
    plt.show()

def plot_mm_error():
    tra=translate_trajectory(y_vel=0.2)
    t=[]
    position_x_err=[]
    position_y_err=[]
    position_z_err=[]
    R_err=[]
    P_err=[]
    Y_err=[]
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/state.txt','r') as f:
        for line in f.readlines()[1:]:
            data=list(map(float,line.split(' ')))
            t.append(data[0])
            pose=tra.compute_desired_pose(data[0])

            length=math.sqrt(data[4]**2+data[5]**2)
            alpha=math.atan2(data[5],data[4])

            cur_x=data[1]+length*math.cos(alpha+data[3])
            cur_y=data[2]+length*math.sin(alpha+data[3])
            cur_z=(data[6])

            cur_pose=Pose()
            cur_pose.orientation.w=data[7]
            cur_pose.orientation.x=data[8]
            cur_pose.orientation.y=data[9]
            cur_pose.orientation.z=data[10]
            cur_R,cur_P,cur_Y=quaternion_to_euler(cur_pose.orientation)
            cur_Y+=data[3]

            position_x_err.append(pose.position.x-cur_x)
            position_y_err.append(pose.position.y-cur_y)
            position_z_err.append(pose.position.z-cur_z)

            d_R,d_P,d_Y=quaternion_to_euler(pose.orientation)
            R_err.append(d_R-cur_R)
            P_err.append(d_P-cur_P)
            Y_err.append(d_Y-cur_Y)
    plt.figure(12)
    plt.subplot(2,1,1)
    plt.plot(t,position_x_err,label='x err')
    plt.plot(t,position_y_err,label='y err')
    plt.plot(t,position_z_err,label='z err')
    plt.title('mm pose error: translation')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')

    plt.subplot(2,1,2)
    plt.plot(t,R_err,label='R err')
    plt.plot(t,P_err,label='P err')
    plt.plot(t,Y_err,label='Y err')
    plt.title('mm pose error: orientation')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')
    
    plt.show()





if __name__ == '__main__':
    plot_base_position()
    plot_end_effector()
    plot_base_velocity()
    plot_base_error()
    plot_mm_pose()
    plot_mm_error()
