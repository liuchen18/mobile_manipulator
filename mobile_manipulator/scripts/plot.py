#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


def plot_fig():
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
            position_x.append(float(line[1])+float(line[4]))
            position_y.append(float(line[2])+float(line[5]))
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
    plt.title('translation')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')

    plt.subplot(2,1,2)
    plt.plot(t,orientation_w,label='orientation_w')
    plt.plot(t,orientation_x,label='orientation_x')
    plt.plot(t,orientation_y,label='orientation_y')
    plt.plot(t,orientation_z,label='orientation_z')
    plt.title('orientation')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')
    
    plt.show()


def plot_base():
    t_d=[]
    position_x_d=[]
    position_y_d=[]
    position_z_d=[]
    t_a=[]
    position_x_a=[]
    position_y_a=[]
    position_z_a=[]
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory.txt','r') as f:
        for line in f.readlines()[1:-1]:
            data=list(map(float,line.split(' ')))
            position_x_d.append(data[0])
            position_y_d.append(data[1])
            position_z_d.append(data[2])
            t_d.append(data[4])
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/actual_base_trajectory.txt','r') as f:
        for line in f.readlines()[1:]:
            data=list(map(float,line.split(' ')))
            position_x_a.append(data[0])
            position_y_a.append(data[1])
            position_z_a.append(data[2])
            t_a.append(data[3])
    plt.figure(12)
    plt.subplot(3,1,1)
    plt.plot(t_d,position_x_d,label='desired x position')
    plt.plot(t_a,position_x_a,label='actual x position')
    plt.title('x distance')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')

    plt.subplot(3,1,2)
    plt.plot(t_d,position_y_d,label='desired y position')
    plt.plot(t_a,position_y_a,label='actual y position')
    plt.title('y distance')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')

    plt.subplot(3,1,3)
    plt.plot(t_d,position_z_d,label='desired z position')
    plt.plot(t_a,position_z_a,label='actual z position')
    plt.title('z distance')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')
    
    plt.show()

if __name__ == '__main__':
    plot_base()
    #plot_fig()
