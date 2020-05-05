#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


rospy.init_node('plot_fig')

def plot_fig():
    data_d=[[] for i in range(7)]
    data_a=[[] for i in range(7)]
    data_c=[[] for i in range(7)]
    
    with open('/home/chen/ws_chen/src/mm_meta_pkg/manipulator_moveit_config/data/manipulator_state_desired.txt','r') as f:
        for line in f.readlines()[2:]:
            line=line.split(' ')
            for i in range(7):
                data_d[i].append(float(line[i]))
    with open('/home/chen/ws_chen/src/mm_meta_pkg/manipulator_moveit_config/data/manipulator_state_noise.txt','r') as f:
        for line in f.readlines()[2:]:
            line=line.split(' ')
            for i in range(7):
                data_a[i].append(float(line[i]))
    with open('/home/chen/ws_chen/src/mm_meta_pkg/manipulator_moveit_config/data/manipulator_state_controlled.txt','r') as f:
        for line in f.readlines()[2:]:
            line=line.split(' ')
            for i in range(7):
                data_c[i].append(float(line[i]))
    plt.figure(12)
    plt.subplot(2,3,1)
    plt.plot(data_d[0],data_d[1],label='d')
    plt.plot(data_a[0],data_a[1],label='a')
    plt.plot(data_c[0],data_c[1],label='c')
    plt.title('x_position')
    plt.xlabel('time')
    plt.ylabel('value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,2)
    plt.plot(data_d[0],data_d[2],label='d')
    plt.plot(data_a[0],data_a[2],label='a')
    plt.plot(data_c[0],data_c[2],label='c')
    plt.title('y_position')
    plt.xlabel('time')
    plt.ylabel('value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,3)
    plt.plot(data_d[0],data_d[3],label='d')
    plt.plot(data_a[0],data_a[3],label='a')
    plt.plot(data_c[0],data_c[3],label='c')
    plt.title('z_position')
    plt.xlabel('time')
    plt.ylabel('value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,4)
    plt.plot(data_d[0],data_d[4],label='d')
    plt.plot(data_a[0],data_a[4],label='a')
    plt.plot(data_c[0],data_c[4],label='c')
    plt.title('rx_position')
    plt.xlabel('time')
    plt.ylabel('value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,5)
    plt.plot(data_d[0],data_d[5],label='d')
    plt.plot(data_a[0],data_a[5],label='a')
    plt.plot(data_c[0],data_c[5],label='c')
    plt.title('ry_position')
    plt.xlabel('time')
    plt.ylabel('value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,6)
    plt.plot(data_d[0],data_d[6],label='d')
    plt.plot(data_a[0],data_a[6],label='a')
    plt.plot(data_c[0],data_c[6],label='c')
    plt.title('rz_position')
    plt.xlabel('time')
    plt.ylabel('value')
    plt.legend(loc='upper left')

    plt.show()

if __name__ == '__main__':
    plot_fig()
