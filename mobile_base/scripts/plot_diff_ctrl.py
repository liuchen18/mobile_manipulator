#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


rospy.init_node('plot_fig')

def plot_fig():
    t_c=[]
    position_x_c=[]
    position_y_c=[]
    orientation_z_c=[]
    t_c_=[]
    position_x_c_=[]
    position_y_c_=[]
    orientation_z_c_=[]
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_c.txt','r') as f:
        for line in f.readlines()[2:]:
            line=line.split(' ')
            t_c.append(float(line[0]))
            position_x_c.append(float(line[1]))
            position_y_c.append(float(line[2]))
            orientation_z_c.append(float(line[3]))
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_c_.txt','r') as f:
        for line in f.readlines()[2:]:
            line=line.split(' ')
            t_c_.append(float(line[0]))
            position_x_c_.append(float(line[1]))
            position_y_c_.append(float(line[2]))
            orientation_z_c_.append(float(line[3]))
    plt.figure(12)
    plt.subplot(221)
    plt.plot(t_c,position_y_c,label='control')
    plt.plot(t_c_,position_y_c_,label='control_')
    plt.title('position_y')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')
    

    plt.subplot(2,2,2)  
    plt.plot(t_c,position_x_c,label='control')  
    plt.plot(t_c_,position_x_c_,label='control_')       
    plt.title('position_x')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')

    plt.subplot(223)  
    plt.plot(t_c,orientation_z_c,label='control')
    plt.plot(t_c_,orientation_z_c_,label='control_')         
    plt.title('orientation_z')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')

    plt.subplot(224)
    plt.plot(position_x_c,position_y_c,label='control')
    plt.plot(position_x_c_,position_y_c_,label='control_')
    plt.title('trajectory')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend(loc='upper left')

    plt.show()

if __name__ == '__main__':
    plot_fig()
