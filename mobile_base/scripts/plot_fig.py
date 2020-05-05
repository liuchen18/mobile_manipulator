#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


rospy.init_node('plot_fig')

def plot_fig():
    t_d=[]
    position_x_d=[]
    position_y_d=[]
    orientation_z_d=[]
    t_a=[]
    position_x_a=[]
    position_y_a=[]
    orientation_z_a=[]
    t_c=[]
    position_x_c=[]
    position_y_c=[]
    orientation_z_c=[]
    t_pd=[]
    position_x_pd=[]
    position_y_pd=[]
    orientation_z_pd=[]
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_d.txt','r') as f:
        for line in f.readlines()[2:]:
            line=line.split(' ')
            t_d.append(float(line[0]))
            position_x_d.append(float(line[1]))
            position_y_d.append(float(line[2]))
            orientation_z_d.append(float(line[3]))
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_a.txt','r') as f:
        for line in f.readlines()[2:]:
            line=line.split(' ')
            t_a.append(float(line[0]))
            position_x_a.append(float(line[1]))
            position_y_a.append(float(line[2]))
            orientation_z_a.append(float(line[3]))
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_c.txt','r') as f:
        for line in f.readlines()[2:]:
            line=line.split(' ')
            t_c.append(float(line[0]))
            position_x_c.append(float(line[1]))
            position_y_c.append(float(line[2]))
            orientation_z_c.append(float(line[3]))
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_pd.txt','r') as f:
        for line in f.readlines()[2:]:
            line=line.split(' ')
            t_pd.append(float(line[0]))
            position_x_pd.append(float(line[1]))
            position_y_pd.append(float(line[2]))
            orientation_z_pd.append(float(line[3]))
    plt.figure(12)
    plt.subplot(2,2,1)
    plt.plot(t_d,position_y_d,label='desired')
    plt.plot(t_a,position_y_a,label='actual')
    plt.plot(t_c,position_y_c,label='control')
    plt.plot(t_pd,position_y_pd,label='pd_control')
    plt.title('position_y')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')
    

    plt.subplot(2,2,2)
    plt.plot(t_d,position_x_d,label='desired')
    plt.plot(t_a,position_x_a,label='actual')   
    plt.plot(t_c,position_x_c,label='control')   
    plt.plot(t_pd,position_x_pd,label='pd_control')       
    plt.title('position_x')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')

    plt.subplot(223)
    plt.plot(t_d,orientation_z_d,label='desired')
    plt.plot(t_a,orientation_z_a,label='actual')   
    plt.plot(t_c,orientation_z_c,label='control')  
    plt.plot(t_pd,orientation_z_pd,label='pd_control')       
    plt.title('orientation_z')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.legend(loc='upper left')

    plt.subplot(224)
    plt.plot(position_x_d,position_y_d,label='desired')
    plt.plot(position_x_a,position_y_a,label='actual')
    plt.plot(position_x_c,position_y_c,label='control')
    plt.plot(position_x_pd,position_y_pd,label='pd_control')
    plt.title('trajectory')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend(loc='upper left')

    plt.show()

if __name__ == '__main__':
    plot_fig()
