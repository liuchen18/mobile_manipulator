#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt


rospy.init_node('plot_fig')

def plot_fig():
    t=[]
    x=[]
    y=[]
    z=[]
    r=[]
    p=[]
    yoll=[]
    t_n=[]
    x_n=[]
    y_n=[]
    z_n=[]
    r_n=[]
    p_n=[]
    yoll_n=[]
    '''
    t_v=[]
    position_v=[]
    with open('/home/chen/catkin_chen/src/mm_meta_pkg/manipulator_moveit_config/data/robot_state_v.txt','r') as f:
        for line in f.readlines()[2:]:
            line=line.split(' ')
            t_v.append(float(line[0]))
            position_v.append(float(line[1]))
    '''
    with open('/home/chen/ws_chen/src/mm_meta_pkg/manipulator_moveit_config/data/manipulator_state_desired.txt','r') as f:
        for line in f.readlines()[2:]:
            line=line.split(' ')
            t.append(float(line[0]))
            x.append(float(line[1]))
            y.append(float(line[2]))
            z.append(float(line[3]))
            r.append(float(line[4]))
            p.append(float(line[5]))
            yoll.append(float(line[6]))
    with open('/home/chen/ws_chen/src/mm_meta_pkg/manipulator_moveit_config/data/manipulator_state_noise.txt','r') as f:
        for line in f.readlines()[2:]:
            line=line.split(' ')
            t_n.append(float(line[0]))
            x_n.append(float(line[1]))
            y_n.append(float(line[2]))
            z_n.append(float(line[3]))
            r_n.append(float(line[4]))
            p_n.append(float(line[5]))
            yoll_n.append(float(line[6]))
    #plt.figure(12)
    #plt.subplot(2,1,1)
    plt.plot(t,x,label='x')
    plt.plot(t,y,label='y')
    plt.plot(t,z,label='z')
    plt.plot(t_n,x_n,label='x_n')
    plt.plot(t_n,y_n,label='y_n')
    plt.plot(t_n,z_n,label='z_n')
    plt.title('end effector pose')
    plt.xlabel('time')
    plt.ylabel('value')
    plt.legend(loc='upper left')

    plt.show()

if __name__ == '__main__':
    plot_fig()
