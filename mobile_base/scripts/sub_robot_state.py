#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import *
import matplotlib.pyplot as plt
from std_msgs.msg import UInt8
import math

class sub_robot_state_class(object):
    def __init__(self):
        rospy.init_node('sub_robot_state', anonymous=True)
        rospy.Subscriber('control_type', UInt8,self.callback)
        self.pub_state=rospy.Publisher('ready_to_sub',UInt8,queue_size=10)
        self.rate=rospy.Rate(20)
        self.start_to_sub=False
        self.start_time=float('inf')
        self.control_msg_time=float('inf')
        self.is_plot=False
        self.control_type=0


    def callback(self,msg):
        self.start_to_sub=True
        self.control_msg_time=rospy.get_time()
        #print(rospy.get_time())
        if msg.data == 0:
            print('sub robot_state node got control type: desired')
            self.control_type=0
        elif msg.data == 1:
            print('sub robot_state node got control type: actual')
            self.control_type=1
        elif msg.data == 2:
            print('sub robot_state node got control type: control')
            self.control_type=2
        elif msg.data == 3:
            print('sub robot_state node got control type: pd')
            self.control_type=3
        self.start_time=rospy.get_time()
        print('------ subscribing robot states --------')


    def plot_fig(self):
        print('starting to plot figures')
        if self.control_type == 0:
            dir='/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_d.txt'
        elif self.control_type == 1:
            dir='/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_a.txt'
        elif self.control_type == 2:
            dir='/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_c.txt'
        elif self.control_type == 3:
            dir='/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_pd.txt'
        with open(dir,'r') as f:
            t=[]
            position_x=[]
            position_y=[]
            orientation_z=[]

            for line in f.readlines()[2:]:
                line=line.split(' ')
                t.append(float(line[0]))
                position_x.append(float(line[1]))
                position_y.append(float(line[2]))
                orientation_z.append(float(line[3]))
        plt.plot(t,position_y,marker='o',label='position_y')
        plt.plot(t,position_x,marker='*',label='position_x')
        plt.plot(t,orientation_z,label='orientation_z')
        plt.xlabel('time')
        plt.ylabel('distance')
        plt.legend(loc='upper left')
        plt.show()




def main():
    sub_robot_state=sub_robot_state_class()
    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    
    model = GetModelStateRequest()
    model.model_name = 'mm'
    robot_state = get_state_service(model)
    #print(robot_state)
    r=rospy.Rate(100)
    count=0
    while count < 10:
        ready_to_sub=UInt8()
        sub_robot_state.pub_state.publish(ready_to_sub)
        count = count + 1
        r.sleep()
    print('sub_robot_state node is ready')
    while not sub_robot_state.start_to_sub == True:
        r.sleep()
    if sub_robot_state.control_type ==0:
        with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_d.txt','w') as f:
            f.write('robot state:time  position_x  position_y orientation_z\r\n ')
    if sub_robot_state.control_type ==1:
        with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_a.txt','w') as f:
            f.write('robot state:time  position_x  position_y orientation_z\r\n ')
    if sub_robot_state.control_type ==2:
        with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_c.txt','w') as f:
            f.write('robot state:time  position_x  position_y orientation_z\r\n ')
    if sub_robot_state.control_type ==3:
        with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_pd.txt','w') as f:
            f.write('robot state:time  position_x  position_y orientation_z\r\n ')
    while not rospy.is_shutdown():
        model = GetModelStateRequest()
        model.model_name = 'mm'
        robot_state = get_state_service(model)
        #print('got robot_state')
        if sub_robot_state.control_type == 0:
            with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_d.txt','a') as f:
                f.write(str(rospy.get_time()-sub_robot_state.start_time)+' '+str(robot_state.pose.position.x)+' '+str(robot_state.pose.position.y)+' '+str(robot_state.pose.orientation.z)+'\r\n')
        if sub_robot_state.control_type == 1:
            with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_a.txt','a') as f:
                f.write(str(rospy.get_time()-sub_robot_state.start_time)+' '+str(robot_state.pose.position.x)+' '+str(robot_state.pose.position.y)+' '+str(robot_state.pose.orientation.z)+'\r\n')
        if sub_robot_state.control_type == 2:
            with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_c.txt','a') as f:
                f.write(str(rospy.get_time()-sub_robot_state.start_time)+' '+str(robot_state.pose.position.x)+' '+str(robot_state.pose.position.y)+' '+str(robot_state.pose.orientation.z)+'\r\n')
        if sub_robot_state.control_type == 3:
            with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_base/data/robot_state_pd.txt','a') as f:
                f.write(str(rospy.get_time()-sub_robot_state.start_time)+' '+str(robot_state.pose.position.x)+' '+str(robot_state.pose.position.y)+' '+str(robot_state.pose.orientation.z)+'\r\n')
        #print(rospy.get_time()-sub_robot_state.control_msg_time)
        #print(sub_robot_state.start_to_sub)
        if((rospy.get_time()-sub_robot_state.control_msg_time)>2*math.pi):
            sub_robot_state.is_plot=True
        if(sub_robot_state.is_plot):
            sub_robot_state.plot_fig()
        sub_robot_state.rate.sleep()

if __name__ == '__main__':
    main()



