#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import math
from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelState
import copy
from moveit_msgs.srv import GetPositionIK,GetPositionIKRequest,GetPositionIKResponse
import planner
from geometry_msgs.msg import Pose

class mob_mani_move():
    def __init__(self):
        rospy.init_node('mobile_manipulator_move')
        self.vel_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
        self.mani_pub=rospy.Publisher('/manipulator_controller/command',JointTrajectory,queue_size=10)
        self.time_duration=0


def main():
    rospy.init_node('mobile_manipulator_move')
    vel_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
    mani_pub=rospy.Publisher('/manipulator_controller/command',JointTrajectory,queue_size=10)
    x_position_o=[]
    y_position_o=[]
    theta_position_o=[]
    manipulability=[]
    time_o=[]
    with open('/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory.txt','r') as f:
        for line in f.readlines()[1:]:
            if line == 'Done!':
                break
            x_position_o.append(float(line.split(' ')[0]))
            y_position_o.append(float(line.split(' ')[1]))
            theta_position_o.append(float(line.split(' ')[2]))
            manipulability.append(float(line.split(' ')[3]))
            time_o.append(float(line.split(' ')[4]))
    time_duration=time_o[1]-time_o[0]

    x_position=[]
    y_position=[]
    theta_position=[]
    time=[]
    interpolation=10
    for i in range(len(x_position_o)-1):
        for j in range(interpolation):
            x_position.append(copy.deepcopy((x_position_o[i+1]-x_position_o[i])/interpolation*j+x_position_o[i]))
            y_position.append(copy.deepcopy((y_position_o[i+1]-y_position_o[i])/interpolation*j+y_position_o[i]))
            theta_position.append(copy.deepcopy((theta_position_o[i+1]-theta_position_o[i])/interpolation*j+theta_position_o[i]))
            time.append(copy.deepcopy((time_o[i+1]-time_o[i])/interpolation*j+time_o[i]))

    mani=planner.manipulator()
    p=planner.planner()
    trajectory_length=len(x_position)
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model = GetModelStateRequest()
    model.model_name = 'robot'
    robot_state = get_state_service(model)

    rate=rospy.Rate(1/time_duration*interpolation)
    while rospy.get_time() == 0:
        continue
    start_time=rospy.get_time()
    count=0
    while not rospy.is_shutdown():
        if count < trajectory_length:
            if count %2 == 0:
                desired_pose=p.desired_end_effector_pose(rospy.get_time()-start_time)
                d_pose=Pose()
                d_pose.position.x=desired_pose.position.x-x_position[count]
                d_pose.position.y=desired_pose.position.y-y_position[count]
                d_pose.position.z=desired_pose.position.z
                desired_R,desired_P,desired_Y=p.quaternion_to_euler(desired_pose.orientation)
                desired_Y-=theta_position[count]
                d_pose.orientation=p.euler_to_quaternion(desired_R,desired_P,desired_Y)
                print(d_pose)
                joint_values=mani.compute_inverse_kinematics(d_pose)
                print(joint_values)
                command=JointTrajectory()
                command.joint_names=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint']
                point=JointTrajectoryPoint()
                point.positions=joint_values[:]
                #point.velocities=joint_values[:]
                point.time_from_start=rospy.Duration(0.5)
                command.points.append(point)
                command.header.seq=command.header.seq+1
                command.header.stamp=rospy.Time.now()
                mani_pub.publish(command)



            model = SetModelStateRequest()
            desired_position=[x_position[count],y_position[count],theta_position[count]]
            count+=1
            desired_state=ModelState()
            desired_state.model_name='robot'
            desired_state.pose=robot_state.pose
            desired_state.pose.position.x = desired_position[0]
            desired_state.pose.position.y = desired_position[1]
            desired_state.pose.orientation.z = desired_position[2]
            set_state_service(desired_state)
        '''
        if count < trajectory_length-2:
            cartisian_x_vel=(x_position[count+1]-x_position[count])/time_duration
            cartisian_y_vel=(y_position[count+1]-y_position[count])/time_duration
            theta_vel=(theta_position[count+1]-theta_position[count])/time_duration
            vel=Twist()
            vel.linear.x=cartisian_x_vel*math.cos(theta_position[count])+cartisian_y_vel*math.sin(theta_position[count])
            vel.linear.y=cartisian_y_vel*math.cos(theta_position[count])-cartisian_x_vel*math.sin(theta_position[count])
            vel.angular.z=theta_vel
            vel_pub.publish(vel)
            print(vel)
            #print(rospy.get_time()-start_time)
        else:
            vel=Twist()
            vel_pub.publish(vel)
        count+=1
        '''
        rate.sleep()


if __name__ == '__main__':
    main()



