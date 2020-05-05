#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import numpy as np
import copy
import moveit_commander
from moveit_commander import MoveGroupCommander



if __name__ == '__main__':
    rospy.init_node('ceshi')
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    m=group.get_jacobian_matrix([1,1,1,1,1,1])
    print(m)
    print('----------------------')
    n=group.get_jacobian_matrix([1,1,1,1,1,1])
    print(n)
    
