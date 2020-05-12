#!/usr/bin/env python
import rospy
from mobile_manipulator.sample_velocity_planner import sample_velocity_planner

rospy.init_node('base_planner')
planner=sample_velocity_planner()
planner.plan_whole_trajectory()
rospy.loginfo('trajectory planned')