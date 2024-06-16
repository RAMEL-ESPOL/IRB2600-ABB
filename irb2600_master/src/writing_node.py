#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from   sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Point
import os
import math
import moveit_commander
from visualization_msgs.msg import Marker, MarkerArray

global pos
pos =       [0.0,0.0,0.0,0.0,0.0,0.0]
global real_pos
real_pos =  [0.0,0.0,0.0,0.0,0.0,0.0]
global torque
torque =    [0.0,0.0,0.0,0.0,0.0,0.0]
global real_vel
real_vel =  [0.0,0.0,0.0,0.0,0.0,0.0]

global ruta_file
ruta_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'matlab', 'data')

global new_file
new_file  = False

global fig
fig = '_none'

global pen
pen = 0.97

global marker_array
marker_array = MarkerArray()

global marker
marker = Marker()
marker.header.frame_id = "base_link"
marker.type = marker.POINTS
marker.action = marker.ADD
marker.scale.x = 0.008
marker.scale.y = 0.008
marker.scale.z = 0
marker.color.r = 0.0
marker.color.g = 0.0
marker.color.b = 0.0
marker.color.a = 1.0
marker.lifetime = rospy.Duration(50)

#Recibimos del Subscriber un msg de tipo JointState de moveit y posteriormente lo publicamos con el Publisher como goal
def state_position(goal_state: JointState):
    global pos
    if fig != '_none': 
        plan_marker()
    else:
        marker.points.clear()
        marker_array.markers.append(marker)

def plan_marker():
    global marker_array
    global marker
    global pen
    pose = group.get_current_pose(group.get_end_effector_link())

    if (pose.pose.position.z <= pen + 0.005) and (pose.pose.position.z >= pen - 0.005):
        p = Point() 
        p = pose.pose.position
        p.z = 0.25
        
        marker.points.append(p)
        marker_array.markers.append(marker)
        marker_pub.publish(marker_array)

def figure(data_figure : str):
    global fig
    global pen
    if "," in str(data_figure.data):
        fig = (str(data_figure.data)).split(",")[0]
        pen = float((str(data_figure.data)).split(",")[1])
    else:
        fig = str(data_figure.data)

if __name__ == "__main__":
    rospy.init_node("writing_node")
    moveit_commander.roscpp_initialize(sys.argv)
    marker_pub      = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size = 2)
    subGoalState    = rospy.Subscriber("/joint_states", JointState, callback = state_position)
    subWritingData  = rospy.Subscriber("/figure_writing", String, callback = figure)
    group           = moveit_commander.MoveGroupCommander("robot_arm")

    rospy.logwarn("The writing_node has been started")
    rospy.spin()