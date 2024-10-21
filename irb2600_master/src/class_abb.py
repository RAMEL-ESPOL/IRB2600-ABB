#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String
import math
import write_word

if __name__ == "__main__":

    #By executing this file we can make the robot move to several preconfigured positions in Cartesian coordinates, in the order in which they are in the file
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('class_node', anonymous=True)
    rate = rospy.Rate(10)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()    
    group = moveit_commander.MoveGroupCommander("irb2600_arm")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    data_writing_publisher       = rospy.Publisher('/figure_writing', String, queue_size=2)


    write_word.home()

    write_word.write(robot, scene, group, display_trajectory_publisher, data_writing_publisher, "y=2(x+4)")

    write_word.write(robot, scene, group, display_trajectory_publisher, data_writing_publisher, "y=2x+8", 0.9)

    write_word.write(robot, scene, group, display_trajectory_publisher, data_writing_publisher, "Ec factorized!", 0.8)

    write_word.write(robot, scene, group, display_trajectory_publisher, data_writing_publisher, "23", 0.7, 0.01)

    

    