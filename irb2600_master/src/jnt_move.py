#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import moveit_commander


def perform_trajectory():

    rospy.init_node('abb_trajectory_publisher') #Node definition

    argv = sys.argv[1:] #It will read the webshell 

    # We get the joint values from the group and change some of the values:
    joint_goal = [ float(argv[0]) , float(argv[1]) , float(argv[2]) ,float(argv[3]) ,float(argv[4]) , float(argv[5])  ] #Positions in radians where to move
    
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)


if __name__ == '__main__':

    group = moveit_commander.MoveGroupCommander("irb2600_arm")
    perform_trajectory()