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


    pen = 1
    #pen = -1.3 #pizarra

    theta = 0
    #theta = -90 #pizarra

    t = 0.0001
    # t = 0.01

    y_h = 1.4 
    #y_h = 1.5 #pizarra

    size = 0.055

    space = 0.2*size

    x_i = -0.5
    
    waypoints = []

    write_word.home()

    wpose = group.get_current_pose().pose

    (waypoints, wpose) = write_word.triangle(wpose, waypoints, data_writing_publisher, 0.25, x_i, y_h, pen, theta , )

    (waypoints, wpose) = write_word.write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, "A=b*h/2", y_h + 0.05, x_i + 0.2, size, space, pen, theta, t )

    (waypoints, wpose) = write_word.write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, "A=20*17/2", y_h - 0.05, x_i + 0.2, size, space, pen, theta, t )

    (waypoints, wpose) = write_word.write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, "A=170 cm", y_h - 0.15, x_i + 0.2, size, space, pen, theta, t )

    (waypoints, wpose) = write_word.write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, "2", y_h - 0.15, -wpose.position.y, size*0.4, space, pen, theta, t )

    # (waypoints, wpose) = write_word.write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, "y=2(x+4)", y_h, x_i, size, space, pen, theta, t )

    # (waypoints, wpose) = write_word.write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, "y=2x+8", 0.9)

    # (waypoints, wpose) = write_word.write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, "Ec factorized!", 0.8)

    # (waypoints, wpose) = write_word.write(wpose, waypoints, robot, scene, group, display_trajectory_publisher, data_writing_publisher, "23", 0.7, 0.01)

    plan  = group.compute_cartesian_path(waypoints, t, 0.0)[0]

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    group.execute(plan, wait=True)
    rospy.loginfo("Planning succesfully executed.\n")
    rospy.sleep(1)
    data_writing_publisher.publish("_none")
