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
from moveit_commander.conversions import pose_to_list
import math
from spatialmath_rospy import to_spatialmath, to_ros
from spatialmath import SE3, SO3

# Altura del lapiz
global pen 
#pen = 1
pen = -1.3 #pizarra

global quit
quit = 0

global theta
#theta = 0
theta = -90 #pizarra

global t
t = 0.1

#Altura máxima a la que llegará cada letra en Y
global y_h 
#y_h = 1.0 
y_h = 1.5 #pizarra

#Tamaño de cada letra en ancho y alto
global size
size = 0.07

#Espacio entre cada letra
global space
space = 0.01

global rmatrix
rmatrix = SE3.Ry(theta,'deg')

#Altura cuando se levanta el l
def home():
    # We get the joint values from the group and change some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)
    rospy.loginfo("The robotic arm is at home position.")

def loginfog(msg: str):
    rospy.loginfo("\033[92m%s\033[0m" % msg)

def print_plan(w: list, s: str):
    message = "\n--------------------------------------------------------------"
    for i in range(len(w)):
        message += """
Pose {0}:\n{1}
--------------------------------------------------------------""".format( i , w[i])
    
    rospy.loginfo(message)
    loginfog("Drawing a " + s)

def pen_up_down(wpose, waypoints : list):
    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    return wpose, waypoints

def up_pen(wpose, waypoints : list):
    wpose.position.z = pen + 0.02
    waypoints.append(copy.deepcopy(wpose))

    return wpose, waypoints

def down_pen(wpose, waypoints : list):

    wpose.position.z = pen
    waypoints.append(copy.deepcopy(wpose))

    return wpose, waypoints

def plane_rotation(waypoints : list):
    way = []
    for i in range(len(waypoints)):
        #Primer elemento del producto es la parte de traslación de la matriz de transformación la cual usa las coordenadas cartsianas de cada pose (wpose)
        #Segundo y tercer elementos del producto son las rotaciones necesarias para que el efector final esté perpendicular al plano XY
        T = (SE3(waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z)) * (SE3.Rz(-88, 'deg')) * (SE3.Rx(180, 'deg'))
      
        #La matriz T representa la orientación y posición del efector final con respecto al plano XY original, al multiplicarla por 
        #la matriz de rotación obtenemos la orientación y posición del efector con respecto al plano con la inclinación indicada
        way.append(copy.deepcopy(to_ros(rmatrix*T)))
    return way
    
def move_pen(wpose, waypoints : list, d_x : float, d_y: float, d_z : float = 0):
    #Se copia la pose actual para únicamente modificar las coordenadas cartesianas y que la orientación
    #del efector final no se vea modificada, de esta manera mantenemos el lápiz perpendicular al suelo

    wpose.position.y -= d_x
    wpose.position.x = (y_h if d_y == y_h else
                       (wpose.position.x + d_y))
    if (d_z != 0):
        wpose.position.z = d_z

    waypoints.append(copy.deepcopy(wpose))

    return (wpose, waypoints)

def set_pen(wpose, waypoints : list, p_x : float, p_y: float, p_z : float = 0):
    
    wpose.position.y = -p_x
    wpose.position.x = p_y
    wpose.position.z = p_z
    waypoints.append(copy.deepcopy(wpose))

    return (wpose, waypoints)


def plan_A(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.5*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h)

    return (waypoints, wpose)


def plan_B(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.85, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.15, -size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.2*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.15, -size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.85, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.85, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.15, -size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size*0.2)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.15, -size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.85, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, y_h)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, 0)


    return (waypoints, wpose)


def plan_C(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)

    return (waypoints, wpose)


def plan_D(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.85, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.15, -size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size*0.7)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.15, -size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.85, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, y_h)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, 0)


    return (waypoints, wpose)


def plan_E(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size*0.5)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h)


    return (waypoints, wpose)


def plan_F(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size*0.5)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)


    return (waypoints, wpose)


def plan_G(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)
    
    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.85, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.15, -size*0.15)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size*0.7)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.15, -size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.7, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.15, size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size*0.3)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.3, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size*0.3 + space, y_h)

    return (waypoints, wpose)


def plan_H(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size/2)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)

    return (waypoints, wpose)


def plan_I(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.5, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.5, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)

    return (waypoints, wpose)


def plan_J(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.35, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.5, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size*0.15, size*0.15)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h, pen + 0.02)
    

    return (waypoints, wpose)


def plan_K(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.5*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, -0.5*size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0.5*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, y_h)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)


    return (waypoints, wpose)


def plan_L(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)


    return (waypoints, wpose)


def plan_M(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)


    return (waypoints, wpose)


def plan_N(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, y_h)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)


    return (waypoints, wpose)


def plan_O(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h)    


    return (waypoints, wpose)


def plan_P(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h)


    return (waypoints, wpose)


def plan_Q(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, -0.5*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, -0.5*size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)


    return (waypoints, wpose)


def plan_R(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, -0.5*size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)


    return (waypoints, wpose)


def plan_S(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size*0.5)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size*0.5)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h)
    
    
    return (waypoints, wpose)


def plan_T(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)   

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.5*size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size + space, y_h)


    return (waypoints, wpose)


def plan_U(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)    

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, y_h)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)


    return (waypoints, wpose)


def plan_V(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints) 

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, y_h)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, 0)


    return (waypoints, wpose)


def plan_W(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)  

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, y_h)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, 0)


    return (waypoints, wpose)


def plan_X(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, size)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, pen + 0.02)

    return (waypoints, wpose)


def plan_Y(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size*0.5)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size*0.5)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h, pen + 0.02)

    return (waypoints, wpose)


def plan_Z(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h, pen + 0.02)

    return (waypoints, wpose)


def plan_space(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, space*3, y_h)

    return (waypoints, wpose)


def plan_1(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.3*size)
    
    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0.3*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.5*size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)

    return (waypoints, wpose)

def plan_2(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)

    return (waypoints, wpose)


def plan_3(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.2*size, 0.5*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.8*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)

    return (waypoints, wpose)


def plan_4(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.5*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)

    return (waypoints, wpose)


def plan_5(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.8*size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.2*size, -0.2*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.1*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.2*size, -0.2*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.8*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h)

    return (waypoints, wpose)


def plan_6(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h)

    return (waypoints, wpose)


def plan_7(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, -size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h)

    return (waypoints, wpose)


def plan_8(wpose, waypoints : list):

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size*0.5)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, size + space, y_h)

    return (waypoints, wpose)


def plan_9(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, size, -size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)

    return (waypoints, wpose)


def plan_0(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, 0.7*size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size, -0.15*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.7*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.15*size, -0.15*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.7*size, 0)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.15*size, 0.15*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0.7*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size, 0.15*size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.85*size + space, y_h)

    return (waypoints, wpose)

def plan_minus(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)

    return (waypoints, wpose)

def plan_plus(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.25*size, -0.25*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.5*size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.25*size, 0.25*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)

    return (waypoints, wpose)

def plan_times(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.25*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, -0.5*size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.5*size, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0.5*size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, space, y_h)

    return (waypoints, wpose)


def plan_circle( center_x : float , center_y : float , r : float , theta_o : float  , theta_f : float , wpose, circle_waypoints : list , sentido_x : bool, sentido_y : bool):
    
    if (sentido_x and sentido_y):
        for theta in range(theta_o, theta_f + 1, 2):
            wpose.position.y = center_y + r*math.sin(theta*math.pi/180)
            wpose.position.x = center_x + r*math.cos(theta*math.pi/180)
            circle_waypoints.append(copy.deepcopy(wpose))
    elif (not(sentido_x) and sentido_y):
        for theta in range(theta_o, theta_f + 1, 2):
            wpose.position.y = center_y + r*math.sin(theta*math.pi/180)
            wpose.position.x = center_x - r*math.cos(theta*math.pi/180)
            circle_waypoints.append(copy.deepcopy(wpose))
    elif (sentido_x and not(sentido_y)):
        for theta in range(theta_o, theta_f + 1, 2):
            wpose.position.y = center_y - r*math.sin(theta*math.pi/180)
            wpose.position.x = center_x + r*math.cos(theta*math.pi/180)
            circle_waypoints.append(copy.deepcopy(wpose))
    else:
        for theta in range(theta_o, theta_f + 1, 2):
            wpose.position.y = center_y - r*math.sin(theta*math.pi/180)
            wpose.position.x = center_x - r*math.cos(theta*math.pi/180)
            circle_waypoints.append(copy.deepcopy(wpose))

    return wpose, circle_waypoints


def plan_divide(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.25*size + space/2, -0.25*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = plan_circle(wpose.position.x + space/2, wpose.position.y, space/2, 0, 360, wpose, waypoints, 0, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints)
    
    (wpose, waypoints) = move_pen(wpose, waypoints, -0.25*size + space/2, -0.25*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.25*size + space/2, -0.25*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = plan_circle(wpose.position.x - space/2, wpose.position.y, space/2, 0, 360, wpose, waypoints, 1, 1)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.25*size + 1.5*space, y_h)

    return (waypoints, wpose)


def plan_equal(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 2*space, -0.33*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.33*size)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.5*size, 0)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.5*size + space, y_h)

    return (waypoints, wpose)

def plan_left_parenthesis(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size + 1.5*space, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.15*size, -0.15*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.7*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size, -0.15*size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 2.5*space, y_h)

    return (waypoints, wpose)

def plan_right_parenthesis(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 2.5*space, 0)

    (wpose, waypoints) = down_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size, -0.15*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, -0.7*size)

    (wpose, waypoints) = move_pen(wpose, waypoints, -0.15*size, -0.15*size)

    (wpose, waypoints) = up_pen(wpose, waypoints)

    (wpose, waypoints) = move_pen(wpose, waypoints, 0.15*size + 2.5*space, y_h)

    return (waypoints, wpose)


def plan_(wpose, waypoints : list):

    (wpose, waypoints) = move_pen(wpose, waypoints, 0, 0)

    return (waypoints, wpose)


#By executing this file we can make the robot move to several preconfigured positions in Cartesian coordinates, in the order in which they are in the file
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('planing_node', anonymous=True)
rate = rospy.Rate(10)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("irb2600_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
data_writing_publisher = rospy.Publisher('/figure_writing', String, queue_size=2)
data_writing_publisher.publish(("_none," + str(pen)))
home()
# Calling ``stop()`` ensures that there is no residual movement
group.stop()
wpose = group.get_current_pose().pose

word = input("\n--------------------\nWrite the word you want the robotic arm write: ").upper()

if ((((space + size)*len(word)) - (space/2 + size/2)*(word.count("+") + word.count("-") + word.count(" ") + word.count("*") + word.count("/") + word.count("(") + word.count(")"))) <= 1.5):
    waypoints = []

    x_i = -1*(len(word)/2 * (size + space))#Cálculo de la posición inicial del lápiz
    #Moviendo lápiz a la posición inicial
    (wpose, waypoints) = set_pen(wpose, waypoints, x_i, y_h, pen + 0.02)

    for w in word:
        (waypoints, wpose) = (plan_A(wpose,waypoints) if w == "A" else
                             (plan_B(wpose,waypoints) if w == "B" else
                             (plan_C(wpose,waypoints) if w == "C" else
                             (plan_D(wpose,waypoints) if w == "D" else
                             (plan_E(wpose,waypoints) if w == "E" else
                             (plan_F(wpose,waypoints) if w == "F" else
                             (plan_G(wpose,waypoints) if w == "G" else
                             (plan_H(wpose,waypoints) if w == "H" else
                             (plan_I(wpose,waypoints) if w == "I" else
                             (plan_J(wpose,waypoints) if w == "J" else
                             (plan_K(wpose,waypoints) if w == "K" else
                             (plan_L(wpose,waypoints) if w == "L" else
                             (plan_M(wpose,waypoints) if w == "M" else
                             (plan_N(wpose,waypoints) if w == "N" else
                             (plan_O(wpose,waypoints) if w == "O" else
                             (plan_P(wpose,waypoints) if w == "P" else
                             (plan_Q(wpose,waypoints) if w == "Q" else
                             (plan_R(wpose,waypoints) if w == "R" else
                             (plan_S(wpose,waypoints) if w == "S" else
                             (plan_T(wpose,waypoints) if w == "T" else
                             (plan_U(wpose,waypoints) if w == "U" else
                             (plan_V(wpose,waypoints) if w == "V" else
                             (plan_W(wpose,waypoints) if w == "W" else
                             (plan_X(wpose,waypoints) if w == "X" else
                             (plan_Y(wpose,waypoints) if w == "Y" else
                             (plan_Z(wpose,waypoints) if w == "Z" else 
                             (plan_1(wpose,waypoints) if w == "1" else 
                             (plan_2(wpose,waypoints) if w == "2" else 
                             (plan_3(wpose,waypoints) if w == "3" else 
                             (plan_4(wpose,waypoints) if w == "4" else 
                             (plan_5(wpose,waypoints) if w == "5" else 
                             (plan_6(wpose,waypoints) if w == "6" else 
                             (plan_7(wpose,waypoints) if w == "7" else 
                             (plan_8(wpose,waypoints) if w == "8" else 
                             (plan_9(wpose,waypoints) if w == "9" else 
                             (plan_0(wpose,waypoints) if w == "0" else 
                             (plan_space(wpose,waypoints) if w == " " else                               
                             (plan_plus (wpose,waypoints) if w == "+" else 
                             (plan_minus(wpose,waypoints) if w == "-" else 
                             (plan_times(wpose,waypoints) if w == "*" else 
                             (plan_divide(wpose,waypoints) if w == "/" else
                             (plan_equal(wpose,waypoints) if w == "=" else 
                             (plan_left_parenthesis(wpose,waypoints) if w == "(" else 
                             (plan_right_parenthesis(wpose,waypoints) if w == ")" else 
                             []))))))))))))))))))))))))))))))))))))))))))))
    waypoints = (plane_rotation(waypoints) if theta != 0 else waypoints)
        
    data_writing_publisher.publish("_" + str(word).lower() + "," + str(pen))
    rospy.sleep(1)

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
else:
    rospy.logerr("The word has too many letters.")
home()
