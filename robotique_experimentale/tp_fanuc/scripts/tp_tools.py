#!/usr/bin/env python3
import argparse
import os
import sys

import time

import numpy as np
from math import *

import rospy
import tf # pour faire les transformations
import tf2_ros # pour envoyer les tf

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *


#   load_robot(robot_name)
#
#   Description :
#       Chargement du robot depuis le serveur des parametres.
#
#   Entree :
#       - robot_name : nom du robot [string]
#
#   Sortie :
#       - DH : table des parametres (convention de Denavit-Hartenberg modifiee) [ndarray]

def load_robot(robot_name):

    links = rospy.get_param(robot_name)
    list_links = list(links.items())
    nb_axis = len(list_links)

    DH = np.zeros([nb_axis,5])
    for axis in range(0,nb_axis):
        link = robot_name + "/link" + str(axis+1)
        sigma = float( rospy.get_param( link + "/sigma" ) )
        a = float( rospy.get_param( link + "/a" ) )
        alpha = float( rospy.get_param( link + "/alpha" ) )*(np.pi/180.0)
        d = float( rospy.get_param( link + "/d" ) )
        theta = float( rospy.get_param( link + "/theta" ) )*(np.pi/180.0)

        DH[axis,:] = np.array([sigma, a, alpha, d, theta])

    return DH


#   pose = matrix_to_pose(T)
#
#   Description :
#       Transforme une matrice de tranformation rigide en geometry_msgs/Pose.
#
#   Entree :
#       - T : matrice de transformation rigide
#
#   Sortie :
#       - pose : geometry_msgs/Pose

def matrix_to_pose(T):
    pose = Pose()
    pose.position = Point(T[0,3], T[1,3], T[2,3])
    q = tf.transformations.quaternion_from_matrix(T)
    pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    return pose



#   matrix = pose_to_matrix(T)
#
#   Description :
#       Transforme une geometry_msgs/Pose en matrice de tranformation rigide.
#
#   Entree :
#       - pose : geometry_msgs/Pose
#
#   Sortie :
#       - T : matrice de transformation rigide

def pose_to_matrix(pose):
    T = np.identity(4)
    T = tf.transformations.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    T[0,3] = pose.position.x
    T[1,3] = pose.position.y
    T[2,3] = pose.position.z
    return T



#   matrix = tf_to_matrix(T)
#
#   Description :
#       Transforme une geometry_msgs/Transform en matrice de tranformation rigide.
#
#   Entree :
#       - transform : geometry_msgs/Transform
#
#   Sortie :
#       - T : matrice de transformation rigide

def tf_to_matrix(transform):
    T = np.identity(4)
    T = tf.transformations.quaternion_matrix([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
    T[0,3] = transform.translation.x
    T[1,3] = transform.translation.y
    T[2,3] = transform.translation.z
    return T



#   joints = array_to_jointstate(q, joints_name)
#
#   Description :
#       Transforme un array en sensor_msgs/JointState.
#
#   Entree :
#       - q : Vecteurs des coordonnees articulaires [array]
#       - joints_name : Nom des articulations [list]
#
#   Sortie :
#       - joints : sensor_msgs/JointState

def array_to_jointstate(q, joints_name):
    joints = JointState()
    joints.header.stamp = rospy.Time.now()
    joints.name = joints_name
    joints.position = q
    return joints
