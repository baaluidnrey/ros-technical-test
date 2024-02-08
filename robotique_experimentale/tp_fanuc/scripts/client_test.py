#!/usr/bin/env python3
import time
import numpy as np
import rospy
import tf # pour faire les transformations
import tf2_ros # pour envoyer les tf
from math import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *

from tp_fanuc.srv import *



#   -----------------------------------------------------------------------
#   Services MGD
#   -----------------------------------------------------------------------

# get_pose
def client_get_pose(joints):
    rospy.wait_for_service('get_pose')
    try:
        get_pose = rospy.ServiceProxy('get_pose', mgd_get_pose)
        resp = get_pose(joints)
        return resp.pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def test_get_pose():
    joints = JointState()
    joints.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pose = client_get_pose(joints)

    print("-------------------------------")
    print("Test de get_pose :")
    print(joints)
    print(pose)


# move
def client_move(joints):
    rospy.wait_for_service('move')
    try:
        move = rospy.ServiceProxy('move', mgd_move)
        resp = move(joints)
        return resp.motion_done
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def test_move():
    joints = JointState()
    joints.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    motion_done = client_move(joints)

    print("-------------------------------")
    print("Test de move :")
    print(joints)
    print(motion_done)


# get_homogeneous_transform
def client_get_homogeneous_transform(joints,i,j):
    rospy.wait_for_service('get_homogeneous_transform')
    try:
        get_homogeneous_transform = rospy.ServiceProxy('get_homogeneous_transform', mgd_get_homogeneous_transform)
        resp = get_homogeneous_transform(joints,i,j)
        return resp.pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def test_get_homogeneous_transform():
    joints = JointState()
    joints.position = [0.0, 0.0]
    i = Int8(0)
    j = Int8(2)
    pose = client_get_homogeneous_transform(joints,i,j)

    print("-------------------------------")
    print("Test de get_homogeneous_transform :")
    print(joints)
    print(i)
    print(j)
    print(pose)


# get_links_pose
def client_get_links_pose(joints):
    rospy.wait_for_service('get_links_pose')
    try:
        get_links_pose = rospy.ServiceProxy('get_links_pose', mgd_get_links_pose)
        resp = get_links_pose(joints)
        return resp.links_pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def test_get_links_pose():
    joints = JointState()
    joints.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    links_pose = client_get_links_pose(joints)

    print("-------------------------------")
    print("Test de get_links_pose :")
    print(joints)
    print(links_pose)


# get_joints
def client_mgd_get_joints():
    rospy.wait_for_service('mgd_get_joints')
    try:
        get_joints = rospy.ServiceProxy('mgd_get_joints', mgd_get_joints)
        resp = get_joints()
        return resp.joints
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def test_mgd_get_joints():
    joints = client_mgd_get_joints()

    print("-------------------------------")
    print("Test de mgd_get_joints :")
    print(joints)


#   -----------------------------------------------------------------------
#   Services MGI
#   -----------------------------------------------------------------------

# get_joints
def client_get_joints(pose, config):
    rospy.wait_for_service('get_joints')
    try:
        get_joints = rospy.ServiceProxy('get_joints', mgi_get_joints)
        resp = get_joints(pose, config)
        return resp.joints
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def test_get_joints():
    pose = Pose()
    pose.position = Point(x=0.2, y=0.3, z=0.2)
    pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    config = String('NUT')
    joints = client_get_joints(pose,config)

    print("-------------------------------")
    print("Test de get_joints :")
    print(pose)
    print(joints)



#   -----------------------------------------------------------------------
#   Services jacobienne
#   -----------------------------------------------------------------------

# get_joints_velocity
def client_get_joints_velocity(joints_position, cmd_vel):
    rospy.wait_for_service('get_joints_velocity')
    try:
        get_joints_velocity = rospy.ServiceProxy('get_joints_velocity', jacobienne_get_joints_velocity)
        resp = get_joints_velocity(joints_position, cmd_vel)
        return resp.joints_velocity
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def test_get_joints_velocity():

    joints_position = JointState()
    joints_position.position = [0.0, 0.0, 0.0, 0.0, np.pi/2, 0.0]

    cmd_vel = Twist()
    cmd_vel.linear = Vector3(0.01, 0.0, 0.0)
    cmd_vel.angular = Vector3(0.0, 0.0, 0.0)

    joints_velocity = client_get_joints_velocity(joints_position, cmd_vel)

    print("-------------------------------")
    print("Test de get_joints_velocity :")
    print(joints_position)
    print(cmd_vel)
    print(joints_velocity)





#   -----------------------------------------------------------------------
#   Programme principal
#   -----------------------------------------------------------------------

def main():

    rospy.init_node("client_test", anonymous=False)
    print "noeud de test des services"
    #test_get_pose()
    #test_move()
    #test_get_homogeneous_transform()
    #test_get_links_pose()
    test_mgd_get_joints()
    #test_get_joints()
    #test_get_joints_velocity()
    rospy.spin()

if __name__ == '__main__':
    main()
