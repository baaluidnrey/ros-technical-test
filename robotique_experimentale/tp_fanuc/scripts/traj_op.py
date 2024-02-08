#!/usr/bin/env python3
import numpy as np
import rospy
from math import *
import tf # pour faire les transformations
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tp_tools


def eulerZYX_to_matrix(alpha,beta,gamma):

    R = np.identity(4)
    return R



def main():

    # --------------------------------------------------------------------------
    # CONFIG ROS
    # --------------------------------------------------------------------------


    # --------------------------------------------------------------------------
    # CIBLE
    # --------------------------------------------------------------------------
    print("\nPose initiale (position en [m], angles d'Euler ZYX en [rad]) :")
    x0 = np.array(list(map(float,input().split())))
    print("pose initiale =")
    print(np.round(x0*1000.0)/1000.0)

    print("\nPose cible (position en [m], angles d'Euler ZYX en [rad]) :")
    x_cible = np.array(list(map(float,input().split())))
    print("pose cible =")
    print(np.round(x_cible*1000.0)/1000.0)

    print("\nDuree du mouvement [secs] :")
    duree = float(input())
    print("d = " + str(duree) + "s")

    print("\nAppuyer sur Entree pour lancer le mouvement")
    go = (input() == "")


    # --------------------------------------------------------------------------
    # TRAJECTOIRE
    # --------------------------------------------------------------------------



if __name__ == '__main__':
    main()
