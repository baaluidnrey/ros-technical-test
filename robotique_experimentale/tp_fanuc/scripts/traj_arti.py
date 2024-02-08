#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import JointState

def main():

    # --------------------------------------------------------------------------
    # CONFIG ROS
    # --------------------------------------------------------------------------
    rospy.init_node("traj_arti", anonymous=False)
    pub = rospy.Publisher('joints_state', JointState, queue_size=1)

    dt = 0.1
    rate = rospy.Rate(1/dt) # 10Hz


    # --------------------------------------------------------------------------
    # CIBLE
    # --------------------------------------------------------------------------
    print("\nCoordonnees articulaires initiales (rotoides en [rad], prismatiques en [m]) :")
    q0 = np.array(list(map(float,input().split())))
    print("q0 =")
    print(np.round(q0*1000.0)/1000.0)

    print("\nCoordonnees articulaires initiales (rotoides en [rad], prismatiques en [m]) :")
    q_cible = np.array(list(map(float,input().split())))
    print("q cible =")
    print(np.round(q_cible*1000.0)/1000.0)

    print("\nDuree du mouvement [secs] :")
    duree = float(input())
    print("d = " + str(duree) + "s")

    print("\nAppuyer sur Entree pour lancer le mouvement")
    go = (input() == "")


    # --------------------------------------------------------------------------
    # TRAJECTOIRE
    # --------------------------------------------------------------------------

    print("Debut du mouvement")

    while not rospy.is_shutdown():

        # A COMPLETER
        
        rate.sleep()

    print("Mouvement termine")


if __name__ == '__main__':
    main()
