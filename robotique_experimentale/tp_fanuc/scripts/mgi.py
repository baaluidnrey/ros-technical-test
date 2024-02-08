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

import tp_tools

from tp_fanuc.srv import *


class MGI(object):

    #   -----------------------------------------------------------------------
    #   Initialisation des arguments
    #   -----------------------------------------------------------------------

    def __init__(self):

        # Publishers
        self._pub_joints = rospy.Publisher('joints_state', JointState, queue_size=1)

        # Subscribers
        self._sub_pose = rospy.Subscriber('pose', Pose, self.pose_Callback)
        self._sub_config = rospy.Subscriber('config', String, self.config_Callback)

        # Services
        self._service_get_joints = rospy.Service('get_joints', mgi_get_joints, self.get_joints)

        # Coordonnees cartesiennes
        self._pose = Pose()
        self._T = np.identity(4)

        # Coordonnees articulaires
        self._q = np.zeros(6)
        self._joints = JointState()
        self._joints_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

        # Configuration du bras
        self._config = "NUT"

        # Parametrisation du robot
        self._DH = np.zeros([0,0])
        self._nb_axis = 0



    #   -----------------------------------------------------------------------
    #   Modele geometrique inverse
    #   -----------------------------------------------------------------------

    #   Q = compute_MGI(self,T,config)
    #
    #   Description :
    #       Calcule les coordonnes articulaires du robot pour atteindre la pose T
    #
    #   Entrees :
    #       - T         : matrice de transformation rigide entre la base (base_link) et la platine (link_6)
    #       - config    : configuration du robot (parmi les 8 possibles)
    #                       NUT = No-Flip   / Up    / Toward
    #                       FUT = Flip      / Up    / Toward
    #                       NDT = No-Flip   / Down  / Toward
    #                       FDT = Flip      / Down  / Toward
    #                       NUB = No-Flip   / Up    / Backward
    #                       FUB = Flip      / Up    / Backward
    #                       NDB = No-Flip   / Down  / Backward
    #                       FDB = Flip      / Down  / Backward
    #
    #   Sortie :
    #       - Q     : vecteur des coordonnees articulaires [rad ou m]
    #
    #   Remarques :
    #       - calcul des matrices de transformation rigide : compute_homogeneous_transform (voir les commentaires pour l'utilisation)

    def compute_MGI(self,T,config):

        #   Parametres geometriques
        #   --------------------------------------------------------------------
        d6 = self._DH[5,3]

        #   1. Position du centre du poignet dans la base B0 = (O0, [X0, Y0, Z0])
        #   --------------------------------------------------------------------
        P64_R4 = np.array([[0], [0], [-d6], [1]])
        P04_R0 = np.dot(T, P64_R4)

        #   2. Articulation 1
        #   --------------------------------------------------------------------
        #   Bras : Toward
        if config[2] == 'T':
            q1 = 0  # a calculer

        #   Bras : Backward
        if config[2] == 'B':
            q1 = 0  # a calculer


        #   3. Position du centre du poignet dans la base B1 = (O1, [X1, Y1, Z1])
        #   --------------------------------------------------------------------
        x4_R1 = 0   # a calculer
        y4_R1 = 0   # a calculer
        z4_R1 = 0   # a calculer

        P14_R1 = np.array([[x4_R1, y4_R1, z4_R1, 1]]).T



        #   4. Articulation 3
        #   --------------------------------------------------------------------

        #   a. Segment 0304
        alpha = 0   # a calculer
        d = 0       # a calculer

        #   b. Expression de q3

        #   Bras : TOWARD
        if config[2] == 'T':

            # Avant-bras : UP
            if config[1] == 'U':
                q3 = 0      # a calculer

            # Avant-bras : DOWN
            if config[1] == 'D':
                q3 = 0      # a calculer

        #   Bras : BACKWARD
        elif config[2] == 'B':

            # Avant-bras : UP
            if config[1] == 'U':
                q3 = 0      # a calculer

            # Avant-bras : DOWN
            if config[1] == 'D':
                q3 = 0      # a calculer


        #   5. Articulation 2
        #   --------------------------------------------------------------------
        q2 = 0  # a calculer


        #   6. Articulation 4, 5 et 6
        #   --------------------------------------------------------------------

        #   Matrice de rotation R36
        T03 = self.compute_homogeneous_transform([q1, q2, q3],0,3)
        R03 = T03[0:3, 0:3]

        R06 = T[0:3, 0:3]

        R36 = np.identity(3)    # a calculer



        #   Articulation 5
        if config[0] == 'N' :        # poignet NO-FLIP
            q5 = 0  # a calculer

        elif config[0] == 'F' :        # poignet FLIP
            q5 = 0  # a calculer

        # Articulation 4 et 6
        if sin(q5) != 0 :
            q4 = 0  # a calculer
            q6 = 0  # a calculer

        else :  # singularite : choix arbitraire q4 = 0
            q4 = 0  # a calculer
            q6 = 0  # a calculer


        #   7. Resultat : Q
        #   --------------------------------------------------------------------
        Q = np.array([q1, q2, q3, q4, q5, q6])
        return Q



    #   -----------------------------------------------------------------------
    #   Methodes outils
    #   -----------------------------------------------------------------------

    def compute_joints(self):
        self._T = tp_tools.pose_to_matrix(self._pose)
        self._q = self.compute_MGI(self._T, self._config)
        self._joints = tp_tools.array_to_jointstate(self._q, self._joints_names)


    #   -----------------------------------------------------------------------
    #   Callbacks ROS
    #   -----------------------------------------------------------------------

    def pose_Callback(self,data):
        self._pose = data
        self.compute_joints()
        self.joints_broadcast()


    def config_Callback(self,data):
        self._config = data.data
        self.compute_joints()
        self.joints_broadcast()





    #   -----------------------------------------------------------------------
    #   Publications ROS
    #   -----------------------------------------------------------------------

    def joints_broadcast(self):
        self._pub_joints.publish(self._joints)




    #   -----------------------------------------------------------------------
    #   Services ROS
    #   -----------------------------------------------------------------------

    def get_joints(self,req):
        pose = req.pose
        config = req.config.data
        T = tp_tools.pose_to_matrix(pose)
        q = self.compute_MGI(T, config)
        joints = tp_tools.array_to_jointstate(q, self._joints_names)
        return mgi_get_jointsResponse(joints)




    #   -----------------------------------------------------------------------
    #   Appel au modele geometrique direct
    #   -----------------------------------------------------------------------

    def client_get_homogeneous_transform(self,joints,i,j):
        rospy.wait_for_service('get_homogeneous_transform', timeout=1)
        try:
            get_homogeneous_transform = rospy.ServiceProxy('get_homogeneous_transform', mgd_get_homogeneous_transform)
            resp = get_homogeneous_transform(joints,i,j)
            return resp.pose
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    #   T = compute_homogeneous_transform(self,q,ind1,ind2)
    #
    #   Description :
    #       Calcule la matrice de transformation homogene entre les corps ind1 et ind2
    #       Cette fonction fait ensuite appel au service get_homogeneous_transform du noeud mgd
    #
    #   Entrees :
    #       - q : coordonnees articulaires [rad ou m] : exemple [0.0] ou [0.0, 0.0, 0.0]
    #       -ind1 et ind2 : valeurs entieres, indice du repere de base et du repere final
    #       ind2 - ind1 = dim(q)
    #
    #   Sortie :
    #       - T : matrice de transformation rigide

    def compute_homogeneous_transform(self,q,ind1,ind2):
        joints = tp_tools.array_to_jointstate(q, self._joints_names[ind1:ind2])
        i = Int8(ind1)
        j = Int8(ind2)
        pose = self.client_get_homogeneous_transform(joints,i,j)
        T = tp_tools.pose_to_matrix(pose)
        return T



    #   -----------------------------------------------------------------------
    #   Test
    #   -----------------------------------------------------------------------

    def test(self):
        print("--------------------------")
        print("Test du modele inverse")
        print("--------------------------")
        while not rospy.is_shutdown():
            print("\nCoordonnees articulaires (rotoides en [rad], prismatiques en [m]) :")
            Q = input()
            if (Q == "exit" or Q == "quit" or Q == ""):
                print("Fermeture du programme.")
                return
            else:
                Q = np.array(list(map(float,Q.split())))
                print("Q = ")
                print(np.round(Q*1000.0)/1000.0)

            # pose du robot pour les coordonnees articulaires en question
            T = self.compute_homogeneous_transform(Q,0,self._nb_axis)
            if T.all!=None:
                print("\nPose cible = ")
                print(np.round(T*1000.0)/1000.0)

            # calcul du modele inverse pour toutes les configurations
            for config in ["NUT", "NUB", "NDT", "NDB", "FUT", "FUB", "FDT", "FDB"]:
                Q_tmp = self.compute_MGI(T,config)
                T_tmp = self.compute_homogeneous_transform(Q_tmp,0,self._nb_axis)

                print("\n-")
                print("\nconfig : " + config)
                print("\nQ = ")
                print(np.round(Q_tmp*1000.0)/1000.0)
                print("\nT = ")
                print(np.round(T_tmp*1000.0)/1000.0)

            print("\n---")



    #   -----------------------------------------------------------------------
    #   Programme
    #   -----------------------------------------------------------------------

    def clean_shutdown(self):
        print("Noeud arrete : modele_inverse")


    def init(self):
        print("Chargement du robot fanuc_lrmate")
        self._DH = tp_tools.load_robot("fanuc_lrmate")
        self._nb_axis = np.shape(self._DH)[0]
        print("Nombres d'axes : " + str(self._nb_axis))
        print("DH = ")
        print(self._DH)


    def start(self):
        print("Noeud en cours : modele_inverse")



#   -----------------------------------------------------------------------
#   Programme principal
#   -----------------------------------------------------------------------

def main():
    rospy.init_node("mgi", anonymous=False)

    # arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', required=False, action="store_true", help="Tester le modele cinematique en ligne de commande.")
    args = parser.parse_args(rospy.myargv()[1:])

    modele = MGI()
    rospy.on_shutdown(modele.clean_shutdown)
    modele.init()

    # execution du noeud en mode test
    if args.test:
        modele.test()

    # execution du noeud avec ROS
    else:
        modele.start()
        rospy.spin()

if __name__ == '__main__':
    main()
