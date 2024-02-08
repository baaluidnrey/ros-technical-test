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


class Modele(object):

    #   -----------------------------------------------------------------------
    #   Initialisation des arguments
    #   -----------------------------------------------------------------------

    def __init__(self):


        # Publishers
        self._pub_joints = rospy.Publisher('joints_state', JointState, queue_size=1)

        # Subscribers
        self._sub_joints = rospy.Subscriber('joints_state', JointState, self.joints_Callback)
        self._sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_Callback)

        # Services
        self._service_get_joints_velocity = rospy.Service('get_joints_velocity', jacobienne_get_joints_velocity, self.get_joints_velocity)

        # Vitesse cartesienne
        self._linear = np.zeros(3)
        self._angular = np.zeros(3)

        # Frequence d'execution
        # Le noeud ne publie pas lorsque la vitesse est nulle.
        # Sinon il publie a une frequence donnee
        self._motion = False
        self._dt = 0.01
        self._rate = rospy.Rate(1/self._dt)


        # Coordonnees articulaires
        self._q = np.zeros(6)
        self._joints = JointState()
        self._joints_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


        # Parametrisation du robot
        self._DH = np.zeros([0,0])
        self._nb_axis = 0



    #   -----------------------------------------------------------------------
    #   Jacobienne
    #   -----------------------------------------------------------------------

    #   J = compute_J(self,Q)
    #
    #   Description :
    #       Calcule la jacobienne du robot dans la configuration articulaire Q.
    #
    #   Entrees :
    #       - Q     : vecteur des coordonnees articulaires [rad ou m]
    #
    #   Sortie :
    #       - J     : matrice jacobienne [dimension: 6 x nb_axis]
    #
    #   Remarques :
    #       - ATTENTION aux indices utilises
    #       - les types de liaisons (sigma) sont dans self._DH


    def compute_J(self,Q):

        if len(Q) != self._nb_axis:
            print("Le vecteurs des coordonnees articulaires ne correspond pas au nombre d'axes")
            return

        else:
            # Calcul des matrices de transformation rigides de la base a chacun des corps
            liste_T0i = self.compute_links_pose(Q)

            J = np.zeros((6,self._nb_axis))

        return J



    #   q_vel = compute_q_velocity(self,linear,angular,q)
    #
    #   Description :
    #       Calcule les vitesses articulaires pour obtenir des vitesses operationnelles donnees.
    #
    #   Entrees :
    #       - linear    : vecteur vitesse lineaire [m/s]
    #       - angular   : vecteur vitesse angulaire [rad/s]
    #       - q         : vecteur des coordonnees articulaires [rad ou m]
    #
    #   Sortie :
    #       - q_vel     : vecteur des vitesses articulaires [m/s ou rad/s]
    #
    #   Remarque :
    #       La sortie doit etre un array de dimension (nb_axis,) pour que la suite du code fonctionne (np.shape() pour le verifier)
    #           Exemple : q_vel = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def compute_q_velocity(self,linear,angular,q):

        q_vel = np.zeros([6,])

        return q_vel




    #   q = compute_q(self,linear,angular,q0,dt)
    #
    #   Description :
    #       Integration des vitesses articulaires a une certaine frequence pour l'envoi des coordonnees articulaires.
    #
    #   Entrees :
    #       - linear : vecteur vitesse lineaire [m/s]
    #       - angular : vecteur vitesse angulaire [rad/s]
    #       - q0 : vecteur des coordonnees articulaires initiales [rad ou m]
    #       - dt : temps d'echantillonnage [s]
    #
    #   Sortie :
    #       - q : vecteur des coordonnees articulaires [m ou rad]
    #
    #   Remarques :
    #       - La sortie doit etre un array de dimension (nb_axis,) pour que la suite du code fonctionne (np.shape() pour le verifier)
    #           Exemple : q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def compute_q(self,linear,angular,q0,dt):

        q = np.zeros([self._nb_axis,])

        return q




    #   -----------------------------------------------------------------------
    #   Methodes outils
    #   -----------------------------------------------------------------------

    def compute_joints(self):
        self._q = self.compute_q(self._linear, self._angular, self._q, self._dt)
        self._joints = tp_tools.array_to_jointstate(self._q, self._joints_names)


    #   -----------------------------------------------------------------------
    #   Callbacks ROS
    #   -----------------------------------------------------------------------

    def joints_Callback(self,data):
        self._q = data.position
        self._joints = data


    def cmd_vel_Callback(self,data):
        self._linear = np.array([ data.linear.x, data.linear.y, data.linear.z ])
        self._angular = np.array([ data.angular.x, data.angular.y, data.angular.z ])

        # si consigne nulle, on stoppe le mouvement
        v_nulle = np.array([0.0, 0.0, 0.0])
        if np.array_equal(self._linear, v_nulle) and np.array_equal(self._angular, v_nulle):
            self._motion = False

        else:
            self._motion = True



    #   -----------------------------------------------------------------------
    #   Publications ROS
    #   -----------------------------------------------------------------------

    def joints_broadcast(self):
        self._pub_joints.publish(self._joints)



    #   -----------------------------------------------------------------------
    #   Services ROS
    #   -----------------------------------------------------------------------

    def get_joints_velocity(self,req):
        q = req.joints_position.position
        linear = np.array([ req.cmd_vel.linear.x, req.cmd_vel.linear.y, req.cmd_vel.linear.z ])
        angular = np.array([ req.cmd_vel.angular.x, req.cmd_vel.angular.y, req.cmd_vel.angular.z ])

        q_vel = self.compute_q_velocity(linear,angular,q)

        joints = JointState()
        joints.header.stamp = rospy.Time.now()
        joints.name = self._joints_names
        joints.velocity = q_vel

        return jacobienne_get_joints_velocityResponse(joints)





    #   -----------------------------------------------------------------------
    #   Appel au modele geometrique direct
    #   -----------------------------------------------------------------------

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

    def client_get_homogeneous_transform(self,joints,i,j):
        rospy.wait_for_service('get_homogeneous_transform')
        try:
            get_homogeneous_transform = rospy.ServiceProxy('get_homogeneous_transform', mgd_get_homogeneous_transform)
            resp = get_homogeneous_transform(joints,i,j)
            return resp.pose
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def compute_homogeneous_transform(self,q,ind1,ind2):
        joints = tp_tools.array_to_jointstate(q, self._joints_names[ind1:ind2-1])
        i = Int8(ind1)
        j = Int8(ind2)
        pose = self.client_get_homogeneous_transform(joints,i,j)
        T = tp_tools.pose_to_matrix(pose)
        return T


    #   links_pose = compute_links_pose(self,joints)
    #
    #   Description :
    #       Calcule la pose de chacun des corps dans le repere de base
    #       Cette fonction fait ensuite appel au service mgd_get_links_pose du noeud mgd
    #
    #   Entrees :
    #       - joints : coordonnees articulaires [rad ou m]
    #
    #   Sortie :
    #       - links_pose : liste des poses

    def client_get_links_pose(self,joints):
        rospy.wait_for_service('get_links_pose', timeout=1)
        try:
            get_links_pose = rospy.ServiceProxy('get_links_pose', mgd_get_links_pose)
            resp = get_links_pose(joints)
            return resp.links_pose
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def compute_links_pose(self,q):
        joints = tp_tools.array_to_jointstate(q, self._joints_names)
        links_pose = self.client_get_links_pose(joints) # TFMessage
        liste_T0i = self._nb_axis*[np.identity(4)]      # liste de matrices
        for axis in range(0,self._nb_axis):
            tf = links_pose.transforms[axis].transform
            liste_T0i[axis] = tp_tools.tf_to_matrix(tf)
        return liste_T0i



    #   joints = client_mgd_get_joints(self)
    #
    #   Description :
    #       Fait appel au service mgd_get_joints pour recuperer la position des articulations
    #
    #   Entrees :
    #
    #   Sortie :
    #       - joints : coordonnees articulaires [rad ou m]

    def client_mgd_get_joints(self):
        rospy.wait_for_service('mgd_get_joints', timeout=1)
        try:
            get_joints = rospy.ServiceProxy('mgd_get_joints', mgd_get_joints)
            resp = get_joints()
            return resp.joints
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)



    #   -----------------------------------------------------------------------
    #   Test
    #   -----------------------------------------------------------------------

    def test(self):
        print("--------------------------")
        print("Test du modele cinematique")
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

            J = self.compute_J(Q)
            if J.all!=None:
                print("\nJacobienne :")
                print(np.round(J*1000.0)/1000.0)
                print("\n---")




    #   -----------------------------------------------------------------------
    #   Programme
    #   -----------------------------------------------------------------------

    def clean_shutdown(self):
        print("Noeud arrete : jacobienne")


    def init(self):
        print("\nChargement du robot fanuc_lrmate")
        self._DH = tp_tools.load_robot("fanuc_lrmate")
        self._nb_axis = np.shape(self._DH)[0]
        print("Nombres d'axes : " + str(self._nb_axis))
        print("DH = ")
        print(self._DH)

        print("\nPosition articulaire initiale")
        self._joints = self.client_mgd_get_joints()
        self._q = self._joints.position
        print(self._q)


    def start(self):
        print("Noeud en cours : jacobienne")
        while not rospy.is_shutdown():

            if self._motion:
                self.compute_joints()
                self.joints_broadcast()

            self._rate.sleep()




#   -----------------------------------------------------------------------
#   Programme principal
#   -----------------------------------------------------------------------

def main():
    rospy.init_node("jacobienne", anonymous=False)

    # arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', required=False, action="store_true", help="Tester le modele cinematique en ligne de commande.")
    parser.add_argument('--dt', required=False, type=float, help="Specifier le temps d'echantillonnage.")
    parser.add_argument('--robot', required=False, help="Robot a charger.")
    args = parser.parse_args(rospy.myargv()[1:])

    modele = Modele()
    rospy.on_shutdown(modele.clean_shutdown)
    modele.init()

    # temps d'echantillonnage
    if args.dt != None:
        modele._dt = args.dt
        modele._rate = rospy.Rate(1/modele._dt)
        print("Temps d'echantillonnage : " + str(modele._dt) + "s")

    # robot
    if args.robot != None:
        print("Chargement du robot " + args.robot)
        modele._DH = tp_tools.load_robot(args.robot)
        modele._nb_axis = np.shape(modele._DH)[0]
        print("Nombres d'axes : " + str(modele._nb_axis))
        print("DH = ")
        print(modele._DH)

    # execution du noeud en mode test
    if args.test:
        modele.test()

    # execution du noeud avec ROS
    else:
        modele.start()
        rospy.spin()

if __name__ == '__main__':
    main()
