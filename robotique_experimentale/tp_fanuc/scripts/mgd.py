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
from tf2_msgs.msg import TFMessage

import tp_tools

from tp_fanuc.srv import *


class MGD(object):

    #   -----------------------------------------------------------------------
    #   Initialisation des arguments
    #   -----------------------------------------------------------------------

    def __init__(self):

        # boucle ROS
        self._dt = 0.1
        self._rate = rospy.Rate(1/self._dt)

        # publications des tf
        self._tf_broadcast = tf2_ros.TransformBroadcaster()
        self._static_tf_broadcast = tf2_ros.StaticTransformBroadcaster()
        self._frame_id = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5"]
        self._child_id = ["link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]
        self._seq_tf = 0
        self._seq_path = 0

        # publication de la trajectoire
        self._pub_path = rospy.Publisher('path', Path, queue_size=1)

        # subscribers
        self._sub_joints = rospy.Subscriber('joints_state', JointState, self.joints_Callback)
        self._sub_manage_path = rospy.Subscriber('manage_path', String, self.manage_path_Callback)

        # services
        self._service_get_pose = rospy.Service('get_pose', mgd_get_pose, self.get_pose)
        self._service_move = rospy.Service('move', mgd_move, self.move)
        self._service_get_homogeneous_transform = rospy.Service('get_homogeneous_transform', mgd_get_homogeneous_transform, self.get_homogeneous_transform)
        self._service_get_links_pose = rospy.Service('get_links_pose', mgd_get_links_pose, self.get_links_pose)
        self._service_get_joints = rospy.Service('mgd_get_joints', mgd_get_joints, self.get_joints)
        self._service_manage_path = rospy.Service('manage_path', mgd_manage_path, self.manage_path)

        # joints
        self._joints_state = np.zeros(6)
        self._joints_data = JointState()
        self._newPose = False

        # matrices de transformation rigide
        self._liste_Ti = 6*[np.identity(4)]
        self._T06 = np.identity(4)
        self._pose = Pose()

        # trajectoire
        self._path = Path()

        # Parametrisation du robot
        self._DH = np.zeros([0,0])
        self._nb_axis = 0



    #   -----------------------------------------------------------------------
    #   Modele geometrique direct
    #   -----------------------------------------------------------------------

    #   T = compute_Ti(self,dh,q)
    #
    #   Description :
    #       Calcule la matrice de transformation rigide d'un corps par rapport a un autre.
    #
    #   Entrees :
    #       - q : coordonnee articulaire [rad ou m]
    #
    #       - dh : parametres geometriques entre les deux corps suivant la convention de Denavit-Hartenberg modifiee
    #           [sigma, a, alpha, d, theta]
    #           type : array de dimension (5,) (np.shape() pour le verifier)
    #
    #   Sortie :
    #       - T : matrice de transformation rigide
    #           type : array de dimension (4, 4) (np.shape() pour le verifier)


    def compute_Ti(self,dh,q):

        T = np.identity(4)

        return T


    #   T = compute_T(self,Q,i,j)
    #
    #   Description :
    #       Calcule la matrice de transformation rigide entre le corps i et le corps j.
    #
    #   Entrees :
    #       - Q : vecteur des coordonnees articulaires des articulations entre le corps i et le corps j [rad ou m]
    #           type : array de dimension (j-i,) (np.shape() pour le verifier)
    #
    #   Sortie :
    #       - T : matrice de transformation rigide
    #           type : array de dimension (4, 4) (np.shape() pour le verifier)

    def compute_T(self,Q,i,j):

        if len(Q) != (j-i):
            print("Le vecteurs des coordonnees articulaires ne correspond pas au nombre d'axes")
            return

        else:
            T = np.identity(4)
            for axis in range(i,j):
                T = T   # a calculer

            return T


    #   T = compute_robot_state(self,Q)
    #
    #   Description :
    #       Calcule l'etat du robot :
    #           - matrice de transformation rigide entre le corps 0 et le corps terminal
    #           - liste des transformations entre les corps i-1 et i
    #           - liste des trasformations entre la base et le corps i
    #
    #   Entrees :
    #       - Q : vecteur des coordonnees articulaires [rad ou m]
    #           type : array de dimension (self._nb_axis,) (np.shape() pour le verifier)
    #
    #   Sortie :
    #       - T : matrice de transformation rigide de la base au corps terminal
    #           type : array de dimension (4, 4) (np.shape() pour le verifier)
    #
    #       - liste_Ti : liste des matrices de transformation rigide entre les corps i et i+1
    #           type : liste de self._nb_axis arrays de dimension (4, 4)
    #
    #       - liste_T0i : liste des matrices de transformation rigide entre la base et le corps i
    #           type : liste de self._nb_axis arrays de dimension (4, 4)

    def compute_robot_state(self,Q):

        liste_Ti = self._nb_axis*[np.identity(4)]
        liste_T0i  = self._nb_axis*[np.identity(4)]
        T0i = np.identity(4)

        for axis in range(0,self._nb_axis):

            Ti = np.identity(4)     # a calculer
            T0i = T0i               # a calculer

            liste_Ti[axis] = Ti
            liste_T0i[axis] = T0i


        return T0i, liste_Ti, liste_T0i



    #   -----------------------------------------------------------------------
    #   Methodes outils
    #   -----------------------------------------------------------------------

    #   t = compute_tf(self,frame_id,child_frame_id,T)
    #
    #   Description :
    #       Creation d'un message de type geometry_msgs/TransformStamped
    #
    #   Entrees :
    #       - frame_id          : nom du repere parent
    #       - child_frame_id    : nom du repere enfant
    #       - T                 : matrice de transformation rigide entre le repere parent et le repere enfant
    #
    #   Sortie :
    #       - t : geometry_msgs/TransformStamped
    #
    # tf Python API :
    # - http://docs.ros.org/melodic/api/tf/html/python/tf_python.html
    # - http://docs.ros.org/melodic/api/tf/html/python/transformations.html#examples

    def compute_tf(self,frame_id,child_frame_id,T):

        t = TransformStamped()
        t.header.seq = self._seq_tf
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id

        t.transform.translation = Vector3(T[0,3], T[1,3], T[2,3])
        q = tf.transformations.quaternion_from_matrix(T)
        t.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])

        return t


    #   compute_path(self)
    #
    #   Description :
    #       Ajout de la pose courante a la trajectoire

    def compute_path(self):

        self._path.header.seq = self._seq_path
        self._path.header.stamp = rospy.Time.now()
        self._path.header.frame_id = "base_link"

        pose_stamped = PoseStamped()
        pose_stamped.header.seq = self._seq_path
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = self._pose
        self._path.poses.append(pose_stamped)


    #   clear_path(self)
    #
    #   Description :
    #       Suppression des poses de la trajectoire

    def clear_path(self):
        self._seq_path = 0
        self._path.poses = []




    #   -----------------------------------------------------------------------
    #   Callbacks ROS
    #   -----------------------------------------------------------------------

    #   joints_Callback(self,data)
    #
    #   Description :
    #       A chaque nouveau message sur le topic /joints_state, les positions articulaires du robots, ainsi que la trajectoire sont mises a jour

    def joints_Callback(self,data):
        self._joints_data = data
        self._joints_state = data.position
        self._T06, self._liste_Ti, _ = self.compute_robot_state(self._joints_state)
        self._pose = tp_tools.matrix_to_pose(self._T06)
        self.compute_path()
        self._newPose = True



    #   manage_path_Callback(self,data)
    #
    #   Description :
    #       A chaque nouveau message sur le topic /manage_path, ce callback est appele
    #       Ici, il existe uniquement la commande "clear".
    #       Un service permet de faire la meme chose

    def manage_path_Callback(self,data):
        command = data.data

        if command == "clear":
            self.clear_path()
            self.compute_path()
            self.path_broadcast()




    #   -----------------------------------------------------------------------
    #   Publications ROS
    #   -----------------------------------------------------------------------

    #   tf_broadcast(self)
    #
    #   Description :
    #       Publication des reperes de chaque corps du robot ('tf') pour affichage dans rviz

    def tf_broadcast(self):

        tf_array = 6*[TransformStamped()]

        for axis in range(0,6):
            tf_array[axis] = self.compute_tf(self._frame_id[axis], self._child_id[axis], self._liste_Ti[axis])

        self._tf_broadcast.sendTransform(tf_array)
        self._seq_tf += 1


    #   static_tf_broadcast(self)
    #
    #   Description :
    #       Publication des reperes statiques

    def static_tf_broadcast(self):
        T = np.identity(4)
        t = self.compute_tf("link_6", "tool0", T)
        self._static_tf_broadcast.sendTransform(t)


    #   path_broadcast(self)
    #
    #   Description :
    #       Publication de la trajectoire de l'effecteur du robot

    def path_broadcast(self):
        self._pub_path.publish(self._path)
        self._seq_path += 1




    #   -----------------------------------------------------------------------
    #   Services ROS
    #   -----------------------------------------------------------------------

    #   get_pose(self,req)
    #
    #   Description :
    #       Service qui permet d'obtenir la pose du robot pour une configuration articulaire donnee
    #
    #   Entrees (Requete) :
    #       - sensor_msgs/JointState joints_state : configuration articulaire
    #
    #   Sorties (Reponse) :
    #       - geometry_msgs/Pose pose : pose de l'effecteur du robot

    def get_pose(self,req):
        Q = req.joints_state.position
        T = self.compute_T(Q,0,self._nb_axis)
        pose = tp_tools.matrix_to_pose(T)
        return mgd_get_poseResponse(pose)


    #   move(self,req)
    #
    #   Description :
    #       Service qui permet de deplacer le robot a une configuration articulaire donnee
    #
    #   Entrees (Requete) :
    #       - sensor_msgs/JointState joints_state : configuration articulaire
    #
    #   Sorties (Reponse) :
    #       - std_msgs/Bool motion_done : booleen qui specifie que le mouvement a ete realise

    def move(self,req):
        self._joints_state = req.joints_state.position
        self._T06, self._liste_Ti, _ = self.compute_robot_state(self._joints_state)
        self.tf_broadcast()
        motion_done = Bool(True)
        return mgd_moveResponse(motion_done)


    #   get_homogeneous_transform(self,req)
    #
    #   Description :
    #       Service qui permet d'obtenir la transformation rigide entre le corps i et le corps j
    #
    #   Entrees (Requete) :
    #       - sensor_msgs/JointState joints_state : vecteur des coordonnees articulaires entre le corps i et le corps j
    #       - std_msgs/Int8 i : indice du corps i
    #       - std_msgs/Int8 j : indice du corps j
    #
    #   Sorties (Reponse) :
    #       - geometry_msgs/Pose pose : transformation rigide entre le corps i et le corps j sous forme de pose

    def get_homogeneous_transform(self,req):
        Q = req.joints_state.position
        i = req.i.data
        j = req.j.data
        T = self.compute_T(Q,i,j)
        pose = tp_tools.matrix_to_pose(T)
        return mgd_get_homogeneous_transformResponse(pose)


    #   manage_path(self,req)
    #
    #   Description :
    #       Service qui permet d'effacer la trajectoire de l'effecteur
    #
    #   Entrees (Requete) :
    #       - std_msgs/String command : message de commande
    #
    #   Sorties (Reponse) :
    #       - std_msgs/Bool done : booleen pour indiquer que tout s'est bien passe

    def manage_path(self,req):
        command = req.command.data

        if command == "clear":
            self.clear_path()
            self.compute_path()
            self.path_broadcast()

        done = Bool(True)
        return mgd_manage_pathResponse(done)



    #   get_links_pose(self,req)
    #
    #   Description :
    #       Service qui permet de recuperer la pose de chacun des corps du robot par rapport a la base du robot
    #
    #   Entrees (Requete) :
    #       - sensor_msgs/JointState joints_state : vecteur des coordonnees articulaires
    #
    #   Sorties (Reponse) :
    #       - tf2_msgs/TFMessage : liste de geometry_msgs/Transform

    def get_links_pose(self,req):
        Q = req.joints_state.position

        tf_msg = TFMessage()
        tf_array = self._nb_axis*[TransformStamped()]
        _, _, liste_T0i = self.compute_robot_state(Q)

        for axis in range(0,self._nb_axis):
            tf_array[axis] = self.compute_tf("base_link", self._child_id[axis], liste_T0i[axis])

        tf_msg.transforms = tf_array
        return mgd_get_links_poseResponse(tf_msg)



    #   get_joints(self,req)
    #
    #   Description :
    #       Service qui permet de recuperer l'etat des joints
    #
    #   Entrees (Requete) :
    #
    #   Sorties (Reponse) :
    #       - sensor_msgs/JointState joints

    def get_joints(self,req):
        joints = self._joints_data
        return mgd_get_jointsResponse(joints)



    #   -----------------------------------------------------------------------
    #   Test
    #   -----------------------------------------------------------------------

    def test(self):
        print("--------------------------")
        print("Test du modele geometrique")
        print("--------------------------")
        while not rospy.is_shutdown():
                
            # Sélection de la méthode à tester    
            print("\nQue souhaitez-vous faire ? :")
            print(" 0: Quitter le programme")
            print(" 1: Test de compute_T")
            print(" 2: Test de compute_robot_state")

            x = input()
            
            if x=="0":
                print("Fermeture du programme.")
                return
            
            # Saisie des coordonnées articulaires (commun aux deux tests)
            print("\nCoordonnees articulaires (rotoides en [rad], prismatiques en [m]) :")
            Q = input()
            Q = np.array(list(map(float,Q.split())))
            print("Q = ")
            print(np.round(Q*1000.0)/1000.0)
            
            # Test de compute_T
            if x=="1":
                print("\nRepere de base")
                i = int(input())
                print("i = " + str(i))

                print("\nRepere d'arrivee")
                j = int(input())
                print("j = " + str(j))

                T = self.compute_T(Q,i,j)
                if T.all!=None:
                    print("\nMatrice de transformation rigide entre le corps " + str(i) + " et le corps " + str(j) + " :")
                    print(np.round(T*1000.0)/1000.0)
                    print("\n---")

            # Test de compute_robot_state
            elif x=="2":
                T0i, liste_Ti, liste_T0i = self.compute_robot_state(Q)
                print("\nListe des matrices de transformation rigide entre les corps i et i+1 :")
                for T in liste_Ti:
                    print(np.round(T*1000.0)/1000.0)
                    print("\n")
                print("\nListe des matrices de transformation rigide entre la base et le corps i :")
                for T in liste_T0i:
                    print(np.round(T*1000.0)/1000.0)
                    print("\n")
                print("\n---")



    #   -----------------------------------------------------------------------
    #   Programme
    #   -----------------------------------------------------------------------

    def clean_shutdown(self):
        print("Noeud arrete : modele_direct")


    def init(self):
        print("Chargement du robot fanuc_lrmate")
        self._DH = tp_tools.load_robot("fanuc_lrmate")
        self._nb_axis = np.shape(self._DH)[0]
        print("Nombres d'axes : " + str(self._nb_axis))
        print("DH = ")
        print(self._DH)

        # joints
        self._joints_data.position = np.zeros(self._nb_axis)

        print("Publication des transformations rigides statiques")
        self.static_tf_broadcast()
        self._T06, self._liste_Ti, _ = self.compute_robot_state(self._joints_state)
        self.compute_path()


    def start(self):

        print("Noeud en cours : modele_direct")

        while not rospy.is_shutdown():

            if self._newPose:
                self.path_broadcast()
                self._newPose = False

            self.tf_broadcast()
            self._rate.sleep()





#   -----------------------------------------------------------------------
#   Programme principal
#   -----------------------------------------------------------------------

def main():
    rospy.init_node("mgd", anonymous=False)

    # arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', required=False, action="store_true", help="Tester le modele cinematique en ligne de commande.")
    parser.add_argument('--dt', required=False, type=float, help="Specifier le temps d'echantillonnage.")
    parser.add_argument('--robot', required=False, help="Robot a charger.")
    args = parser.parse_args(rospy.myargv()[1:])

    modele = MGD()
    rospy.on_shutdown(modele.clean_shutdown)
    modele.init()

    # temps d'echantillonnage
    if args.dt != None:
        modele._dt = args.dt
        modele._rate = rospy.Rate(1/modele._dt)
        print("Periode d'echantillonnage : " + str(modele._dt) + "s")

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
