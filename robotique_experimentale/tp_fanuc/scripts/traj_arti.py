#!/usr/bin/env python3

"""
TP de modélisation d'un bras manipulateur à 6DDL.
Utilisé dans le cadre des TPs de robotique expérimentale de la spécialité robotique de Polytech Sorbonne.
Ce code est une refonte en Python et ROS d'un code sous Matlab précédemment mis en place par Guillaume Morel et Marie-Aude Vitrani.

traj_arti.py: interpolation articulaire
"""
__author__ = "Aline Baudry"
__copyright__ = "2021, Polytech Sorbonne"
__credits__ = ["Aline Baudry", "Guillaume Morel", "Marie-Aude Vitrani"]
__maintainer__ = "Aline Baudry"
__email__ = "aline.baudry@sorbonne-universite.fr"
import numpy as np
import rospy
from sensor_msgs.msg import JointState
def main():

  # --------------------------------------------------------------------------
  # ROS CONFIGURATION
  # --------------------------------------------------------------------------
  rospy.init_node("traj_arti", anonymous=False)
  pub = rospy.Publisher('joints_state', JointState, queue_size=1)

  dt = 0.1  # Time step in seconds
  rate = rospy.Rate(1/dt)  # Publish rate at 10Hz

  # --------------------------------------------------------------------------
  # TARGET
  # --------------------------------------------------------------------------
  print(JointState())
  print("\nInitial joint coordinates (rotational in [rad], prismatic in [m]):")
  q0 = np.array(list(map(float, input().split())))
  print("q0:")
  print(np.round(q0 * 1000.0) / 1000.0)  # Print with 3 decimal places

  print("\nTarget joint coordinates (rotational in [rad], prismatic in [m]):")
  q_cible = np.array(list(map(float, input().split())))
  print("q_cible:")
  print(np.round(q_cible * 1000.0) / 1000.0)  # Print with 3 decimal places

  print("\nMovement duration [seconds]:")
  duree = float(input())
  print("d = " + str(duree) + "s")

  print("\nPress Enter to start the movement")
  go = (input() == "")

  # --------------------------------------------------------------------------
  # TRAJECTORY
  # --------------------------------------------------------------------------
  print("Movement started")

  t = 0  # Current time
  while not rospy.is_shutdown() and t < duree:

    # Interpolate joint positions for current time step
    q = q0 + (q_cible - q0) * t / duree

    # Publish joint states
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    joint_state.position = q.tolist()  # Convert numpy array to list

    pub.publish(joint_state)
    print(joint_state)

    t += dt
    rate.sleep()

  print("Movement completed")


if __name__ == '__main__':
  main()

