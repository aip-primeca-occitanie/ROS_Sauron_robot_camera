#!/usr/bin/env python
# coding: utf-8

from pyquaternion import Quaternion
import numpy as np

import rospy

from sensor_msgs.msg import JointState
from  geometry_msgs.msg import Pose

from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionFKResponse

# Fonction pour passer d'une matrice de passage homogène à un message geometry_msgs/Pose 
def homogeneous_matrix_to_pose_msg(mat):
    # message creation
    pose = Pose()

    # Extracting the translation
    pose.position.x = mat[0,3]
    pose.position.y = mat[1,3]
    pose.position.z = mat[2,3]

    # Conversion of the rotation matrix into a quaternion
    q = Quaternion(matrix=mat[:3,:3])
    pose.orientation.x = q.x
    pose.orientation.y = q.y
    pose.orientation.z = q.z
    pose.orientation.w = q.w

    return pose

# Fonction pour passer d'un message geometry_msgs/Pose à une matrice de passage homogène
def pose_msg_to_homogeneous_matrix(pose):
    q = Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
    mat = np.eye(4)

    # Extracting the rotation
    mat[:3,:3] = q.rotation_matrix

    # Extracting the translation
    mat[0,3] = pose.position.x
    mat[1,3] = pose.position.y
    mat[2,3] = pose.position.z

    return mat

# fonction pour obtenir un geometry_msgs/Pose de la forward kinematics
def get_fk(joint_state = None, fk_link="tool0", frame_id="base_link"):
        """
        Do an FK call to with.
        :param sensor_msgs/JointState joint_state: JointState message
            containing the full state of the robot.
        :param str or None fk_link: link to compute the forward kinematics for.
        """
        if joint_state is None:
            joint_state = rospy.wait_for_message('/joint_states', JointState)

        req = GetPositionFKRequest()
        req.header.frame_id = 'base_link'
        req.fk_link_names = [fk_link]
        req.robot_state.joint_state = joint_state

        try:
            fk_srv = rospy.ServiceProxy('/compute_fk', GetPositionFK)
            resp = fk_srv.call(req)
            return resp.pose_stamped[0].pose
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            return None

# Fonction pour obtenir la matrice de rotation entre deux vecteur
def rotation_between_vect(a, b):
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)

    if (a == -b).all():
        return -np.eye(3)
    elif (a == b).all():
        return np.eye(3)

    v = np.cross(a, b)
    s = np.linalg.norm(v)
    c = np.dot(a,b)
    
    vx = np.array([ [0, -v[2], v[1]],
                    [v[2], 0, -v[0]],
                    [-v[1], v[0], 0]])

    return np.eye(3) + vx + np.dot(vx,vx) * (1-c) / s**2
