#!/usr/bin/env python
from tsp_solver.greedy import solve_tsp
import numpy as np
import rospy
import time
from deplacement_robot.srv import Robot_move, Robot_move_predef

def run_pointage(dict_points, diametres, pub=None):
    pub_state(pub, "Debut pointage")

    ### Recuperations des points a parcourir et tri dans l'ordre optimal
    pub_state(pub, "Calcul de la trajectoire")
    # path =  msg_ROS[]
    path = find_path(dict_points, diametres)

    pub_state(pub, "Trajectoire trouvee")

    ### Deplacement du robot
    # Service pour deplacer le robot a un point donne
    move_robot = rospy.ServiceProxy('move_robot', Robot_move)
    # Service pour deplacer le robot a sa position de parking
    move_parking = rospy.ServiceProxy('move_robot_parking', Robot_move_predef)

    pub_state(pub, "Deplacement a la position de parking")
    move_parking()
    pub_state(pub, "Deplacement termine")

    nbPoses = len(path)

    for i,p in enumerate(path):
        # Verification que ROS tourne toujours
        if rospy.is_shutdown():
            exit()

        # Deplacement du robot
        pub_state(pub, "Deplacement a la position " + str(i+1) + "/" + str(nbPoses))
        res = move_robot(p)

        if res:
            pub_state(pub, "Deplacement termine")
            # Si le deplacement est reussi on attend une seconde
            time.sleep(1)
        else :
            # Si le deplacement echou on passe au point suivant
            pub_state(pub, "Position inatteignable. Passage au position suivante")
            rospy.logwarn("Point inatteignable. Passage au point suivant.")

    pub_state(pub, "Pointage fini, retour au parking")

    # Retour a la position de parking
    move_parking()

    pub_state(pub, "Pointage termine")



def find_path(dict_points, diametres):
    # dict_point = {diametre:{(x,y,z):(msg_ROS,conforme)}}
    
    # selected_pose = {(x,y,z):msg_ROS}
    selected_pose = {}

    # Recuperation des positions avec le bon diametre et conforme
    for d in dict_points:
        if d in diametres :
            poses = dict_points[d] # {(x,y,z):(msg_ROS,conforme)}
            for p in poses:
                value = poses[p] # (msg_ROS,conforme)
                if value[1]:
                    selected_pose[p] = value[0]

    # Calcul du chemin optimal
    keys = selected_pose.keys()

    dist_keys = get_distance(keys)
    order = solve_tsp(dist_keys)

    print(dist_keys)

    res = []

    # Tri des position dans l ordre optimal
    for o in order:
        res.append(selected_pose[keys[o]])

    print(res)

    return res 


# Fonction pour passer d une liste de vecteur a une matrice de distance
def get_distance(v):
    v = np.array(v)
    dist_l = []
    for p in v:
        dist_l.append(np.linalg.norm(v-p,axis=1))
    dist_M = np.array(dist_l)
    dist_M = dist_M.T
    return dist_M

def pub_state(pub, msg):
    if not pub is None:
        pub.publish(msg)
