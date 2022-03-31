#!/usr/bin/env python
# coding: utf-8

import rospkg
import rospy
from useful_robot import get_fk, pose_msg_to_homogeneous_matrix
from run_qualite import run_qualite
from run_identification import run_identification
from run_pointage import run_pointage
from run_localisation import run_localisation
from deplacement_robot.msg import Identification, Qualite, Localisation, Result, Forcer_conforme
from std_msgs.msg import Bool, String
from deplacement_robot.srv import Robot_set_state, Robot_move_predef, Set_etat_loc
import moveit_commander
import sys
import time
import numpy as np

import numpy as np

class Robot:
    def __init__(self):
        self.plaque_pos = None
        self.nom_plaque = None
        self.intrinsic = None
        self.distorsion = None
        self.outil_cam = np.array([ [0, -1, 0, -0.035],
                                    [1,  0, 0, -0.035],
                                    [0,  0, 1,  0.05],
                                    [0,  0, 0,  1]])
        self.image_global = None
        self.dic_3D_2D = None
        self.moveit_commander = moveit_commander.roscpp_initialize(sys.argv)
        self.res_qualite = {} # {diametre:{(x,y,z):(msg_ROS,conforme)}}

        rospack = rospkg.RosPack()
        self.step_folder = rospack.get_path("deplacement_robot") + "/plaques"

        self.pub_result = rospy.Publisher("result", Result, queue_size=10)
        self.pub_identification = rospy.Publisher("result/identification", Identification, queue_size=10)
        self.pub_qualite = rospy.Publisher("result/qualite", Qualite, queue_size=10)
        self.pub_localisation = rospy.Publisher("result/localisation", Localisation, queue_size=10)
        
        self.pub_prod_state = rospy.Publisher("production_state", String,  queue_size=10)

        rospy.wait_for_service("set_robot_state")
        self.srv_set_robot_state = rospy.ServiceProxy("set_robot_state", Robot_set_state)

        rospy.Subscriber("/result/ok", Bool, self.result_aquitement)
        rospy.Subscriber("/forcer_conformite", Forcer_conforme, self.forcer_conformite)

        self.aquitement = False

    # Fonction de callback qui recoit l aquitement
    def result_aquitement(self, msg):
        self.aquitement = True

    # Fonction pour envoyer un message jusqu a aquitement
    def spam_result(self, pub, msg):
        self.aquitement = False
        rate = rospy.Rate(10)
        while not self.aquitement and not rospy.is_shutdown():
            #print("spam")
            pub.publish(msg)

    # Fonction pour changer l etat de la production
    def set_robot_state(self, state):
        self.srv_set_robot_state(state)

    def fin_prod(self):
        group = moveit_commander.MoveGroupCommander("manipulator")

        parking = group.get_named_target_values("parking")
        keys = parking.keys()

        parking_list = []
        for k in sorted(keys):
            parking_list.append(parking[k])        

        parking_list = np.round(parking_list, 3)
        curent_state = np.round(group.get_current_joint_values(), 3)

        if np.linalg.norm(np.array(parking_list) - np.array(curent_state)) <= 0.2:
            self.set_robot_state("LIBRE INIT")
        else:
            self.set_robot_state("LIBRE NON INIT")


    def execute_initialisation(self, send_result=True):
        self.set_robot_state("INITIALISATION")

        # Service pour deplacer le robot a sa position de parking
        move_parking = rospy.ServiceProxy('move_robot_parking', Robot_move_predef)

        move_parking()

        rospack = rospkg.RosPack()
        folder_path = rospack.get_path("deplacement_robot")
        try :
            self.intrinsic = np.loadtxt(folder_path+"/saves/intrinsec")
            self.distorsion = np.loadtxt(folder_path+"/saves/distorsion")
            if send_result:
                self.fin_prod()
            return True
        except:
            if send_result:
                self.fin_prod()
            return False

    # Fonction pour lancer la phase de calibration
    def execute_calibration(self):
        self.set_robot_state("CALIBRATION")

        #TODO self.intrinsic, self.distorsion = run_calibration()
        # Only test
        self.intrinsic = np.array([ [4.78103205e+03, 0.00000000e+00, 1.20113948e+03],
                                    [0.00000000e+00, 4.77222528e+03, 1.14533714e+03],
                                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.distorsion = np.array([[ 1.55284357e-01, -3.07067931e+00,  5.16274059e-03, -4.78075223e-03, 1.80663250e+01]])

        rospack = rospkg.RosPack()
        folder_path = rospack.get_path("deplacement_robot")
        np.savetxt(folder_path+"/saves/intrinsec", self.intrinsic)
        np.savetxt(folder_path+"/saves/distorsion", self.distorsion)

        self.fin_prod()

    # Fonction pour lancer la phase de localisation
    def execute_localisation(self, nom_plaque, send_result=True):
        # Reset des resultat de la qualite
        self.res_qualite = {}

        self.nom_plaque = nom_plaque

        msg,H,succes = run_localisation(self.step_folder + "/" + nom_plaque + ".stp", self.distorsion, self.intrinsic, self.outil_cam, pub=self.pub_prod_state)

        srv_etat_loc = rospy.ServiceProxy("set_etat_loc", Set_etat_loc)

        if not succes :
            self.pub_result.publish(False,"Plaque non localisée")
            srv_etat_loc("NOK")
            return False

        self.plaque_pos = H
        srv_etat_loc("OK")

        if send_result:
            self.pub_result.publish(True,"")
            self.spam_result(self.pub_localisation, msg)

        return True

    # Fonction pour lancer la phase d identication
    def execute_identification(self, nom_plaque, diametres, send_result=True):
        if self.intrinsic is None or self.distorsion is None:
            self.execute_initialisation(send_result=False)
        if self.nom_plaque != nom_plaque or self.plaque_pos is None:
            succes = self.execute_localisation(nom_plaque, send_result=False)
            if not succes:
                return False

        # Reset des resultat de la qualite
        self.res_qualite = {}

        msg,self.image_global,self.dic_3D_2D = run_identification(self.plaque_pos, nom_plaque, self.step_folder, diametres, self.intrinsic, self.outil_cam, pub=self.pub_prod_state) #TODO get image global

        if send_result:
            self.pub_result.publish(True,"")
            self.spam_result(self.pub_identification, msg)

        return True

    # Fonction pour lancer la phase de qualites
    def execute_qualite(self, nom_plaque, diametres, send_result=True):
        if self.nom_plaque != nom_plaque or self.plaque_pos is None:
            succes = self.execute_localisation(nom_plaque, send_result=False)
            if not succes :
                return False
            self.execute_identification(nom_plaque, diametres, send_result=False)
        
        msg,res = run_qualite(self.plaque_pos, nom_plaque, self.step_folder, self.image_global, self.dic_3D_2D, diametres=diametres, pub=self.pub_prod_state)

        if send_result:
            self.pub_result.publish(True,"")
            self.spam_result(self.pub_qualite, msg)

        for k in res:
            self.res_qualite[k] = res[k]

        return True

    # Fonction pour lancer la phase de qualites
    def execute_pointage(self, nom_plaque, diametres):
        if self.nom_plaque != nom_plaque or self.plaque_pos is None:
            self.execute_localisation(nom_plaque, send_result=False)
            self.execute_identification(nom_plaque, diametres, send_result=False)
            self.execute_qualite(nom_plaque, diametres, send_result=False)

        diam_non_qual = []
        for d in diametres:
            if not d in self.res_qualite:
                diam_non_qual.append(d)
        if not diam_non_qual == []:
            self.execute_qualite(nom_plaque, diam_non_qual, send_result=False)

        run_pointage(self.res_qualite, diametres, pub=self.pub_prod_state)

        self.pub_result.publish(True,"")

        return True

    #Fonction pour forcer la conformité des tous
    def forcer_conformite(self, msg):
        dic_trous = self.res_qualite.get(msg.diametre*2, None)

        if dic_trous is None :
            rospy.loginfo("Forcer conformite : diametre inconnu")
            return
        
        for xyz in dic_trous : 
            if (np.round([xyz[0],xyz[1]],3) == np.round([msg.x,msg.y],3)).all():
                dic_trous[xyz] = (dic_trous[xyz][0], True)
                rospy.loginfo("Forcer conformite : trou " + str(xyz) + "force conforme")
                return

        rospy.loginfo("Forcer conformite : trou inconnu")
        
