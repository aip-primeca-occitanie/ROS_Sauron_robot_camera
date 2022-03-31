#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from deplacement_robot.srv import Robot_move, Robot_move_predef, Speed_percentage, Robot_do_square, Robot_set_state, Get_fk, Move_predef, Set_etat_loc
import useful_robot
import copy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import Constraints, JointConstraint, PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from industrial_msgs.msg import RobotStatus

#Fichier pour la demo de la release 4

class Move_robot:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.used = False #TODO
        self.speed = 1
        self.state = "LIBRE NON INIT"
        self.etat_loc = "INCONNU"
        self.robot_enable = False

        self.group.set_planner_id("TRRT")
        self.group.set_planning_time(10)

        rospy.Subscriber("camera/camera_ok", Bool, self.camera_state_listener)
        rospy.Subscriber("/robot_status", RobotStatus, self.read_robot_enable)

        self.camera_state = "DECONNECTEE" # DECONNECTEE ou EN MARCHE

        #Limitation de la vitesse    
        args = sys.argv[1:]
        if len(args) >= 1:
            try:
                speed = float(args[0])
                self.set_speed(speed)
            except ValueError:
                rospy.logerr('Error the speed percentage must be float ! No : "' + args[0] + '"')
                rospy.signal_shutdown("Error speed percentage value.")

        #Lancement des servers
        if not rospy.is_shutdown():
            self.move_robot_server()

        # Publication de l etat a 10 Hz
        rate = rospy.Rate(10)
        publisher = rospy.Publisher("robot_state", String, queue_size=10)
        pub_cam = rospy.Publisher("cam_state", String, queue_size=10)
        pub_etat_loc = rospy.Publisher("plaque_state", String, queue_size=10)
        while not rospy.is_shutdown():
            if self.robot_enable :
                publisher.publish(self.state)
            else :
                publisher.publish("STOPPE")
            pub_cam.publish(self.camera_state)
            pub_etat_loc.publish(self.etat_loc)
            rate.sleep()

    def read_robot_enable(self, msg) :
        if msg.motion_possible.val == 0 :
            self.robot_enable = False
        else :
            self.robot_enable = True

    def camera_state_listener(self, msg) :
        if msg.data :
            self.camera_state = "EN MARCHE"
        else :
            self.camera_state = "DECONNECTEE"

    def handler_robot_move(self, msg):
        pose_goal = msg.Pose
        print("Move robot to : " + str(pose_goal))

        for i in range(5):
            self.group.set_pose_target(pose_goal)

            plan = self.group.plan()

            if plan.joint_trajectory.joint_names != [] :
                break

        if plan.joint_trajectory.joint_names == [] :
            print(False)
            return False
        else :
            self.group.execute(plan)
            self.group.stop()
            self.group.clear_pose_targets()
            return True

    def handler_robot_move_parking(self, a):
        return self.move_predef("parking")

    def handler_get_fk(self, a):
        pose = useful_robot.get_fk()
        print(pose)
        print(useful_robot.pose_msg_to_homogeneous_matrix(pose))
        return pose

    def handler_set_speed_perentage(self, msg):
        self.set_speed(msg.speed_percentage)
        return []

    def handler_set_etat_loc(self, msg):
        self.etat_loc = msg.data
	return []

    def set_speed(self, speed):
        self.speed = speed/100
        self.group.set_max_velocity_scaling_factor(self.speed)
        rospy.loginfo("Speed limited to " + str(speed) + "%.")

    def handler_robot_move_lin(self, msg):
        waypoints = [msg.Pose]

        print("Move robot lin to : " + str(msg.Pose))

        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        if plan.joint_trajectory.joint_names == [] :
            print(False)
            return False
        else :
            plan = self.group.retime_trajectory(self.robot.get_current_state(),plan,velocity_scaling_factor = self.speed)
            self.group.execute(plan, wait=True)
            self.group.stop()
            self.group.clear_pose_targets()
            return True

    def move_picture(self, a):
        res = self.move_predef("picture")
        return res

    def move_camera(self, a):
        res = self.move_predef("camera")
        return res

    def set_state(self, msg):
        self.state = msg.state
        return []

    def handler_move_predef(self, msg):
        return self.move_predef(msg.pos)

    def move_robot_server(self):
        s = rospy.Service('move_robot', Robot_move, self.handler_robot_move)
        rospy.loginfo("Server move robot ready !")

        s4 = rospy.Service('move_robot_parking', Robot_move_predef, self.handler_robot_move_parking)
        rospy.loginfo("Server move robot to parking ready !")

        s6 = rospy.Service('move_camera', Robot_move_predef, self.move_camera)
        rospy.loginfo("Server move robot to camera ready !")

        s7 = s = rospy.Service('move_robot_lin', Robot_move, self.handler_robot_move_lin)
        rospy.loginfo("Server move robot ready !")

        s_fk = rospy.Service('get_fk', Get_fk, self.handler_get_fk)

        s_speed = rospy.Service('set_speed_percentage', Speed_percentage, self.handler_set_speed_perentage)

        s_state = rospy.Service("set_robot_state", Robot_set_state, self.set_state)

        s_predef = rospy.Service("move_predef", Move_predef, self.handler_move_predef)

        s_loc = rospy.Service("set_etat_loc", Set_etat_loc, self.handler_set_etat_loc)

        rospy.loginfo("Robot ready to move !")
        

    def move_predef(self, conf_name):
        target = self.group.get_named_target_values(conf_name)

        rospy.loginfo("Move robot to " + conf_name + ".")
        rospy.loginfo("Joint Values " + str(target))

        self.group.set_joint_value_target(target)
        plan = self.group.plan()

        if plan.joint_trajectory.joint_names == [] :
            rospy.logerr("Unreachable position")
            return False
        else :
            self.group.go(wait=True)
            self.group.stop()
            return True





if __name__ == "__main__":
    rospy.init_node('move_robot_server', anonymous=True)

    moveR = Move_robot()

    if not rospy.is_shutdown():
            rospy.spin()
