#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import WrenchStamped, Pose
from std_msgs.msg import Float64MultiArray
import moveit_commander
import moveit_msgs.msg
import sys


class Admittance_control():

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('admittance_control_node')
        self.get_params()
        # self.robot = moveit_commander.RobotCommander()
        # self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "UR10_r/UR_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.move_group

        rospy.Subscriber(self.wrench_topic, WrenchStamped, self.wrench_callback)
        rospy.Subscriber(self.target_pose_topic, Pose, self.pose_callback)

    def update(self):
        
        rate = rospy.Rate(100)
        self.move_group.set_end_effector_link("UR10_r/shoulder_link")
        while not rospy.is_shutdown():
            # get direct kinematics from moveit
            self.actual_pose = self.move_group.get_current_pose().pose
            #print(self.move_group.get_joints())
            #print(self.actual_pose)

    def wrench_callback(self, msg):
        self.wrench = msg.wrench

    def pose_callback(self, msg):
        self.target_pose = msg

    def get_params(self):
        self.wrench_topic = rospy.get_param("~wrench_topic")
        self.target_pose_topic = rospy.get_param("~target_pose_topic")



if __name__ == '__main__':
    admittance_control = Admittance_control()
    admittance_control.update()