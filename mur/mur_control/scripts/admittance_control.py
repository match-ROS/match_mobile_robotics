#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import WrenchStamped, Pose, Wrench
from std_msgs.msg import Float64MultiArray
import moveit_commander
import moveit_msgs.msg
import sys
from tf import transformations


class Admittance_control():

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('admittance_control_node')
        self.get_params()
        self.actual_wrench = Wrench()
        self.target_wrench = Wrench()
        self.target_pose = Pose()
        self.actual_pose = Pose()
        

        rospy.Subscriber(self.wrench_topic, WrenchStamped, self.wrench_callback)
        rospy.Subscriber(self.target_pose_topic, Pose, self.target_pose_callback)
        rospy.Subscriber(self.actual_pose_topic, Pose, self.actual_pose_callback)
        rospy.Subscriber(self.target_wrench_topic, WrenchStamped, self.target_wrench_callback)
        rospy.sleep(1.0) # wait for the subscribers to be ready
        

    def update(self):
        
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():

            # Calculate the error between the current pose and the target pose
            pose_error = self.pose_error(self.actual_pose, self.target_pose)

            # Calculate the error betrween the current wrench and the target wrench
            wrench_error = self.wrench_error(self.actual_wrench, self.target_wrench)

            # Calculate the desired force
            desired_compliance = self.calculate_desired_force(pose_error, wrench_error)

            rate.sleep()

    def pose_error(self, actual_pose, target_pose):
        pose_error = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pose_error[0]   = target_pose.position.x - actual_pose.position.x
        pose_error[1]   = target_pose.position.y - actual_pose.position.y
        pose_error[2]   = target_pose.position.z - actual_pose.position.z
        # convert quaternion to euler angles
        [act_roll, act_pitch, act_yaw] = transformations.euler_from_quaternion([actual_pose.orientation.x, actual_pose.orientation.y, actual_pose.orientation.z, actual_pose.orientation.w])
        [tar_roll, tar_pitch, tar_yaw] = transformations.euler_from_quaternion([target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w])
        pose_error[3]   = tar_roll - act_roll
        pose_error[4]   = tar_pitch - act_pitch
        pose_error[5]   = tar_yaw - act_yaw
        return pose_error

    def wrench_error(self, actual_wrench, target_wrench):
        wrench_error = Wrench()
        wrench_error.force.x = target_wrench.force.x - actual_wrench.force.x
        wrench_error.force.y = target_wrench.force.y - actual_wrench.force.y
        wrench_error.force.z = target_wrench.force.z - actual_wrench.force.z
        wrench_error.torque.x = target_wrench.torque.x - actual_wrench.torque.x
        wrench_error.torque.y = target_wrench.torque.y - actual_wrench.torque.y
        wrench_error.torque.z = target_wrench.torque.z - actual_wrench.torque.z    
        return wrench_error

    def calculate_desired_force(self, pose_error, current_wrench):
        desired_force = Wrench()
        desired_force.force.x = pose_error[0] * self.admittance_gain_force_x
        desired_force.force.y = pose_error[1] * self.admittance_gain_force_y
        desired_force.force.z = pose_error[2] * self.admittance_gain_force_z
        desired_force.torque.x = pose_error[3] * self.admittance_gain_torque_x
        desired_force.torque.y = pose_error[4] * self.admittance_gain_torque_y
        desired_force.torque.z = pose_error[5] * self.admittance_gain_torque_z
        return desired_force

    def desired_compliance(self, desired_force, current_wrench):
        desired_compliance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        

        desired_compliance.force.x = desired_force.force.x - current_wrench.force.x
        desired_compliance.force.y = desired_force.force.y - current_wrench.force.y
        desired_compliance.force.z = desired_force.force.z - current_wrench.force.z
        desired_compliance.torque.x = desired_force.torque.x - current_wrench.torque.x
        desired_compliance.torque.y = desired_force.torque.y - current_wrench.torque.y
        desired_compliance.torque.z = desired_force.torque.z - current_wrench.torque.z
        return desired_compliance


    def wrench_callback(self, msg: WrenchStamped):
        self.actual_wrench = msg.wrench

    def target_pose_callback(self, msg):
        self.target_pose = msg

    def actual_pose_callback(self, msg):
        self.actual_pose = msg

    def target_wrench_callback(self, msg):
        self.target_wrench = msg.wrench

    def get_params(self):
        self.wrench_topic = rospy.get_param("~actual_wrench_topic")
        self.target_pose_topic = rospy.get_param("~target_pose_topic")
        self.actual_pose_topic = rospy.get_param("~actual_pose_topic")
        self.target_wrench_topic = rospy.get_param("~target_wrench_topic")
        self.propotional_gain_pose_x = rospy.get_param("~propotional_gain_pose_x")
        self.propotional_gain_pose_y = rospy.get_param("~propotional_gain_pose_y")
        self.propotional_gain_pose_z = rospy.get_param("~propotional_gain_pose_z")
        self.propotional_gain_pose_roll = rospy.get_param("~propotional_gain_pose_roll")
        self.propotional_gain_pose_pitch = rospy.get_param("~propotional_gain_pose_pitch")
        self.propotional_gain_pose_yaw = rospy.get_param("~propotional_gain_pose_yaw")
        self.admittance_gain_force_x = rospy.get_param("~admittance_gain_force_x")
        self.admittance_gain_force_y = rospy.get_param("~admittance_gain_force_y")
        self.admittance_gain_force_z = rospy.get_param("~admittance_gain_force_z")
        self.admittance_gain_torque_x = rospy.get_param("~admittance_gain_torque_x")
        self.admittance_gain_torque_y = rospy.get_param("~admittance_gain_torque_y")
        self.admittance_gain_torque_z = rospy.get_param("~admittance_gain_torque_z")





if __name__ == '__main__':
    admittance_control = Admittance_control()
    admittance_control.update()