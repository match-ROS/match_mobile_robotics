#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from math import sin, cos, pi
import numpy as np
from geometry_msgs.msg import Pose
from copy import deepcopy
import tf

class Dual_arm_collision_avoidance():

    def __init__(self):
        self.load_parameters()
        self.inital_run_l = True
        self.inital_run_r = True
        self.joint_pose_l = Pose()
        self.joint_pose_r = Pose()
        self.joints_state_index_l = [0,1,2,3,4,5]
        self.joints_state_index_r = [0,1,2,3,4,5]
        self.q_l = [0,0,0,0,0,0]
        self.q_r = [0,0,0,0,0,0]
        self.DH_params = [[0,0,0.1807,pi/2],[0,-0.6127,0,0],[0,-0.57155,0,0],[0,0,0.17415,pi/2],[0,0,0.11985,-pi/2],[0,0,0.11655,0]]
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber(self.joint_states_topic, JointState, self.joint_states_callback_l)
        rospy.Subscriber(self.joint_states_topic, JointState, self.joint_states_callback_r)
        rospy.sleep(1)
        self.run()

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            
            # get trafos from mobile base to ur base
            (trans_l,rot_l) = self.tf_listener.lookupTransform(self.tf_prefix +'/base_link', self.tf_prefix + "/" + self.ur_prefix_l + 'base_link', rospy.Time(0))
            (trans_r,rot_r) = self.tf_listener.lookupTransform(self.tf_prefix +'/base_link', self.tf_prefix + "/" + self.ur_prefix_r + 'base_link', rospy.Time(0))

            T_l = [[1,0,0,0],
             [0,1,0,0],
             [0,0,1,0],
             [0,0,0,1]]
            T_r = [[1,0,0,0],
             [0,1,0,0],
             [0,0,1,0],
             [0,0,0,1]]
            # reset pose lists every run
            self.joint_pose_list_l = []
            self.joint_pose_list_r = []
            self.joint_pose_list_l = []
            self.joint_pose_list_r = []


            # compute pose of every joint
            for i in range(0,6):
                T_l = np.dot(T_l,self.compute_DH_matrix(self.q_l[i] , self.DH_params[i][2]  
                ,self.DH_params[i][1] , self.DH_params[i][3]))
                self.joint_pose_l.position.x = -T_l[0][3] + trans_l[0]
                self.joint_pose_l.position.y = -T_l[1][3] + trans_l[1]
                self.joint_pose_l.position.z = T_l[2][3] + trans_l[2]
                self.joint_pose_list_l.append(deepcopy(self.joint_pose_l)) # do not append the reference, but a copy of the object

                T_r = np.dot(T_r,self.compute_DH_matrix(self.q_r[i] , self.DH_params[i][2] 
                ,self.DH_params[i][1] , self.DH_params[i][3]))
                self.joint_pose_r.position.x = -T_r[0][3] + trans_r[0]
                self.joint_pose_r.position.y = -T_r[1][3] + trans_r[1]
                self.joint_pose_r.position.z = T_r[2][3] + trans_r[2]
                self.joint_pose_list_r.append(deepcopy(self.joint_pose_r))
            
                # br = tf.TransformBroadcaster()
                # br.sendTransform((self.joint_pose_l.position.x, self.joint_pose_l.position.y, self.joint_pose_l.position.z),
                #         tf.transformations.quaternion_from_euler(0, 0, 0),
                #         rospy.Time.now(),
                #         "joint"+str(i)+"_l",
                #         self.tf_prefix +'/base_link')

                # br = tf.TransformBroadcaster()
                # br.sendTransform((self.joint_pose_r.position.x, self.joint_pose_r.position.y, self.joint_pose_r.position.z),
                #         tf.transformations.quaternion_from_euler(0, 0, 0),
                #         rospy.Time.now(),
                #         "joint"+str(i)+"_r",
                #         self.tf_prefix +'/base_link')

            # generate link poses
            for i in range(0,2):
                joint0_pose_l = self.joint_pose_list_l[i]
                joint0_pose_r = self.joint_pose_list_r[i]
                joint1_pose_l = self.joint_pose_list_l[i+1]
                joint1_pose_r = self.joint_pose_list_r[i+1]
                for idx in range(0,self.collision_objects_per_link):
                    link_pose_l = Pose()
                    link_pose_l.position.x = joint0_pose_l.position.x + (joint1_pose_l.position.x - joint0_pose_l.position.x) * (idx+1) / (self.collision_objects_per_link+1)
                    link_pose_l.position.y = joint0_pose_l.position.y + (joint1_pose_l.position.y - joint0_pose_l.position.y) * (idx+1) / (self.collision_objects_per_link+1)
                    link_pose_l.position.z = joint0_pose_l.position.z + (joint1_pose_l.position.z - joint0_pose_l.position.z) * (idx+1) / (self.collision_objects_per_link+1)
                    self.joint_pose_list_l.append(link_pose_l)

                    link_pose_r = Pose()
                    link_pose_r.position.x = joint0_pose_r.position.x + (joint1_pose_r.position.x - joint0_pose_r.position.x) * (idx+1) / (self.collision_objects_per_link+1)
                    link_pose_r.position.y = joint0_pose_r.position.y + (joint1_pose_r.position.y - joint0_pose_r.position.y) * (idx+1) / (self.collision_objects_per_link+1)
                    link_pose_r.position.z = joint0_pose_r.position.z + (joint1_pose_r.position.z - joint0_pose_r.position.z) * (idx+1) / (self.collision_objects_per_link+1)
                    self.joint_pose_list_r.append(link_pose_r)

                    # br = tf.TransformBroadcaster()
                    # br.sendTransform((link_pose_l.position.x, link_pose_l.position.y, link_pose_l.position.z),
                    #         tf.transformations.quaternion_from_euler(0, 0, 0),
                    #         rospy.Time.now(),
                    #         "link"+str(i)+"_l" + str(idx),
                    #         self.tf_prefix +'/base_link')

                    # br = tf.TransformBroadcaster()
                    # br.sendTransform((link_pose_r.position.x, link_pose_r.position.y, link_pose_r.position.z),
                    #         tf.transformations.quaternion_from_euler(0, 0, 0),
                    #         rospy.Time.now(),
                    #         "link"+str(i)+"_r" + str(idx),
                    #         self.tf_prefix +'/base_link')

            rate.sleep()


    def compute_DH_matrix(self,theta,d,a,alpha):
        return [[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]]


    def load_parameters(self):
        self.joint_states_topic = rospy.get_param('~joint_states_topic')
        self.joint_names = rospy.get_param('~joint_names')
        self.rate = rospy.get_param('~rate')
        self.ur_prefix_l = rospy.get_param('~ur_prefix_l')
        self.ur_prefix_r = rospy.get_param('~ur_prefix_r')
        self.tf_prefix = rospy.get_param('~tf_prefix')
        self.collision_objects_per_link = rospy.get_param('~collision_objects_per_link')

    def sort_joint_states_l(self, JointState):
        for i in range(0,6):
            for idx in range(0,len(JointState.name)):
                if JointState.name[idx] == self.ur_prefix_l + self.joint_names[i]:
                    self.joints_state_index_l[i] = idx

    def sort_joint_states_r(self, JointState):
        for i in range(0,6):
            for idx in range(0,len(JointState.name)):
                if JointState.name[idx] == self.ur_prefix_r + self.joint_names[i]:
                    self.joints_state_index_r[i] = idx
    def joint_states_callback_l(self, JointState):
        if self.inital_run_l:
            self.sort_joint_states_l(JointState)
            self.inital_run_l = False
        for i in range(0,6):
            self.q_l[i] = JointState.position[self.joints_state_index_l[i]]

    def joint_states_callback_r(self, JointState):
        if self.inital_run_r:
            self.sort_joint_states_r(JointState)
            self.inital_run_r = False
        for i in range(0,6):
            self.q_r[i] = JointState.position[self.joints_state_index_l[i]]

if __name__ == '__main__':
    rospy.init_node('dual_arm_collision_avoidance')
    dual_arm_collision_avoidance = Dual_arm_collision_avoidance()
    rospy.spin()