#!/usr/bin/env python
import rospy
import socket
import time, math

from geometry_msgs.msg import Twist



class igus_TCP_driver():


    def __init__(self):
        rospy.init_node("igus_TCP_driver")
        TCP_IP = '192.168.3.11'
        TCP_PORT = 3920
        BUFFER_SIZE = 1024
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((TCP_IP, TCP_PORT))
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.q4 = 0.0
        self.q5 = 0.0
        rospy.Subscriber("/igus/cmd_joint_vel", Twist, self.joint_vel_cb)
        self.run()
        
        rospy.spin()

    def run(self):  
        message_idx = 1;
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            message_idx += 1
            MESSAGE = "CRISTART " + str(message_idx) + " ALIVEJOG " + str(self.q1) + " " + str(self.q2) + " " + str(self.q3) + " " + str(self.q4) + " " + str(self.q5) + " 60.0 70.0 80.0 90.0 CRIEND"

            
            self.socket.send(MESSAGE.encode())
            self.respone = self.socket.recv(1024)
            R = self.respone.split(" ")
            if len(R)> 10:
                q1_act = R[6]
                q2_act = R[7]
                q3_act = R[8]
                q4_act = R[9]
                q5_act = R[10]
                
                print(q1_act , q2_act , q3_act, q4_act, q5_act)
            #idx = 8
            #print(R)
            # if len(R)> idx:
            #     print(R[idx])
            


        self.socket.close()



    def joint_vel_cb(self,data):
        self.q1 = data.linear.x
        self.q2 = data.linear.y
        self.q3 = data.linear.z



if __name__=="__main__":
    igus_TCP_driver()
