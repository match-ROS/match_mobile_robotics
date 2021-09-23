#!/usr/bin/env python3
import rospy
import socket
import time, math



class igus_TCP_driver():


    def __init__(self):

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


    def run(self):  
        message_idx = 1;
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            message_idx += 1
            MESSAGE = "CRISTART " + str(message_idx) + " ALIVEJOG " + str(self.q1) + " " + str(self.q2)
            + " " + str(self.q3) + " " + str(self.q4) + " " + str(self.q5) + " 60.0 70.0 80.0 90.0 CRIEND"

            
            self.socket.send(MESSAGE.encode())
            self.respone = self.socket.recv(BUFFER_SIZE)
            


        self.socket.close()

