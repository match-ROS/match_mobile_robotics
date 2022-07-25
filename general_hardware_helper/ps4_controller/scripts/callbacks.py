#!/usr/bin/env python3

import rospy

class Callbacks():

    def __init__(self,ps4_driver):
        self.ps4_driver=ps4_driver

    def dummy(self):
        pass

    def lifting(self):
        rospy.INFO("Lifting!")
        if self.ps4_driver._buttons[self.ps4_driver.R1] and not self.ps4_driver._buttons[self.ps4_driver.L1]: #rising edge and not lowering
            self.ps4_driver.move_vertical(1) #lift
        else:
            self.ps4_driver.move_vertical(0) #no movement

    def lowering(self):
        rospy.INFO("Lowering!")
        if self.ps4_driver._buttons[self.ps4_driver.L1] and not self.ps4_driver._buttons[self.ps4_driver.R1]: #rising edge and not lifting
            self.ps4_driver.move_vertical(-1) #lower
        else:
            self.ps4_driver.move_vertical(0) #no movement

    def increaseRot(self):
        print("Increasing rot")
        self.ps4_driver.speed_rotation=self.ps4_driver.speed_rotation * (1+self.ps4_driver.rot_incr)
        
      
    def increaseTrans(self):
        print("Increasing trans")
        self.ps4_driver.speed_translation=self.ps4_driver.speed_translation * (1+self.ps4_driver.trans_incr)
    
    def decreaseRot(self):
        print("Decreasing rot")
        self.ps4_driver.speed_rotation=self.ps4_driver.speed_rotation* (1- self.ps4_driver.rot_incr) 
        if self.ps4_driver.speed_rotation<0.0:
            self.ps4_driver.speed_rotation=0.0
      
    def decreaseTrans(self):
        print("Decreasing trans")
        self.ps4_driver.speed_translation=self.ps4_driver.speed_translation * (1-self.ps4_driver.trans_incr)
        if  self.ps4_driver.speed_translation<0.0:
            self.ps4_driver.speed_translation=0.0
    
    def changeRobot(self):
        self.ps4_driver.active_robot = (self.ps4_driver.active_robot + 1) % len(self.ps4_driver.robotnames)