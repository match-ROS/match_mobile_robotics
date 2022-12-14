#!/usr/bin/env python3

import rospy
from ur_msgs.srv import SetIO

from typing import Optional, List

class SchunkController():
    def __init__(self, prefix_ur="ur/", pin_magnet=0, pin_demagnet=1, pin_force_a=2, pin_force_b=3):
        """To control the Schunk gripper

        Args:
            prefix_ur (str, optional): prefrix of service. Defaults to "ur/".
            pin_magnet (int, optional): Magnetization ON. Defaults to 0.
            pin_demagnet (int, optional): Magnetization OFF. Defaults to 1.
            pin_force_a (int, optional): Force signal A. Defaults to 2.
            pin_force_b (int, optional): Force signal B. Defaults to 3.
            
        TODO Iniput:
            Message "Workpiece available"
            Message "Malfunction"
            Message "Magnetization status"
        """
        self.prefix_ur=prefix_ur
        self.pin_magnet=pin_magnet
        self.pin_demagnet=pin_demagnet
        self.pin_force_a=pin_force_a
        self.pin_force_b=pin_force_b
        
        rospy.init_node('schunk_controller_node', anonymous=True)
        self.config()
        
    def config(self):
        self.srv = rospy.ServiceProxy(self.prefix_ur+'ur_hardware_interface/set_io', SetIO)
        self.srv.wait_for_service()
        self.srv(1 ,self.pin_magnet, False) # fun, pin, state (fun=1: digital out)
        self.srv(1 ,self.pin_demagnet, False) # fun, pin, state (fun=1: digital out)
        self.srv(1 ,self.pin_force_a, False) # fun, pin, state (fun=1: digital out)
        self.srv(1 ,self.pin_force_b, False) # fun, pin, state (fun=1: digital out)
        
    def test(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            print(f"Grasping: {self.grasp()}")
            rate.sleep()
            print(f"Releasing: {self.release()}")
            rate.sleep()
        print(self.release())
        
    def grasp(self, force: int=3):
        """Grasps the object

        Args:
            force (int, optional): Holding force level: 3=15%, 2=25%, 1=35%, 0=100%. Defaults to 3.

        Raises:
            ValueError: if force is not 0, 1, 2, or 3

        Returns:
            bool: success
        """
        if not 0 <= force <= 3 or not isinstance(force, int):
            raise ValueError("force must be 0, 1, 2, or 3")
        force = bin(force)
        force_a = bool(int(force[-2]))
        force_b = bool(int(force[-1]))
        self.set_pins([self.pin_force_a, self.pin_force_b, self.pin_demagnet], [force_a, force_b, False])
        rospy.sleep(0.001) # wait for the signal to be set
        self.srv(1, self.pin_magnet, True)
        rospy.sleep(0.006) # wait for the magnet to be set
        self.set_pins() # reset all pins
        
        return True # get from io input
        
    def release(self):
        self.set_pins() # reset all pins
        self.set_pins([self.pin_demagnet], [True])
        return True # get from io input
    
    def set_pins(self, pins:Optional[List[int]] = None, states:Optional[List[bool]] = None, fun=1):
        """Set pins

        Args:
            pins (list): list of pins
            states (list): list of states
            fun (int, optional): function. Defaults to digital.
        """
        
        if pins is None:
            pins = [self.pin_magnet, self.pin_demagnet, self.pin_force_a, self.pin_force_b]
            states = [False]*len(pins)
        if states is None:
            states = [False]*len(pins)
        for pin, state in zip(pins, states):
            self.srv(fun, pin, state)
        
if __name__ == '__main__':
    gripper=SchunkController(prefix_ur="mur/ur/")
    gripper.test()
    