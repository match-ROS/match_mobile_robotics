#!/usr/bin/env python3

import rospy
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool, Int8

from typing import Optional, List

class SchunkController():
    def __init__(self, prefix_ur="ur/", pin_magnet=0, pin_demagnet=1, pin_picked=7, pin_force_a=2, pin_force_b=3):
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
        self.pin_picked=pin_picked
        
        rospy.init_node('schunk_controller_node', anonymous=True)
        self.config()
        
    def config(self):
        self.srv = rospy.ServiceProxy(self.prefix_ur+'ur_hardware_interface/set_io', SetIO)
        self.srv.wait_for_service()
        self.set_pins()
        
        rospy.Service(self.prefix_ur+'schunk/grasp', Trigger, self.cb_grasp_srv)
        rospy.Service(self.prefix_ur+'schunk/release', Trigger, self.cb_release_srv)
        rospy.Service(self.prefix_ur+'schunk/is_picked', Trigger, self.cb_is_picked_srv)
        
        
    def cb_grasp_srv(self, req: Trigger):
        self.grasp()
        if self.is_picked():
            return TriggerResponse(True, "Grasp successfull")
        return TriggerResponse(False, "Grasp NOT successfull")
    
    def cb_release_srv(self, req: Trigger):
        self.release()
        if not self.is_picked():
            return TriggerResponse(True, "Release successfull")
        return TriggerResponse(False, "Release NOT successfull")
    
    def cb_is_picked_srv(self, req: Trigger):
        if self.is_picked():
            return TriggerResponse(True, "Object is picked")
        return TriggerResponse(False, "Object is NOT picked")
        
    def listen_to_topics(self):
        """Listen to topics to grasp and release the object
        
        Topics:
            prefix_ur+"release" takes a bool
            prefix_ur+"grasp" takes an int from 0 to 3
        """
        rospy.Subscriber(self.prefix_ur+"release", Bool, self.cb_release)
        rospy.Subscriber(self.prefix_ur+"grasp", Int8, self.cb_grasp)
        
    def is_picked(self):
        try:
            msg=rospy.wait_for_message(self.prefix_ur+"ur_hardware_interface/io_states", IOStates, timeout=10)
            return msg.digital_in_states[self.pin_picked].state 
        except rospy.ROSException:
            rospy.logerr("No message from IOStates")
            return False
    
    def cb_release(self, msg):
        if msg.data:
            self.release()
        else:
            self.grasp()
    
    def cb_grasp(self, msg):
        self.grasp(msg.data)
        
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
            # raise ValueError("force must be 0, 1, 2, or 3")
            rospy.logerr("force must be 0, 1, 2, or 3")
            return False
        # force = bin(force)
        # force_a = bool(int(force[-2]))
        # force_b = bool(int(force[-1]))
        force_a = int(force/2)
        force_b = int(force%2)
        self.set_pins([self.pin_force_a, self.pin_force_b, self.pin_demagnet], [force_a, force_b, False])
        rospy.sleep(0.001) # wait for the signal to be set
        self.set_pins([self.pin_magnet], [True])
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
            if pin is not None and isinstance(state, bool):
                self.srv(fun, pin, state)   #fun=1: digital out)
            else:
                rospy.lodebug(f"pin {pin} or state {state} is not valid")
        
if __name__ == '__main__':
    #DI 7 ist input pick
    gripper=SchunkController(prefix_ur="mur/ur/")
    rospy.spin()
    # gripper.test()
    