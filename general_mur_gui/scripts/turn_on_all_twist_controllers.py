#!/usr/bin/env python3

# first load all twist controllers
# call the switch controller service for all robots

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, LoadController, LoadControllerRequest

class TurnOnAllTwistControllers:

    def config(self):
        self.robot_names = rospy.get_param('~robot_names', ['mur620a','mur620b','mur620c', 'mur620d'])
        self.UR_prefixes = rospy.get_param('~UR_prefixes', ['UR10_l', 'UR10_r'])
        self.controller_name = rospy.get_param('~controller_name', 'twist_controller')
        self.old_controller_name = rospy.get_param('~old_controller_name', 'arm_controller')

    def __init__(self):
        self.config()
        self.turn_on_twist_controllers()


    def turn_on_twist_controllers(self):
        for robot_name in self.robot_names:
            for UR_prefix in self.UR_prefixes:
                load_controller_client = rospy.ServiceProxy(robot_name + "/" + UR_prefix + "/controller_manager/load_controller", LoadController)
                rospy.loginfo("Waiting for " + robot_name + "/" + UR_prefix + "/controller_manager/load_controller")
                load_controller_client.wait_for_service()
                rospy.loginfo("Loading " + self.controller_name + " for " + robot_name + "/" + UR_prefix)
                load_controller_request = LoadControllerRequest()
                load_controller_request.name = self.controller_name
                load_controller_client(load_controller_request)
                rospy.loginfo("Loaded " + self.controller_name + " for " + robot_name + "/" + UR_prefix)


                switch_controller_client = rospy.ServiceProxy(robot_name + "/" + UR_prefix + "/controller_manager/switch_controller", SwitchController)
                rospy.loginfo("Waiting for " + robot_name + "/" + UR_prefix + "/controller_manager/switch_controller")
                switch_controller_client.wait_for_service()
                rospy.loginfo("Switching to " + self.controller_name + " for " + robot_name + "/" + UR_prefix)
                switch_controller_request = SwitchControllerRequest()
                switch_controller_request.start_controllers = [self.controller_name]
                switch_controller_request.stop_controllers = [self.old_controller_name]
                switch_controller_request.strictness = 1
                switch_controller_client(switch_controller_request)
                rospy.loginfo("Switched to " + self.controller_name + " for " + robot_name + "/" + UR_prefix)



if __name__ == '__main__':
    rospy.init_node('turn_on_all_twist_controllers')
    TurnOnAllTwistControllers()
    rospy.spin()