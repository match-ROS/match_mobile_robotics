#!/usr/bin/env python3

# uses the roslaunch API to enable all UR robots

import rospy
import roslaunch
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, UnloadController, UnloadControllerRequest



class EnableAllURs:
    
    def config(self):
        self.robot_names = rospy.get_param('~robot_names', ['mur620a', 'mur620b','mur620c','mur620d'])
        self.UR_prefixes = rospy.get_param('~UR_prefixes', ['UR10_l', 'UR10_r'])
        self.node_name = rospy.get_param('~node_name', 'UR_enable')
        self.launch_pkg = rospy.get_param('~launch_pkg', 'ur_utilities')
        self.target_position_name = rospy.get_param('~target_position_name', 'handling_position')
        self.move_group_names = rospy.get_param('~move_group_names', ['UR_arm_l', 'UR_arm_r'])
        self.twist_controller_name = rospy.get_param('~twist_controller_name', 'twist_controller')

    def __init__(self):
        self.config()
        self.unload_twist_controllers()
        self.start_move_UR_to_home_pose()

    def unload_twist_controllers(self):
        for robot_name in self.robot_names:
            for UR_prefix in self.UR_prefixes:
                manager_ns = f"/{robot_name}/{UR_prefix}/controller_manager"
                switch_service = manager_ns + "/switch_controller"
                unload_service = manager_ns + "/unload_controller"

                switch_client = rospy.ServiceProxy(switch_service, SwitchController)
                unload_client = rospy.ServiceProxy(unload_service, UnloadController)

                try:
                    rospy.loginfo(f"Waiting for {switch_service}")
                    switch_client.wait_for_service()
                    request = SwitchControllerRequest()
                    request.start_controllers = []
                    request.stop_controllers = [self.twist_controller_name]
                    request.strictness = 1
                    rospy.loginfo(f"Stopping {self.twist_controller_name} on {robot_name}/{UR_prefix}")
                    switch_client(request)
                except (rospy.ServiceException, rospy.ROSException) as exc:
                    rospy.logwarn(f"Failed to stop {self.twist_controller_name} on {robot_name}/{UR_prefix}: {exc}")

                try:
                    rospy.loginfo(f"Waiting for {unload_service}")
                    unload_client.wait_for_service()
                    unload_request = UnloadControllerRequest()
                    unload_request.name = self.twist_controller_name
                    rospy.loginfo(f"Unloading {self.twist_controller_name} on {robot_name}/{UR_prefix}")
                    unload_client(unload_request)
                except (rospy.ServiceException, rospy.ROSException) as exc:
                    rospy.logwarn(f"Failed to unload {self.twist_controller_name} on {robot_name}/{UR_prefix}: {exc}")


    def start_move_UR_to_home_pose(self):
        for robot_name in self.robot_names:
            for UR_prefix in self.UR_prefixes:
                topic = "/" + robot_name + "/" + UR_prefix + "/ur_hardware_interface"
                rospy.loginfo("Enabling " + topic)
                namespace = "/" + robot_name + "/" + UR_prefix + "/"
                process = self.launch_ros_node(self.node_name, self.launch_pkg, self.node_name + '.py', namespace, '' , ur_hardware_interface_topic=topic)
                # check if the node is still running
                while process.is_alive() and not rospy.is_shutdown():
                    rospy.sleep(1)

        # shutdown node
        rospy.signal_shutdown('All UR robots enabled.')


    def launch_ros_node(self,node_name, package_name, node_executable, namespace="/", node_args="", **params):
        # get param names from kwargs
        param_names = params.keys()
        # set params on param server
        rospy.loginfo("Setting params for node: " + namespace + node_name)
        for param_name in param_names:
            rospy.loginfo("Setting param: " + namespace + node_name + "/" + param_name + " to " + str(params[param_name]))
            rospy.set_param(namespace + node_name + "/" + param_name, params[param_name])

        package = package_name
        executable = node_executable
        name = node_name
        node = roslaunch.core.Node(package=package, node_type=executable, name=name, namespace=namespace,
                                        machine_name=None, args=node_args, output="screen")
        
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)
        return process




if __name__ == '__main__':
    rospy.init_node('enable_all_URs')
    EnableAllURs()
    rospy.spin()





