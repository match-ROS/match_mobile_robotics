#!/usr/bin/env python3
import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

print('Switching URs to twist control')
# List of robot names
robot_names = rospy.get_param('~robot_names', ['mur620a', 'mur620b', 'mur620c', 'mur620d','VRM'])
UR_prefix = rospy.get_param('~UR_prefix', ['UR10_l', 'UR10_r'])
rospy.init_node('switch_URs_to_twist_control')

# Iterate over each robot
for robot_name in robot_names:
    # Create a ROS service proxy to switch controllers
    switch_controller_proxy = rospy.ServiceProxy('/' + robot_name + '/controller_manager/switch_controller', SwitchController)

    # Create a request to switch controllers
    switch_controller_request = SwitchControllerRequest()
    switch_controller_request.start_controllers = ['joint_group_vel_controller_l/unsafe', 'joint_group_vel_controller_r/unsafe']
    switch_controller_request.stop_controllers = [UR_prefix[0]+'/arm_controller', UR_prefix[1]+'/arm_controller']
    switch_controller_request.strictness = SwitchControllerRequest.BEST_EFFORT

    # Call the service to switch controllers
    switch_controller_response = switch_controller_proxy(switch_controller_request)

    # Check if the service call was successful
    if switch_controller_response.ok:
        rospy.loginfo('Switched controllers for {} successfully'.format(robot_name))
    else:
        rospy.logwarn('Failed to switch controllers for {}'.format(robot_name))

    