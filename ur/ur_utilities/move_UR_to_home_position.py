#!/usr/bin/env python3
import rospy
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest
from sensor_msgs.msg import JointState
import moveit_commander
import sys



class MoveURToHomePosition():
    
    def config(self):
        self.default_controllers = rospy.get_param('~default_controller', ['joint_state_controller', 'force_torque_sensor_controller', 'gripper_controller'])
        self.tf_prefix = rospy.get_param('~tf_prefix', 'mur620a')
        self.UR_prefix = rospy.get_param('~UR_prefix', 'UR10_l')
        self.move_group_name = rospy.get_param('~move_group_name', 'UR_arm_l')
        self.home_position = rospy.get_param('~home_position', 'Home_custom')

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('controller_manager_example')
        self.config()
        self.initial_controller = None

    def main(self):

        # Get the list of loaded controllers
        list_controllers = rospy.ServiceProxy("/" + self.tf_prefix + "/" + self.UR_prefix +  '/controller_manager/list_controllers', ListControllers)
        response = list_controllers()

        for controller in response.controller:
            #check if controller is in the default list
            if controller.name in self.default_controllers:
                #start the controller
                print("Starting controller:", controller.name)
                # (Add your code here to start the controller)

            else:
                # check if controller is running
                if controller.state == 'running':
                    # found the running controller
                    print("Found running controller:", controller.name)
                    self.initial_controller = controller.name
                    break

        # switch to the move_group arm controller and move the robot to its home position
        # create a service proxy for the controller manager switch controller service
        switch_controller = rospy.ServiceProxy("/" + self.tf_prefix + "/" + self.UR_prefix +  '/controller_manager/switch_controller', SwitchController)
        request = SwitchControllerRequest()
        request.start_controllers = ['arm_controller']
        if self.initial_controller is None:
            request.stop_controllers = []
        else:
            request.stop_controllers = [self.initial_controller]
        request.strictness = 2
        response = switch_controller(request)

        print("Switch controller response:", response)

        # Initialize moveit commander
        moveit_commander.roscpp_initialize(sys.argv)
        #robot = moveit_commander.RobotCommander()
        group = moveit_commander.MoveGroupCommander(self.move_group_name)

        # move to the first known pose
        group.set_named_target(self.home_position)
        # reduce velocity 
        group.set_max_velocity_scaling_factor(0.1)
        group.go()

        if self.initial_controller is None:
            pass
        else:
            # switch back to the initial controller
            request.start_controllers = [self.initial_controller]
            request.stop_controllers = ['arm_controller']
            response = switch_controller(request)
            print("Switch controller response:", response)
        
 






if __name__ == '__main__':
    MoveURToHomePosition().main()
