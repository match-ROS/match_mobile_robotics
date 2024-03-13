#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from copy import deepcopy


class UR_twist_limiter():
    
    def config(self):
        self.input_command_topic = rospy.get_param("~input_command_topic", "mur620c/UR10_r/twist_controller/command_safe")
        self.output_command_topic = rospy.get_param("~output_command_topic", "mur620c/UR10_r/twist_controller/command")
        
        # self.lin_vel_limit = rospy.get_param("~max_linear_speed", 0.1)
        # self.angular_vel_limit = rospy.get_param("~angular_vel_limit", 0.1)
        # self.lin_acc_limit = rospy.get_param("~max_linear_acceleration", 0.4)
        # self.angular_acc_limit = rospy.get_param("~angular_acc_limit", 0.005)
        # self.lin_jerk_limit = rospy.get_param("~lin_jerk_limit", 0.04)
        # self.angular_jerk_limit = rospy.get_param("~angular_jerk_limit", 0.1)
        # self.command_timeout = rospy.get_param("~command_timeout", 0.5)
    
    def __init__(self):
        rospy.init_node("ur_twist_limiter")
        self.config()
        self.setup_ddynamic_reconfigure()
                
        rospy.Subscriber(self.input_command_topic, Twist, self.input_callback, True)
        self.output_pub = rospy.Publisher(self.output_command_topic, Twist, queue_size=1)
        
        self.initial_run = True
        self.last_command = Twist()
        self.last_acc = Twist()
        self.time_old = rospy.Time.now()
        
    def monitor(self):
        self.last_command_timestamp = rospy.Time.now()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.last_command_timestamp + rospy.Duration(self.command_timeout) < rospy.Time.now():
                self.input_callback(Twist(), False)
            rate.sleep()
        

    def input_callback(self,msg = Twist(), callback = True):

        # on intial run set time old
        if self.initial_run:
            self.last_command = Twist()
            self.time_old = rospy.Time.now()
            self.last_acc = Twist()
            self.initial_run = False
            return

        now = rospy.Time.now()
        if callback:
            self.last_command_timestamp = now
        
        vel_scale_factor = 1.0
        output = Twist()

        # limit velocities
        if abs(msg.linear.x) > self.lin_vel_limit:
            vel_scale_factor = self.lin_vel_limit / abs(msg.linear.x)
        if abs(msg.linear.y) > self.lin_vel_limit and abs(msg.linear.y) * vel_scale_factor > self.lin_vel_limit:
            vel_scale_factor = self.lin_vel_limit / abs(msg.linear.y)
        if abs(msg.linear.z) > self.lin_vel_limit and abs(msg.linear.z) * vel_scale_factor > self.lin_vel_limit:
            vel_scale_factor = self.lin_vel_limit / abs(msg.linear.z)
        if abs(msg.angular.x) > self.angular_vel_limit and abs(msg.angular.x) * vel_scale_factor > self.angular_vel_limit:
            vel_scale_factor = self.angular_vel_limit / abs(msg.angular.x)
        if abs(msg.angular.y) > self.angular_vel_limit and abs(msg.angular.y) * vel_scale_factor > self.angular_vel_limit:
            vel_scale_factor = self.angular_vel_limit / abs(msg.angular.y)
        if abs(msg.angular.z) > self.angular_vel_limit and abs(msg.angular.z) * vel_scale_factor > self.angular_vel_limit:
            vel_scale_factor = self.angular_vel_limit / abs(msg.angular.z)

        output = self.apply_vel_scale_factor(msg, vel_scale_factor)
        acc_scale_factor = 1.0 # reset scale factor

        # compute accelerations
        acc_lin_x = (output.linear.x - self.last_command.linear.x)
        acc_lin_y = (output.linear.y - self.last_command.linear.y) 
        acc_lin_z = (output.linear.z - self.last_command.linear.z) 
        acc_ang_x = (output.angular.x - self.last_command.angular.x) 
        acc_ang_y = (output.angular.y - self.last_command.angular.y)
        acc_ang_z = (output.angular.z - self.last_command.angular.z) 

        # compute the acc scale factor based on how much the acceleration is over the limit
        if abs(acc_lin_x) > self.lin_acc_limit:
            acc_scale_factor = self.lin_acc_limit / abs(acc_lin_x)
        if abs(acc_lin_y) > self.lin_acc_limit and abs(acc_lin_y) * acc_scale_factor > self.lin_acc_limit:
            acc_scale_factor = self.lin_acc_limit / abs(acc_lin_y)
        if abs(acc_lin_z) > self.lin_acc_limit and abs(acc_lin_z) * acc_scale_factor > self.lin_acc_limit:
            acc_scale_factor = self.lin_acc_limit / abs(acc_lin_z)
        if abs(acc_ang_x) > self.angular_acc_limit and abs(acc_ang_x) * acc_scale_factor > self.angular_acc_limit:
            acc_scale_factor = self.angular_acc_limit / abs(acc_ang_x)
        if abs(acc_ang_y) > self.angular_acc_limit and abs(acc_ang_y) * acc_scale_factor > self.angular_acc_limit:
            acc_scale_factor = self.angular_acc_limit / abs(acc_ang_y)
        if abs(acc_ang_z) > self.angular_acc_limit and abs(acc_ang_z) * acc_scale_factor > self.angular_acc_limit:
            acc_scale_factor = self.angular_acc_limit / abs(acc_ang_z)

        # apply acceleration limit
        acc_lin_x = acc_lin_x * acc_scale_factor
        acc_lin_y = acc_lin_y * acc_scale_factor
        acc_lin_z = acc_lin_z * acc_scale_factor
        acc_ang_x = acc_ang_x * acc_scale_factor
        acc_ang_y = acc_ang_y * acc_scale_factor
        acc_ang_z = acc_ang_z * acc_scale_factor

        # compute jerk based on the limited acceleration
        jerk_lin_x = acc_lin_x - self.last_acc.linear.x
        jerk_lin_y = acc_lin_y - self.last_acc.linear.y
        jerk_lin_z = acc_lin_z - self.last_acc.linear.z
        jerk_ang_x = acc_ang_x - self.last_acc.angular.x
        jerk_ang_y = acc_ang_y - self.last_acc.angular.y
        jerk_ang_z = acc_ang_z - self.last_acc.angular.z

        # compute the jerk scale factor based on how much the jerk is over the limit
        jerk_scale_factor = 1.0
        if abs(jerk_lin_x) > self.lin_jerk_limit:
            jerk_scale_factor = self.lin_jerk_limit / abs(jerk_lin_x)
        if abs(jerk_lin_y) > self.lin_jerk_limit and abs(jerk_lin_y) * jerk_scale_factor > self.lin_jerk_limit:
            jerk_scale_factor = self.lin_jerk_limit / abs(jerk_lin_y)
        if abs(jerk_lin_z) > self.lin_jerk_limit and abs(jerk_lin_z) * jerk_scale_factor > self.lin_jerk_limit:
            jerk_scale_factor = self.lin_jerk_limit / abs(jerk_lin_z)
        if abs(jerk_ang_x) > self.angular_jerk_limit and abs(jerk_ang_x) * jerk_scale_factor > self.angular_jerk_limit:
            jerk_scale_factor = self.angular_jerk_limit / abs(jerk_ang_x)
        if abs(jerk_ang_y) > self.angular_jerk_limit and abs(jerk_ang_y) * jerk_scale_factor > self.angular_jerk_limit:
            jerk_scale_factor = self.angular_jerk_limit / abs(jerk_ang_y)
        if abs(jerk_ang_z) > self.angular_jerk_limit and abs(jerk_ang_z) * jerk_scale_factor > self.angular_jerk_limit:
            jerk_scale_factor = self.angular_jerk_limit / abs(jerk_ang_z)

        # apply jerk limit
        acc_lin_x = self.last_acc.linear.x + jerk_scale_factor * jerk_lin_x
        acc_lin_y = self.last_acc.linear.y + jerk_scale_factor * jerk_lin_y
        acc_lin_z = self.last_acc.linear.z + jerk_scale_factor * jerk_lin_z
        acc_ang_x = self.last_acc.angular.x + jerk_scale_factor * jerk_ang_x
        acc_ang_y = self.last_acc.angular.y + jerk_scale_factor * jerk_ang_y
        acc_ang_z = self.last_acc.angular.z + jerk_scale_factor * jerk_ang_z

        # compute the new velocities
        output.linear.x = self.last_command.linear.x + acc_lin_x
        output.linear.y = self.last_command.linear.y + acc_lin_y
        output.linear.z = self.last_command.linear.z + acc_lin_z
        output.angular.x = self.last_command.angular.x + acc_ang_x
        output.angular.y = self.last_command.angular.y + acc_ang_y
        output.angular.z = self.last_command.angular.z + acc_ang_z

        # update old valueslast_command
        self.last_command = deepcopy(output)
        self.time_old = now
        self.last_acc.linear.x = acc_lin_x
        self.last_acc.linear.y = acc_lin_y
        self.last_acc.linear.z = acc_lin_z
        self.last_acc.angular.x = acc_ang_x
        self.last_acc.angular.y = acc_ang_y
        self.last_acc.angular.z = acc_ang_z

        # republish message
        self.output_pub.publish(output)

    
    
    
    def apply_vel_scale_factor(self, msg = Twist(), scale_factor = 1.0):
        msg.linear.x *= scale_factor
        msg.linear.y *= scale_factor
        msg.linear.z *= scale_factor
        msg.angular.x *= scale_factor
        msg.angular.y *= scale_factor
        msg.angular.z *= scale_factor
        return msg
        
    def dyn_rec_callback(self,config, level):
        self.lin_vel_limit = config["lin_vel_limit"]
        self.angular_vel_limit = config["angular_vel_limit"] 
        self.lin_acc_limit = config["lin_acc_limit"] / 1000.0
        self.angular_acc_limit = config["angular_acc_limit"] / 1000.0
        self.angular_jerk_limit = config["angular_jerk_limit"] / 1000.0
        self.lin_jerk_limit = config["lin_jerk_limit"] / 1000.0
        self.command_timeout = config["command_timeout"]
        
        return config
    
    def setup_ddynamic_reconfigure(self):
        # Create a D(ynamic)DynamicReconfigure
        ddynrec = DDynamicReconfigure("example_dyn_rec")

        # Add variables (name, description, default value, min, max, edit_method)
        ddynrec.add_variable("lin_vel_limit", "float/double variable", 0.15, 0, 0.3)
        ddynrec.add_variable("angular_vel_limit", "float/double variable", 0.2, 0, 0.6)
        ddynrec.add_variable("lin_acc_limit", "float/double variable", 4.0, 0, 10.0)
        ddynrec.add_variable("angular_acc_limit", "float/double variable", 4.0, 0, 10.0)
        ddynrec.add_variable("lin_jerk_limit", "float/double variable", 0.8, 0, 2.0)
        ddynrec.add_variable("angular_jerk_limit", "float/double variable", 0.8, 0, 2.0)
        ddynrec.add_variable("command_timeout", "float/double variable", 0.05, 0, 0.5)

        # Start the server
        ddynrec.start(self.dyn_rec_callback)
        
        
        
if __name__ == "__main__":
    UR_twist_limiter().monitor()
        
        