import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np
from numpy import linalg as lin
from std_srvs.srv import Empty,EmptyRequest
import rosservice



""" Class that implements a watchdog for a force topic.

An instance of this class calls several services 
when it comes to force overshoot.
"""
class ForceWatchDog:
    def __init__(self): 
        """ Constuctor of the force watchdog.
        
        It searches for the following parameters:
        |Parameter              |Description                                        |
        |-----------------------|-------------------------------------------------- |
        |~lower_abs_force       |Lower thresh for the absolute value of the force   |
        |~upper_abs_force       |Upper thresh for the absolute value of the force   |
        |~lower_abs_torque      |Lower thresh for the absolute value of the torque  |
        |~upper_abs_torque      |Upper thresh for the absolute value of the torque  |
        
        """     

        self.__lower_abs_force=rospy.get_param("~lower_abs_force")
        self.__upper_abs_force=rospy.get_param("~upper_abs_force")
        self.__lower_abs_torque=rospy.get_param("~lower_abs_torque")
        self.__upper_abs_torque=rospy.get_param("~upper_abs_torque")
       
        self.__force_flag=False
        self.__torque_flag=False
        self.__time_flage=False        
        self.__watchdog=rospy.Subscriber("watch_topic",WrenchStamped,self.watch,queue_size=1)
        
        self.__services=list()
        srv_names=rospy.get_param("~services")
        for service in srv_names:
            rospy.loginfo("Watchdog will call service: "+service)
            self.__services.append(rospy.ServiceProxy(service,Empty))

    def watch(self,msg):
        """ Callback for the message the watchdog is setup to.
        
        Is called everytime a new message appeares and if limits are reached the watchdog barks.
        """

        force=np.array([msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z])
        torque=np.array([msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z])
        abs_force=lin.norm(force)
        abs_torque=lin.norm(torque)
        if not (self.__lower_abs_force < abs_force < self.__upper_abs_force):
            self.__force_flag=True
            self.bark()
            
        if not (self.__lower_abs_torque < abs_torque < self.__upper_abs_torque):
            self.__torque_flag=True
            self.bark()

    def bark(self):
        """Procedure to be executed when it comes to force overshoot.

        Dependend on the type of overshoot that occures, different messages for force respectivly torque are displayed.
        Independently of that all services are called.

        """
        
        if self.__force_flag:
           self.__force_flag=False
           rospy.logwarn("Watchdog realised Force overshooting!")
        if self.__torque_flag:
           self.__torque_flag=False
           rospy.logwarn("Watchdog realised Torque overshooting!")

        
        for service in self.__services:
            service.call(EmptyRequest())
        