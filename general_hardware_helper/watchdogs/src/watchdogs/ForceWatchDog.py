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

        self.__limits_low=rospy.get_param("~limits_low")
        self.__limits_high=rospy.get_param("~limits_high")
       
       
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

        current=np.array([msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z,
                         msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z])

        lower=current<np.array(self.__limits_low)
        upper=current>np.array(self.__limits_high)

        if np.any(lower) or np.any(upper):
            self.display(lower,upper)
            self.bark()
            

    def bark(self):
        """Procedure to be executed when it comes to force overshoot.

        Dependend on the type of overshoot that occures, different messages for force respectivly torque are displayed.
        Independently of that all services are called.

        """        
   
       
        
        for service in self.__services:
            service.call(EmptyRequest())
    
    def display(self,lower,upper):
        rospy.logwarn("Upper: ")
        rospy.logwarn(str(upper[0])+" "+str(upper[1])+" "+str(upper[2])+" "+str(upper[3])+" "+str(upper[4])+" "+str(upper[5]))
        rospy.logwarn("Lower: ")
        rospy.logwarn(str(lower[0])+" "+str(lower[1])+" "+str(lower[2])+" "+str(lower[3])+" "+str(lower[4])+" "+str(lower[5]))