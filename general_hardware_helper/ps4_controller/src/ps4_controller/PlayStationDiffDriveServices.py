from PlaystationHandler import PlayStationHandler
from geometry_msgs.msg import Twist,TwistStamped
import rospy
from std_srvs.srv import Empty,EmptyRequest

class PlayStationDiffDriveServices(PlayStationHandler):
    def __init__(self,message_type):
        PlayStationHandler.__init__(self)
        self.__rate=rospy.Rate(10)
       
        self.__speed_translation = rospy.get_param("~translation")
        self.__speed_rotation =  rospy.get_param("~rotation") 

        self.__enable_cmd_vel= rospy.get_param("~allow_cmd_vel") 

        self.__translation=0.0
        self.__rotation=0.0

        self.__publishFunction=None        
            
        if message_type==Twist:
            print("Publishing as Twist")
            self.__publisher=rospy.Publisher("cmd_vel",message_type,queue_size= 10)
            self.__publishFunction=self.__publishTwist__
        elif  message_type==TwistStamped:
            print("Publishing as TwistStamped")
            self.__publisher=rospy.Publisher("cmd_vel",message_type,queue_size=10)
            self.__publishFunction=self.__publishTwistStamped__

        self.__serviceCallers=dict()
        for button in self.BUTTON_MAP:
            self.__serviceCallers[button]=rospy.ServiceProxy(self.BUTTON_MAP[button],Empty)

 


    def __publishTwist__(self):
        msg=Twist()
        msg.linear.x=self.__translation
        msg.angular.z=self.__rotation
        
        self.__publisher.publish(msg)

    def __publishTwistStamped__(self):
        msg=TwistStamped()
        msg.twist.linear.x=self.__translation
        msg.twist.angular.z=self.__rotation
        
        self.__publisher.publish(msg)

    def run(self):
        while not rospy.is_shutdown(): 
            for i,edge in enumerate(self._edges):
                if edge:
                    try:
                        self.__serviceCallers[i].call(EmptyRequest())
                    except Exception as ex:
                        print(ex)
                        pass

            self.__translation = (abs(self._axes[5] - 1) - abs(self._axes[2] - 1)) *self.__speed_translation #data.axes[1] + data.axes[4]
            self.__rotation = (self._axes[0] + self._axes[3])*self.__speed_rotation
            if self.__enable_cmd_vel:
                self.__publishFunction()
            self.__rate.sleep()            