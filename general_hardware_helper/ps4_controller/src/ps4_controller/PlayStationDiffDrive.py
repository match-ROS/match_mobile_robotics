from PlaystationHandler import PlayStationHandler
from geometry_msgs.msg import Twist,TwistStamped
import rospy

class PlayStationDiffDrive(PlayStationHandler):
    def __init__(self,message_type):
        PlayStationHandler.__init__(self)
        self.__rate=rospy.Rate(rospy.get_param("~rate",10))

        self.__speed_translation = rospy.get_param("~translation",0.1)
        self.__speed_rotation =  rospy.get_param("~rotation",0.2)
        self.__trans_incr=rospy.get_param("~trans_incr",0.1)
        self.__rot_incr=rospy.get_param("~rot_incr",0.1)
        self.__callbackList=[   self.__decreaseTrans__,
                                self.__increaseRot__,
                                self.__decreaseRot__,
                                self.__increaseTrans__
                                ]
       
        self.__translation = float()
        self.__rotation = float()
        
        self.__publishFunction=None        
            
        if message_type==Twist:
            print("Publishing as Twist")
            self.__publisher=rospy.Publisher("cmd_vel",message_type,queue_size= 10)
            self.__publishFunction=self.__publishTwist__
        elif  message_type==TwistStamped:
            print("Publishing as TwistStamped")
            self.__publisher=rospy.Publisher("cmd_vel",message_type,queue_size=10)
            self.__publishFunction=self.__publishTwistStamped__


    def __increaseRot__(self):
        print("Increasing rot")
        self.__speed_rotation=self.__speed_rotation * (1+self.__rot_incr)
        
      
    def __increaseTrans__(self):
        print("Increasing trans")
        self.__speed_translation=self.__speed_translation * (1+self.__trans_incr)
    
    def __decreaseRot__(self):
        print("Decreasing rot")
        self.__speed_rotation=self.__speed_rotation* (1- self.__rot_incr) 
        if self.__speed_rotation<0.0:
            self.__speed_rotation=0.0
      
    def __decreaseTrans__(self):
        print("Decreasing trans")
        self.__speed_translation=self.__speed_translation * (1-self.__trans_incr)
        if  self.__speed_translation<0.0:
            self.__speed_translation=0.0
 


    def __publishTwist__(self):
        msg=Twist()
        msg.linear.x=self.__translation
        msg.angular.z=self.__rotation
        
        self.__publisher.publish(msg)

    def __publishTwistStamped__(self):
        msg=TwistStamped()
        msg.header.stamp=rospy.Time.now()
        msg.twist.linear.x=self.__translation
        msg.twist.angular.z=self.__rotation
        
        self.__publisher.publish(msg)

    def run(self):
        while not rospy.is_shutdown(): 
            for i,edge in enumerate(self._edges):
                if edge:
                    self._edges[i] = 0
                    try:
                        self.__callbackList[i]()
                    except Exception as ex:
                        print(ex)
                        pass

            self.__translation = (abs(self._axes[5] - 1) - abs(self._axes[2] - 1)) *self.__speed_translation #data.axes[1] + data.axes[4]
            self.__rotation = (self._axes[0] + self._axes[3])*self.__speed_rotation

            self.__publishFunction()
            self.__rate.sleep()            
