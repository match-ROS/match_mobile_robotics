from PlaystationHandler import PlayStationHandler
from geometry_msgs.msg import Twist,TwistStamped
import rospy

class PlayStationDiffDrive(PlayStationHandler):
    def __init__(self,message_type):
        PlayStationHandler.__init__(self)
        self.rate=rospy.Rate(rospy.get_param("~rate",10))
        self.active_robot = 0
        self.speed_translation = rospy.get_param("~translation",0.1)
        self.speed_rotation =  rospy.get_param("~rotation",0.2)
        self.trans_incr=rospy.get_param("~trans_incr",0.1)
        self.rot_incr=rospy.get_param("~rot_incr",0.1)
        self.robotnames = rospy.get_param("~robot_names",'["mir1","mir2"')
        self.cmd_vel_topic_prefix = rospy.get_param("~cmd_vel_topic_prefix","")
        self.callbackList=[   self.decreaseTrans,
                                self.increaseRot,
                                self.decreaseRot,
                                self.increaseTrans,
                                self.dummy,
                                self.dummy,
                                self.dummy,
                                self.changeRobot
                                ]

        self.translation = float()
        self.rotation = float()
        
        self.publishFunction=None        
            
        if message_type==Twist:
            print("Publishing as Twist")
            self.publishFunction=self.publishTwist
        elif  message_type==TwistStamped:
            print("Publishing as TwistStamped")
            self.publishFunction=self.publishTwistStamped
        
        self.publisher_stack = []   
        for i in self.robotnames: 
            self.publisher_stack.append(rospy.Publisher(i+"/" + self.cmd_vel_topic_prefix + "/cmd_vel",message_type,queue_size= 10))

    def dummy(self):
        pass

    def increaseRot(self):
        print("Increasing rot")
        self.speed_rotation=self.speed_rotation * (1+self.rot_incr)
        
      
    def increaseTrans(self):
        print("Increasing trans")
        self.speed_translation=self.speed_translation * (1+self.trans_incr)
    
    def decreaseRot(self):
        print("Decreasing rot")
        self.speed_rotation=self.speed_rotation* (1- self.rot_incr) 
        if self.speed_rotation<0.0:
            self.speed_rotation=0.0
      
    def decreaseTrans(self):
        print("Decreasing trans")
        self.speed_translation=self.speed_translation * (1-self.trans_incr)
        if  self.speed_translation<0.0:
            self.speed_translation=0.0
    
    def changeRobot(self):
        self.active_robot = (self.active_robot + 1) % len(self.robotnames)


    def publishTwist(self):
        msg=Twist()
        msg.linear.x=self.translation
        msg.angular.z=self.rotation
        
        self.publisher_stack[self.active_robot].publish(msg)

    def publishTwistStamped(self):
        msg=TwistStamped()
        msg.header.stamp=rospy.Time.now()
        msg.twist.linear.x=self.translation
        msg.twist.angular.z=self.rotation
        
        self.publisher_stack[self.active_robot].publish(msg)

    def run(self):
        while not rospy.is_shutdown(): 
            for i,edge in enumerate(self._edges):
                if edge:
                    self._edges[i] = 0
                    try:
                        self.callbackList[i]()
                    except Exception as ex:
                        print(ex)
                        pass
            
            self.translation = (abs(self._axes[5] - 1) - abs(self._axes[2] - 1)) *self.speed_translation #data.axes[1] + data.axes[4]
            self.rotation = (self._axes[0] + self._axes[3])*self.speed_rotation

            self.publishFunction()
            self.rate.sleep()            
