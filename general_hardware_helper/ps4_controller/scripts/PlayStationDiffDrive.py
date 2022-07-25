#!/usr/bin/env python3
from PlaystationHandler import PlayStationHandler
from geometry_msgs.msg import Twist,TwistStamped
import rospy
from callbacks import Callbacks

class PlayStationDiffDrive(PlayStationHandler):
    def __init__(self,message_type):
        PlayStationHandler.__init__(self)
        self.load_config() #load config from param server
        self.active_robot = 0
        callbacks = Callbacks(self)
        self.callbackList=[   callbacks.decreaseTrans,
                                callbacks.increaseRot,
                                callbacks.increaseTrans,
                                callbacks.decreaseRot,
                                callbacks.lowering,
                                callbacks.lifting,
                                callbacks.dummy,
                                callbacks.changeRobot
                                ]

        rospy.loginfo("TODO: Implement Service vertical_movement!")
        self.v_x = float()
        self.v_y = float()
        self.rotation = float()
        self.initialized = False
        self.lower_position_reached = False
        self.publishFunction=None   
        self.publisher_stack = []       #list of publishers    
            
        if message_type==Twist:
            print("Publishing as Twist")
            self.publishFunction=self.publishTwist
        elif  message_type==TwistStamped:
            print("Publishing as TwistStamped")
            self.publishFunction=self.publishTwistStamped
        
        
        if self.robotnames == "":
            self.publisher_stack.append(rospy.Publisher(self.cmd_vel_topic_prefix + "/cmd_vel",message_type,queue_size= 10))
        else:
            for i in self.robotnames: 
                    self.publisher_stack.append(rospy.Publisher(i+"/" + self.cmd_vel_topic_prefix + "/cmd_vel",message_type,queue_size= 10))


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
            
            if self.initialized == True:
                if self._axes[6] != 0.0 or self._axes[7] != 0.0:
                    self.v_x = self._axes[7] * self.speed_translation
                    self.v_y = self._axes[6] * self.speed_translation
                else:
                    self.v_y = self._axes[7] * self.speed_translation
                    self.v_x = self._axes[self.LS_V]*self.speed_translation
                    self.v_y = self._axes[self.LS_H]*self.speed_translation
                    self.rotation = (self._axes[self.RS_H])*self.speed_rotation
                self.publishFunction()
            else:
                rospy.loginfo_throttle(5,"Controller is not initialized. Press and release both shoulder buttons simultaneously")
                if self._axes[2] == -1.0 and self._axes[5] == -1.0:
                    self.lower_position_reached = True
                    rospy.loginfo_once("lower position reached")
                if self.lower_position_reached == True and self._axes[2] == 1.0 and self._axes[5] == 1.0:
                    self.initialized = True
                    rospy.loginfo_once("initilization complete")
            self.rate.sleep()    

    def load_config(self):
        self.speed_translation = rospy.get_param("~translation",0.1)
        self.speed_rotation =  rospy.get_param("~rotation",0.2)
        self.trans_incr=rospy.get_param("~trans_incr",0.1)
        self.rot_incr=rospy.get_param("~rot_incr",0.1)
        self.robotnames = rospy.get_param("~robot_names","")
        self.cmd_vel_topic_prefix = rospy.get_param("~cmd_vel_topic_prefix","")
        self.rate=rospy.Rate(rospy.get_param("~rate",10))

    def publishTwist(self):
        msg=Twist()
        msg.linear.x=self.v_x
        msg.linear.y = self.v_y
        msg.angular.z=self.rotation
        
        self.publisher_stack[self.active_robot].publish(msg)

    def publishTwistStamped(self):
        msg=TwistStamped()
        msg.header.stamp=rospy.Time.now()
        msg.twist.linear.x=self.v_x
        msg.twist.linear.y=self.v_y
        msg.twist.angular.z=self.rotation
        
        self.publisher_stack[self.active_robot].publish(msg)



if __name__=="__main__":
    rospy.init_node("ps4_diffdrive_controller")
    twist_stamped = rospy.get_param("~twist_stamped")
    print(twist_stamped)
    if twist_stamped == True:
        ps4=PlayStationDiffDrive(TwistStamped)
        print("stamped")
    else:
        ps4=PlayStationDiffDrive(Twist)
        print("not stamped")
    ps4.run()
    rospy.spin()