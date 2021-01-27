import rospy
from sensor_msgs.msg import Joy

class PlayStationHandler():
    def __init__(self):
        self.__sub_joy = rospy.Subscriber('joy', Joy, self.__joy_callback__)
        self._buttons=[0]*11
        self._edges=[0]*11
        self._axes=[0]*8
        
    
    def __joy_callback__(self,msg):
        self._edges=[a and not b for a,b in zip(msg.buttons ,self._buttons)]        
        self._buttons=msg.buttons
        self._axes=msg.axes


    def run(self):
        pass