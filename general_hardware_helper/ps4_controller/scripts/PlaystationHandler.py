import rospy
from sensor_msgs.msg import Joy

class PlayStationHandler():
    X=0
    CIRC=1
    RECT=2
    TRI=3
    OPTIONS=7
    SHARE=6
    PS=8
    R1=5
    L1=4
    AX1=9
    AX2=10
    BUTTON_MAP={ X:"X",
                CIRC:"CIRC",
                RECT:"RECT",
                TRI:"TRI",
                OPTIONS:"OPTIONS",
                SHARE:"SHARE",
                PS:"PS",
                R1:"R1",
                L1:"L1",
                AX1:"AX1",
                AX2:"AX2"}

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