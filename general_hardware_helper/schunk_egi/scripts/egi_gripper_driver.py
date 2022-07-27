#!/usr/bin/env python3

import rospy
from bkstools.bks_lib import bks_options
from bkstools.bks_lib.bks_base import BKSBase, eCmdCode
from std_msgs.msg import Float32
from schunk_egi.srv._SetWidth import SetWidth

class egi_gripper_driver():

    def __init__(self):
        rospy.init_node('egi_gripper_driver')
        self.config()
        rospy.Service('set_gripper_width', SetWidth, self.set_witdh_handle)
        self.bksb = BKSBase( self.gripper_ip )
        
        #--- Prepare gripper: Acknowledge any pending error:
        self.bksb.command_code = eCmdCode.CMD_ACK 
        rospy.sleep(0.1)

        #--- Perform a reference movement if necessary:
        if ( (self.bksb.plc_sync_input[0] & self.bksb.sw_referenced) == 0 ):
            self.bksb.command_code = eCmdCode.CMD_REFERENCE
            rospy.sleep(0.1)

            #--- Query the statusword until the referenced-bit is set:
            while ((self.bksb.plc_sync_input[0] & self.bksb.sw_referenced) == 0):
                rospy.sleep(0.1)
        else:
            rospy.loginfo( "Gripper already referenced." )

        print( f"Current gripper position is {self.bksb.actual_pos:.1f} mm." )
        rospy.spin()


    def set_witdh_handle(self,req = Float32):
        width = req.width.data
        self.bksb.set_pos = float(width)
        self.bksb.set_vel = 50.0 # target velocity limited to 50 mm/s
        self.bksb.command_code = eCmdCode.MOVE_POS
        for i in range(0,10):
            i += 1
            if abs(self.bksb.actual_pos - float(width)) > 0.1 and i < 20:
                rospy.sleep(0.1)
            elif abs(self.bksb.actual_pos - float(width)) < 0.1:
                break
            else:
                rospy.logerr("Gripper did not reach target position.")
                return False
        return True




    def config(self):
        self.gripper_ip = rospy.get_param('~gripper_ip', "192.168.0.1")    



if __name__=="__main__":
    egi_gripper_driver()