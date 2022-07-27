#!/usr/bin/env python3

from sre_constants import SUCCESS
import rospy
from bkstools.bks_lib import bks_options
from bkstools.bks_lib.bks_base import BKSBase, eCmdCode
from std_msgs.msg import Float32
from schunk_egi.srv._SetWidth import SetWidth
from std_srvs.srv import Trigger, TriggerResponse

class egi_gripper_driver():

    def __init__(self):
        rospy.init_node('egi_gripper_driver')
        self.config()
        rospy.Service('gripper_set_width', SetWidth, self.set_witdh_handle)
        rospy.Service('gripper_close', Trigger, self.gripper_close_handle)
        rospy.Service('gripper_open', Trigger, self.gripper_open_handle)
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

    def gripper_close_handle(self,req = Trigger):
        #self.bksb.command_code = eCmdCode.MOVE_POS
        self.bksb.set_force = 50  # target force to (approx) 50N
        self.bksb.grp_dir = False # grip from outside
        #input( f"\nActual postion is {bksb.actual_pos:.1f} mm.\nInsert a workpiece to grasp from outside. (Do not use your finger!)\nPress return to perform a grip from outside movement:")
        self.bksb.command_code = eCmdCode.MOVE_FORCE    # (for historic reasons the actual grip command for simple gripping is called MOVE_FORCE...)
        
        pose_old = self.bksb.actual_pos
        i = 0
        rospy.sleep(0.1)
        while not rospy.is_shutdown():
            i += 1
            if abs(self.bksb.actual_pos - pose_old) > 0.1 and i < 30:
                rospy.sleep(0.1)
            elif abs(self.bksb.actual_pos - pose_old) < 0.1:
                break
            else:
                self.error_handle()
                return TriggerResponse(success = False, message = "Gripper did not fully close.")
            pose_old = self.bksb.actual_pos
        
        if self.bksb.actual_pos < 0.1:
            return TriggerResponse(success = False, message = "No Object detected.")
        else:
            rospy.sleep(0.1)
            self.error_handle()
            return TriggerResponse(success = True, message = "Object found at: " + str(self.bksb.actual_pos))

        
    def gripper_open_handle(self,req = Trigger):
        #self.bksb.command_code = eCmdCode.MOVE_POS
        self.bksb.set_force = 100   # target force to (approx) 50N
        self.bksb.grp_dir = True # grip from inside
        self.bksb.command_code = eCmdCode.MOVE_FORCE 
        pose_old = self.bksb.actual_pos
        i = 0
        rospy.sleep(0.1)
        while not rospy.is_shutdown():
            i += 1
            if abs(self.bksb.actual_pos - pose_old) > 0.1:
                rospy.sleep(0.1)
            elif abs(self.bksb.actual_pos - pose_old) < 0.1 and self.bksb.actual_pos > 115.0:
                break
            elif i > 30:
                self.error_handle()
                return TriggerResponse(success = False, message = "Gripper did not fully open.")

            pose_old = self.bksb.actual_pos
        return TriggerResponse(success = True, message = "Gripper opened.")

    def error_handle(self):
        rospy.logerr("Gripper did not respond. Starting error handling.")
        self.bksb.command_code = eCmdCode.CMD_ACK
        rospy.sleep(0.1)
        self.bksb.set_pos = float(self.bksb.actual_pos-0.01)
        self.bksb.set_vel = 50.0
        self.bksb.command_code = eCmdCode.MOVE_POS


    def config(self):
        self.gripper_ip = rospy.get_param('~gripper_ip', "192.168.0.1")    



if __name__=="__main__":
    egi_gripper_driver()