#! /usr/bin/env python
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench




def mean(list_of_wrench):
    x=0.0
    y=0.0
    z=0.0
    t_x=0.0
    t_y=0.0
    t_z=0.0

    for wrench in list_of_wrench:
        x=x+wrench.force.x
        y=y+wrench.force.y
        z=z+wrench.force.z
        t_x=t_x+wrench.torque.x
        t_y=t_y+wrench.torque.y
        t_z=t_z+wrench.torque.z
    
    x=x/len(list_of_wrench)
    y=y/len(list_of_wrench)
    z=z/len(list_of_wrench)
    t_x=t_x/len(list_of_wrench)
    t_y=t_y/len(list_of_wrench)
    t_z=t_z/len(list_of_wrench)

    mean_wrench=Wrench()
    mean_wrench.force.x=x
    mean_wrench.force.y=y
    mean_wrench.force.z=z

    mean_wrench.torque.x=t_x
    mean_wrench.torque.y=t_y
    mean_wrench.torque.z=t_z

    return mean_wrench


def callbackWrench(msg):
    global zero_wrench
    global pub
    global enable_calibration

    wrench=Wrench()
    wrench_filtered=WrenchStamped()
    
    wrench=msg.wrench

    if enable_calibration:
        wrench_list.append(wrench)
        if len(wrench_list)==40:      #IMPORTANT TO CHANGE LATER
            zero_wrench=mean(wrench_list)
            enable_calibration=False

    wrench_filtered.wrench.force.x=wrench.force.x-zero_wrench.force.x
    wrench_filtered.wrench.force.y=wrench.force.y-zero_wrench.force.y
    wrench_filtered.wrench.force.z=wrench.force.z-zero_wrench.force.z
    wrench_filtered.wrench.torque.x=wrench.torque.x-zero_wrench.torque.x
    wrench_filtered.wrench.torque.y=wrench.torque.y-zero_wrench.torque.y
    wrench_filtered.wrench.torque.z=wrench.torque.z-zero_wrench.torque.z
    pub.publish(wrench_filtered)

def callbackCalibrateSrv(msg):
    global enable_calibration
    enable_calibration=True
    return True


if __name__=="__main__":
   # global pub
    global zero_wrench
    zero_wrench=Wrench()
    global wrench_list
    wrench_list=list()
    global enable_calibration
    enable_calibration=False

    rospy.init_node("force_filter")
    pub=rospy.Publisher("/wrench_filtered",WrenchStamped,queue_size=10)
    sub=rospy.Subscriber("/wrench",WrenchStamped,callbackWrench)
    
    calibrate_srv=rospy.Service("force_filter/calibrate",Empty,callbackCalibrateSrv)
    rospy.spin()
