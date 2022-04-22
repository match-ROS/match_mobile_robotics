#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions

from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header

import roslib; roslib.load_manifest('laser_assembler')
from laser_assembler.srv import *

rospy.wait_for_service("assemble_scans")

class keyence_transform():
    def __init__(self):
        rospy.init_node('keyence_transform_node')

        self.t_old = rospy.Time.now()
        self.p_old = PoseStamped() #Type: PoseStamped 
        self.p_set_old = PoseStamped() #Type: PoseStamped 
        self.vel = [0,0,0]
        self.set_vel = [0,0,0]

        rospy.Subscriber("/franka_state_controller/franka_states",FrankaState, self.pose_cb)   
        rospy.sleep(1)
        rospy.Subscriber("/profiles",PointCloud2, self.pointcloud_cb)   
        self.cloud_pub = rospy.Publisher("/cloud_out",PointCloud2, queue_size = 10)
        self.cloud3D_pub = rospy.Publisher("/cloud_3D",PointCloud, queue_size = 10)
        self.cloud_out = PointCloud2()
        self.cloud_3D = PointCloud()
        self.cloud_data = []
        self.num_of_rows = 0
        rospy.spin()

        

    def pose_cb(self,data):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        # Calculate EE velocity
        time_diff = data.header.stamp - self.t_old
        self.vel[0] = ((data.O_T_EE[12] - self.p_old.pose.position.x) /  time_diff.nsecs )
        self.vel[1] = ((data.O_T_EE[13] - self.p_old.pose.position.y) /  time_diff.nsecs )
        self.vel[2] = ((data.O_T_EE[14] - self.p_old.pose.position.z) /  time_diff.nsecs )

        self.set_vel[0] = ((data.O_T_EE_d[12] - self.p_set_old.pose.position.x) /  time_diff.nsecs )
        self.set_vel[1] = ((data.O_T_EE_d[13] - self.p_set_old.pose.position.y) /  time_diff.nsecs )
        self.set_vel[2] = ((data.O_T_EE_d[14] - self.p_set_old.pose.position.z) /  time_diff.nsecs )
        self.t_old = data.header.stamp  
        self.p_old.pose.position.x = data.O_T_EE[12]
        self.p_old.pose.position.y = data.O_T_EE[13]
        self.p_old.pose.position.z = data.O_T_EE[14]
        self.p_set_old.pose.position.x = data.O_T_EE_d[12]
        self.p_set_old.pose.position.y = data.O_T_EE_d[13]
        self.p_set_old.pose.position.z = data.O_T_EE_d[14]

        # calculate transformation from world to EE
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "sensor_optical_frame"

        t.transform.translation.x = data.O_T_EE[12]
        t.transform.translation.y = data.O_T_EE[13]
        t.transform.translation.z = data.O_T_EE[14]
        T = [[data.O_T_EE[0],data.O_T_EE[4],data.O_T_EE[8],data.O_T_EE[12]],
            [data.O_T_EE[1],data.O_T_EE[5],data.O_T_EE[9],data.O_T_EE[13]],
            [data.O_T_EE[2],data.O_T_EE[6],data.O_T_EE[10],data.O_T_EE[14]],
            [data.O_T_EE[3],data.O_T_EE[7],data.O_T_EE[11],data.O_T_EE[15]]]

        q = tf_conversions.transformations.quaternion_from_matrix(T)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.transform = t

        br.sendTransform(t)

    # def pointcloud_cb(self,cloud):

    #     #time_diff = rospy.Time.now() - self.transform.header.stamp 
    #     #y_offset = self.vel[1]*time_diff.nsecs
    
    #     #t.transform.translation.y = self.transform.transform.translation.y + y_offset / 10
    #     #print(y_offset, t.transform.translation.y,y )

    #     self.cloud_out = do_transform_cloud(cloud, self.transform)
    #     self.cloud_pub.publish(self.cloud_out)
    #     self.num_of_rows = self.num_of_rows + 1
        
            
    #     if self.num_of_rows > 500:
            
    #             #print(self.cloud_data)
    #             #print(self.cloud_out.height)

    #             #self.cloud3D_pub.publish(self.cloud_3D)
    #         try:
    #             assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
    #             resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
    #             print "Got cloud with %u points" % len(resp.cloud.points)
                
    #             self.cloud_3D = resp.cloud
    #             self.cloud_3D.header= Header()
    #             self.cloud_3D.header.stamp = rospy.Time.now()
    #             self.cloud_3D.header.frame_id = 'sensor_optical_frame'
    #             #print(self.cloud_3D.header)
    #             self.cloud3D_pub.publish(self.cloud_3D)

    #         except rospy.ServiceException, e:
    #             print "Service call failed: %s"%e
                
                





if __name__=="__main__":
    keyence_transform()