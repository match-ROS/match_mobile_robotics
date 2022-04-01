#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import math

class auswertung():

    def __init__(self):
        rospy.init_node('auswertung')
        self.poseArray = PoseArray() # Type: PoseArray
        self.pose = Pose()
        self.laser_projector = LaserProjection()
        self.sub = rospy.Subscriber("profiles", PointCloud2, self.subcb)
        rospy.spin()
        self.pub = rospy.Publisher('ranges', JointState, queue_size=10)


    def subcb(self,cloud):
        #rospy.loginfo("Got scan, projecting")
        cloud_points = list(pc2.read_points(cloud, skip_nans=True, field_names = ("x", "y", "z")))
        profile = []
        for i in range(1,len(cloud_points)):
            #print(i[2])
            if cloud_points[i][2] == float('inf'):
                profile.append(-999.9)
            else:
                profile.append(cloud_points[i][2])
            

        #rospy.loginfo_throttle(1,profile)
        if len(profile) > 1:
            max_value = max(profile)
            max_index = profile.index(max_value)
            rospy.loginfo_throttle(0.5,[max_value,max_index])
            #print(max_value,max_index)

        print(cloud_points)
        
        #cloud = self.laser_projector.projectLaser(scan)
        #gen = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
        #self.xyz_generator = gen
        #)

if __name__ == '__main__':
    try:
        auswertung()
    except rospy.ROSInterruptException:
        pass
