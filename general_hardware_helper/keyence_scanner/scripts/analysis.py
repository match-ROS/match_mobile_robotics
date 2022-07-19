#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

class auswertung():

    def __init__(self):
        rospy.init_node('auswertung')
        self.poseArray = PoseArray() # Type: PoseArray
        self.pose = Pose()
        self.laser_projector = LaserProjection()
        self.sub = rospy.Subscriber("cloud_out", PointCloud2, self.subcb)
        rospy.spin()
        self.pub = rospy.Publisher('ranges', JointState, queue_size=10)


    def subcb(self,cloud):
        #rospy.loginfo("Got scan, projecting")
        cloud_points = list(pc2.read_points(cloud, skip_nans=True, field_names = ("x", "y", "z")))
        print("running")
        for i in range(1,len(cloud_points)):
            #print(i[2])
            self.pose.position.x = cloud_points[i][2]
            if i == 400:
                rospy.loginfo(self.pose.position.x)
            
            #self.poseArray.append(self.pose)

        
        #cloud = self.laser_projector.projectLaser(scan)
        #gen = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
        #self.xyz_generator = gen
        #)

if __name__ == '__main__':
    try:
        auswertung()
    except rospy.ROSInterruptException:
        pass
