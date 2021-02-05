#!/usr/bin/env python
import rospy
from  geometry_msgs.msg import PoseStamped ,TwistStamped
from nav_msgs.msg import Path

import csv
import numpy as np
import tf

from std_srvs.srv  import Empty



class PathCsvParser():
    def __start_srv(self,req):
        self.__started=True

    def __stop_srv(self,req):
        self.__started=False

    def __init__(self):
        file_name=rospy.get_param("~file_name")
        self.__time_incr=rospy.get_param("~time_incr")
        self.__path_pub=rospy.Publisher("trajectory",Path,queue_size=10)

        self.__start_srv=rospy.Service("start_planner",Empty,self.__start_srv)
        self.__stop_srv=rospy.Service("stop_planner",Empty,self.__stop_srv)
      
        self.__pose_pub=rospy.Publisher("pose",PoseStamped,queue_size=10)        
        self.__vel_pub=rospy.Publisher("velocity",TwistStamped,queue_size=10)

        self.__counter=0
        self.__started=False

        with open(file_name) as csvfile:
            reader = csv.reader(csvfile, delimiter=',')           
            data=list(reader)

            self.__path=Path()     
            self.__path.header.frame_id="map"
            self.__velocities=list()


            for row in data:
                pose=PoseStamped()
                pose.header.frame_id="map"
                pose.pose.position.x=float(row[0])
                pose.pose.position.y=float(row[1])
                pose.pose.position.z=0.0

                quaternion = tf.transformations.quaternion_from_euler(0, 0, float(row[2]))
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                self.__path.poses.append(pose)

                twist=TwistStamped()
                twist.twist.linear.x=float(row[3])/self.__time_incr
                twist.twist.linear.y=float(row[4])/self.__time_incr
                twist.twist.angular.z=float(row[5])/self.__time_incr
                self.__velocities.append(twist)
    
    
    def start(self):
        while not rospy.is_shutdown():
            self.publish()
            if self.__started and self.__counter+1<len(self.__velocities) and self.__counter+1<len(self.__path.poses):
                self.__counter=self.__counter+1
            rospy.sleep(self.__time_incr)

    def getPath(self):
        return self.__path
    
    def getPathList(self):
        return self.__path.poses
    
    def getVelocities(self):
        return self.__velocities

    def publish(self):
        pose_pub=self.__path.poses[self.__counter]
        pose_pub.header.stamp=rospy.Time.now()

        vel_pub=self.__velocities[self.__counter]
        vel_pub.header.stamp=rospy.Time.now()
        
        self.__path_pub.publish(self.__path)   
        self.__pose_pub.publish(pose_pub)
        self.__vel_pub.publish(vel_pub)
        

if __name__=="__main__":
    rospy.init_node("csv_path_parser")           
    parser=PathCsvParser()        
    parser.start()