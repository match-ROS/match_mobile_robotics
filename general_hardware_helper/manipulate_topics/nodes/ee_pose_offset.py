#!/usr/bin/env python  
import rospy
import math
from tf2_geometry_msgs import do_transform_pose
import tf
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs as tf2

from std_srvs.srv import Empty

def poseCallback(msg):
    global initial_trafo
    global publisher
    
    msg_trafo=tf2_ros.TransformStamped()
    msg_trafo.transform.translation.x=msg.pose.position.x
    msg_trafo.transform.translation.y=msg.pose.position.y
    msg_trafo.transform.translation.z=msg.pose.position.z
    
    msg_trafo.transform.rotation.x=msg.pose.orientation.x
    msg_trafo.transform.rotation.y=msg.pose.orientation.y
    msg_trafo.transform.rotation.z=msg.pose.orientation.z
    msg_trafo.transform.rotation.w=msg.pose.orientation.w
    
    pose=geometry_msgs.msg.PoseStamped()
    pose.pose.position.x=initial_trafo.transform.translation.x
    pose.pose.position.y=initial_trafo.transform.translation.y
    pose.pose.position.z=0.0
    pose.pose.orientation.x=0
    pose.pose.orientation.y=0
    pose.pose.orientation.z=0
    pose.pose.orientation.w=1


    pose_new=do_transform_pose(pose,msg_trafo)   
    pose_new.header=msg.header
    publisher.publish(pose_new)

    return

def resetSrvs(req):
  global tfBuffer
  global initial_trafo  

  succeeded=False   
  while not succeeded:
      try:
          initial_trafo = tfBuffer.lookup_transform(source_frame,target_frame, rospy.Time())
          succeeded=True
      except :
          rospy.logwarn_throttle(1,"Error looking up transform from "+source_frame+" to "+target_frame)
  return True




if __name__ == '__main__':
    global initial_trafo
    global publisher
    global tfBuffer
     
    
    rospy.init_node('tf_to_pose')
    

    target_frame=rospy.get_param('~target_frame')
    source_frame=rospy.get_param('~source_frame')
    rate_param=rospy.get_param('~rate')

    srv=rospy.Service("~test",Empty,resetSrvs)


    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(rate_param)
    

    succeeded=False   
    while not succeeded:
        try:
            initial_trafo = tfBuffer.lookup_transform(source_frame,target_frame, rospy.Time())
            succeeded=True
        except :
            rospy.logwarn_throttle(1,"Error looking up transform from "+source_frame+" to "+target_frame)
    rospy.logerr(initial_trafo)
    subscriber=rospy.Subscriber("pose_in",geometry_msgs.msg.PoseStamped,poseCallback)
    publisher=rospy.Publisher("pose_out",geometry_msgs.msg.PoseStamped,queue_size=1)

    rospy.spin()