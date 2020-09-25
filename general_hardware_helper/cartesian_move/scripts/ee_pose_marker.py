#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import PoseStamped


global publisher
global initial_trafo

def processFeedback(feedback):
    global publisher
    global initial_trafo   

    pose_stamped=PoseStamped()
    pose_stamped.header=feedback.header    
    pose_stamped.pose=feedback.pose

    publisher.publish(pose_stamped)
    

if __name__=="__main__":
    global publisher
    
    rospy.init_node("simple_marker")
    publisher=rospy.Publisher("pose",PoseStamped,queue_size=10)

    base_frame=rospy.get_param("~base_frame","base_link")
    ee_frame=rospy.get_param("~ee_frame","tool0")

    global initial_trafo
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    succeeded=False
    while not succeeded:
        try:
            initial_trafo = tfBuffer.lookup_transform(base_frame,ee_frame, rospy.Time())
            succeeded=True
        except :
            rospy.sleep(1)
            rospy.logerr("No such transform")


    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("pose_marker")
    
    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = base_frame
    int_marker.name = "endeffector pose"
    int_marker.description = "Simple 6-DOF Control"
    int_marker.pose.position=initial_trafo.transform.translation
    int_marker.pose.orientation=initial_trafo.transform.rotation
    int_marker.scale=0.1

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.SPHERE
    box_marker.scale.x = 0.05
    box_marker.scale.y = 0.05
    box_marker.scale.z = 0.05
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append( box_marker )

    # add the control to the interactive marker
    int_marker.controls.append( box_control )

    # create a control which will move the box
    # this control does not contain any markers,
    # which will cause RViz to insert two arrows
    x_control = InteractiveMarkerControl()
    x_control.name = "move x"
    x_control.orientation.w = 1
    x_control.orientation.x = 1
    x_control.orientation.y = 0
    x_control.orientation.z = 0
    x_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    rotx_control = InteractiveMarkerControl()
    rotx_control.name = "rotate x"
    rotx_control.orientation.w = 1
    rotx_control.orientation.x = 1
    rotx_control.orientation.y = 0
    rotx_control.orientation.z = 0
    rotx_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS


    y_control = InteractiveMarkerControl()
    y_control.orientation.w = 1
    y_control.orientation.x = 0
    y_control.orientation.y = 0
    y_control.orientation.z = 1
    y_control.name = "move y"
    y_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    roty_control = InteractiveMarkerControl()
    roty_control.name = "rotate y"
    roty_control.orientation.w = 1
    roty_control.orientation.x = 0
    roty_control.orientation.y = 0
    roty_control.orientation.z = 1
    roty_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

    z_control = InteractiveMarkerControl()
    z_control.orientation.w = 1
    z_control.orientation.x = 0
    z_control.orientation.y = 1
    z_control.orientation.z = 0
    z_control.name = "move z"
    z_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    rotz_control = InteractiveMarkerControl()
    rotz_control.name = "rotate z"
    rotz_control.orientation.w = 1
    rotz_control.orientation.x = 0
    rotz_control.orientation.y = 1
    rotz_control.orientation.z = 0
    rotz_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS


    # add the control to the interactive marker
    int_marker.controls.append(x_control)
    int_marker.controls.append(y_control)
    int_marker.controls.append(z_control)
    int_marker.controls.append(rotx_control)
    int_marker.controls.append(roty_control)
    int_marker.controls.append(rotz_control)


    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, processFeedback)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.spin()