#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import threading




class LeaderPathRecorder:
    def __init__(self):
        rospy.init_node('record_leader_path', anonymous=True)
        
        self.path_pub = rospy.Publisher('/virtual_leader/leader_path', Path, queue_size=10)
        self.pose_sub = rospy.Subscriber('/virtual_leader/leader_pose', PoseStamped, self.pose_callback)
        
        self.path = Path()
        self.path.header.frame_id = "map"
        self.recording = False
        self.lock = threading.Lock()

        rospy.loginfo("Press Enter to start/stop recording the leader's path.")
        self.input_thread = threading.Thread(target=self.handle_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def pose_callback(self, msg):
        with self.lock:
            if self.recording:
                self.path.header.stamp = rospy.Time.now()
                self.path.poses.append(msg)
                self.path_pub.publish(self.path)
                

    def handle_input(self):
        while not rospy.is_shutdown():
            input()  # Wait for Enter key press
            with self.lock:
                self.recording = not self.recording
                if self.recording:
                    rospy.loginfo("Recording started.")
                else:
                    rospy.loginfo("Recording stopped.")
                    self.path_pub.publish(self.path)
                    

if __name__ == '__main__':
    try:
        recorder = LeaderPathRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass