#!/usr/bin/env python3
import rospy
import tf
from tf import transformations
from geometry_msgs.msg import PoseStamped, Pose
from copy import deepcopy

class ReferenceMocapToMirMap:
    def __init__(self):
        self.mir_pose_topic = rospy.get_param('~mir_pose_topic', '/mur620b/mir/robot_pose')
        self.mocap_pose_topic = rospy.get_param('~mocap_pose_topic', '/qualisys/mur620b/base_link/pose')
        rospy.Subscriber(self.mir_pose_topic, Pose, self.mir_pose_callback)
        rospy.Subscriber(self.mocap_pose_topic, PoseStamped, self.mocap_pose_callback)

        self.mir_poses = []
        self.mocap_poses = []
        
        # wait for first messages
        rospy.loginfo('Waiting for first messages...')
        rospy.wait_for_message(self.mir_pose_topic, Pose)
        rospy.loginfo('Got first message from mir_pose_topic')
        rospy.wait_for_message(self.mocap_pose_topic, PoseStamped)
        rospy.loginfo('Got first message from mocap_pose_topic')
        rospy.sleep(3.0)
        self.run()
        
    def mir_pose_callback(self, msg):
        self.mir_poses.append(deepcopy(msg))
        
    def mocap_pose_callback(self, msg):
        self.mocap_poses.append(deepcopy(msg.pose))
    
    def run(self):
        # compute average pose
        mir_pose_avg_start = self.average_pose(self.mir_poses)
        mocap_pose_avg_start = self.average_pose(self.mocap_poses)
        
        # wait for the user to move the robot
        rospy.loginfo('Please move the robot to a new position and press enter...')
        input()
        
        self.mir_poses = []
        self.mocap_poses = []
        rospy.sleep(3.0)
        
        mir_pose_avg_end = self.average_pose(self.mir_poses)
        mocap_pose_avg_end = self.average_pose(self.mocap_poses)
        
        
        print('mir_pose_avg_start', mir_pose_avg_start)
        print('mocap_pose_avg_start', mocap_pose_avg_start)
        print('mir_pose_avg_end', mir_pose_avg_end)
        print('mocap_pose_avg_end', mocap_pose_avg_end)
        
        # check if dist is the same
        mir_poses_dist = ((mir_pose_avg_end.position.x - mir_pose_avg_start.position.x)**2 + (mir_pose_avg_end.position.y - mir_pose_avg_start.position.y)**2 + (mir_pose_avg_end.position.z - mir_pose_avg_start.position.z)**2)**0.5
        mocap_poses_dist = ((mocap_pose_avg_end.position.x - mocap_pose_avg_start.position.x)**2 + (mocap_pose_avg_end.position.y - mocap_pose_avg_start.position.y)**2 + (mocap_pose_avg_end.position.z - mocap_pose_avg_start.position.z)**2)**0.5
        
        if abs(mir_poses_dist - mocap_poses_dist) > 0.05:
            rospy.logerr('Distance traveled by MIR and mocap is not the same. Please try again.')
            
        
        
        
        # # compute relative position
        # relative_position = [mir_pose_avg.position.x - mocap_pose_avg.position.x, mir_pose_avg.position.y - mocap_pose_avg.position.y, mir_pose_avg.position.z - mocap_pose_avg.position.z]

        # # compute relative orientation
        # relative_orientation = transformations.quaternion_multiply([mocap_pose_avg.orientation.x, mocap_pose_avg.orientation.y, mocap_pose_avg.orientation.z, mocap_pose_avg.orientation.w], transformations.quaternion_inverse([mir_pose_avg.orientation.x, mir_pose_avg.orientation.y, mir_pose_avg.orientation.z, mir_pose_avg.orientation.w]))
        
        # convert relative orientation to euler angles
        # relative_euler = transformations.euler_from_quaternion(relative_orientation)        
        
        # print('relative_position', relative_position)
        # print('relative_orientation', relative_orientation)

    

        
        
    def average_pose(self,poses):
        avg = Pose()
        for pose in poses:
            avg.position.x += pose.position.x * 1.0/len(poses)
            avg.position.y += pose.position.y * 1.0/len(poses)
            avg.position.z += pose.position.z * 1.0/len(poses)
            # todo: average quaternion instead of summing
            avg.orientation.x += pose.orientation.x * 1.0/len(poses)
            avg.orientation.y += pose.orientation.y * 1.0/len(poses)
            avg.orientation.z += pose.orientation.z * 1.0/len(poses)
            avg.orientation.w += pose.orientation.w * 1.0/len(poses)

        # normalize quaternion
        norm = (avg.orientation.x**2 + avg.orientation.y**2 + avg.orientation.z**2 + avg.orientation.w**2)**0.5
        avg.orientation.x /= norm
        avg.orientation.y /= norm
        avg.orientation.z /= norm
        avg.orientation.w /= norm
        
        return avg
            
if __name__ == '__main__':
    rospy.init_node('reference_mocap_to_mir_map')
    ReferenceMocapToMirMap()
