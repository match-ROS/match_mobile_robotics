#!/usr/bin/env python3
import rospy
import tf
from tf import transformations
from geometry_msgs.msg import PoseStamped, Pose
from copy import deepcopy
import math

class ReferenceMocapToMirMap:
    def __init__(self):
        self.mir_pose_topic = rospy.get_param('~mir_pose_topic', '/mur620c/mir/robot_pose')
        self.mocap_pose_topic = rospy.get_param('~mocap_pose_topic', '/qualisys/mur620c/pose')
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
        
        theta_mir = transformations.euler_from_quaternion([mir_pose_avg_start.orientation.x, mir_pose_avg_start.orientation.y, mir_pose_avg_start.orientation.z, mir_pose_avg_start.orientation.w])[2]
        theta_mocap = transformations.euler_from_quaternion([mocap_pose_avg_start.orientation.x, mocap_pose_avg_start.orientation.y, mocap_pose_avg_start.orientation.z, mocap_pose_avg_start.orientation.w])[2]
        # print('mir_pose_avg_start: ', mir_pose_avg_start)
        # print('theta_mir: ', theta_mir)
        # print('mocap_pose_avg_start: ', mocap_pose_avg_start)
        # print('theta_mocap: ', theta_mocap)
        
        
        # turn both positions to the same orientation
        R = transformations.rotation_matrix(theta_mir, (0, 0, 1))
        
        mir_pose_transformed = Pose()
        mir_pose_transformed.position.x = R[0, 0] * mir_pose_avg_start.position.x + R[0, 1] * mir_pose_avg_start.position.y
        mir_pose_transformed.position.y = R[1, 0] * mir_pose_avg_start.position.x + R[1, 1] * mir_pose_avg_start.position.y
        
        R = transformations.rotation_matrix(theta_mocap, (0, 0, 1))
        mocap_pose_transformed = Pose()
        mocap_pose_transformed.position.x = R[0, 0] * mocap_pose_avg_start.position.x + R[0, 1] * mocap_pose_avg_start.position.y
        mocap_pose_transformed.position.y = R[1, 0] * mocap_pose_avg_start.position.x + R[1, 1] * mocap_pose_avg_start.position.y
        
        # compute position difference
        diff_x = mir_pose_transformed.position.x - mocap_pose_transformed.position.x
        diff_y = mir_pose_transformed.position.y - mocap_pose_transformed.position.y
        
        # transform position differnce to the map frame
        R = transformations.rotation_matrix(-theta_mir, (0, 0, 1))
        diff_x_map = R[0, 0] * diff_x + R[0, 1] * diff_y
        diff_y_map = R[1, 0] * diff_x + R[1, 1] * diff_y
        
        print('diff_x_map: ', diff_x_map)
        print('diff_y_map: ', diff_y_map)
        
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
            
            
                
        
        
        
        rospy.sleep(3.0)
        
        # wait for the user to move the robot
        rospy.loginfo('Please move the robot to a new position and press enter...')
        input()
        
        self.mir_poses = []
        self.mocap_poses = []
        rospy.sleep(3.0)
        
        mir_pose_avg_end = self.average_pose(self.mir_poses)
        mocap_pose_avg_end = self.average_pose(self.mocap_poses)
        
        # check if dist is the same
        mir_poses_dist = ((mir_pose_avg_end.position.x - mir_pose_avg_start.position.x)**2 + (mir_pose_avg_end.position.y - mir_pose_avg_start.position.y)**2 + (mir_pose_avg_end.position.z - mir_pose_avg_start.position.z)**2)**0.5
        mocap_poses_dist = ((mocap_pose_avg_end.position.x - mocap_pose_avg_start.position.x)**2 + (mocap_pose_avg_end.position.y - mocap_pose_avg_start.position.y)**2 + (mocap_pose_avg_end.position.z - mocap_pose_avg_start.position.z)**2)**0.5
        
        if abs(mir_poses_dist - mocap_poses_dist) > 0.05:
            rospy.logerr('Distance traveled by MIR and mocap is not the same. Please try again.')
            
        # compute position difference for start and end
        position_diff_start = [mir_pose_avg_start.position.x - mocap_pose_avg_start.position.x, mir_pose_avg_start.position.y - mocap_pose_avg_start.position.y]
        position_diff_end = [mir_pose_avg_end.position.x - mocap_pose_avg_end.position.x, mir_pose_avg_end.position.y - mocap_pose_avg_end.position.y]
        
        print('position_diff_start: {}'.format(position_diff_start))
        print('position_diff_end: {}'.format(position_diff_end))
        
        # compute angle between the two poses
        mir_angle = math.atan2(mir_pose_avg_end.position.y - mir_pose_avg_start.position.y, mir_pose_avg_end.position.x - mir_pose_avg_start.position.x)
        mocap_angle = math.atan2(mocap_pose_avg_end.position.y - mocap_pose_avg_start.position.y, mocap_pose_avg_end.position.x - mocap_pose_avg_start.position.x)
        

        
        # compute relative angle
        relative_angle = mocap_angle - mir_angle
        
        relative_position_start = [mir_pose_avg_start.position.x - mocap_pose_avg_start.position.x, mir_pose_avg_start.position.y - mocap_pose_avg_start.position.y]
        relative_position_end = [mir_pose_avg_end.position.x - mocap_pose_avg_end.position.x, mir_pose_avg_end.position.y - mocap_pose_avg_end.position.y]
        
        rel_transform = [(relative_position_end[0] + relative_position_start[0])/2, (relative_position_end[1] + relative_position_start[1])/2, relative_angle]
        # output transform
        rospy.loginfo('map transform: {}'.format(rel_transform)) 
        


    

        
        
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
        
        if norm == 0:
            rospy.logerr('quaternion not valid')
            return None
                    
        avg.orientation.x /= norm
        avg.orientation.y /= norm
        avg.orientation.z /= norm
        avg.orientation.w /= norm
        
        return avg
            
if __name__ == '__main__':
    rospy.init_node('reference_mocap_to_mir_map')
    ReferenceMocapToMirMap()
