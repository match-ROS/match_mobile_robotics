#!/usr/bin/env python3
import yaml
import collections
import os
import errno



import rospy
from std_srvs.srv import Empty
from robot_teacher.srv import SetName
from geometry_msgs.msg import PoseStamped


class PoseFileHandler:
    def __init__(self):
        self.__filename=rospy.get_param("~file_name",str())  
        self.__save_srv=rospy.Service("~save_pose",SetName,self.__saveCallback__)
        self.__save_srv=rospy.Service("~save_pose_unnamed",Empty,self.__saveCallbackUnnamed__)                    
                           
        self.__current_pose_sub=rospy.Subscriber("pose",PoseStamped,self.__currentPoseCallback__)
        self.__current_pose=PoseStamped()
        self.__poses=dict()
        self.__load__()
        

    def __currentPoseCallback__(self,msg):            
        self.__current_pose=msg

    def __saveCallback__(self,req):
        if req.name:
            self.__poses[req.name]=self.__current_pose
        else:
            self.__poses["pose"+str(len(self.__poses)+1)]=self.__current_pose
        self.__save__()
        return True
    
    def __saveCallbackUnnamed__(self,req):       
        self.__poses["pose"+str(len(self.__poses)+1)]=self.__current_pose
        self.__save__()

    def __load__(self):
        if os.path.isfile(self.__filename):
            with open(self.__filename, 'r') as infile:
                raw=yaml.safe_load(infile)
                if raw:
                    self.__poses=self.dictToPoses(raw)
                    
            print("Loaded pose: ")
            print(self.__poses)   
  
    def __save__(self):
        if self.__poses:
            rospy.loginfo("Saving:")
            rospy.loginfo(self.__poses)
            rospy.loginfo("to file "+self.__filename)

            if not os.path.exists(os.path.dirname(self.__filename)):
                try:
                    os.makedirs(os.path.dirname(self.__filename))
                except OSError as exc: # Guard against race condition
                    if exc.errno != errno.EEXIST:
                        raise

            with open(self.__filename, 'w') as outfile:
                yaml.safe_dump(self.posesToDict(self.__poses),outfile,default_flow_style=False) 


    def posesToDict(self,poses):
        poses_dict=dict()
        for elem in poses:
            pose_msg=poses[elem]
            pose=dict()
            pose["header"]=dict()
            pose["pose"]=dict()
            pose["pose"]["position"]=dict()
            pose["pose"]["orientation"]=dict()
            pose["header"]["frame_id"]=pose_msg.header.frame_id
            pose["pose"]["position"]["x"]=pose_msg.pose.position.x
            pose["pose"]["position"]["y"]=pose_msg.pose.position.y
            pose["pose"]["position"]["z"]=pose_msg.pose.position.z
            pose["pose"]["orientation"]["x"]=pose_msg.pose.orientation.x
            pose["pose"]["orientation"]["y"]=pose_msg.pose.orientation.y               
            pose["pose"]["orientation"]["z"]=pose_msg.pose.orientation.z
            pose["pose"]["orientation"]["w"]=pose_msg.pose.orientation.w
            poses_dict[elem]=pose
        print(poses_dict)
            
        return poses_dict

    def dictToPoses(self,poses):
        pose_list=dict()
        for elem in poses:
            pose=poses[elem]
            pose_msg=PoseStamped()
            pose_msg.header.frame_id=pose["header"]["frame_id"]
            pose_msg.pose.position.x=pose["pose"]["position"]["x"]
            pose_msg.pose.position.y=pose["pose"]["position"]["y"]
            pose_msg.pose.position.z=pose["pose"]["position"]["z"]
            pose_msg.pose.orientation.x=pose["pose"]["orientation"]["x"]
            pose_msg.pose.orientation.y=pose["pose"]["orientation"]["y"]                 
            pose_msg.pose.orientation.z=pose["pose"]["orientation"]["z"]
            pose_msg.pose.orientation.w=pose["pose"]["orientation"]["w"]   
            pose_list[elem]=pose_msg
        return pose_list