#!/usr/bin/env python
from typing import List,Dict

import yaml
import collections

import rospy
import actionlib
import numpy as np

from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from robot_teacher.srv import SetName,DriveTo
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint


class JointStateSaver:
    def __init__(self,path):
        self.__joint_states=dict()
        self.__filename=path+".yaml"
        self.__save_named_srv=rospy.Service("~save_named",SetName,self.__addStatesCallback__)
        self.__save_srv=rospy.Service("~save",Empty,self.__save__)
        self.__joint_listener=rospy.Subscriber("joint_states",JointState,self.__jointCallback__)
        self.__joint_states_dict=dict()


    def __jointCallback__(self,msg):
        self.__joint_states=dict()
        for index,name in enumerate(msg.name):
            self.__joint_states[name]=msg.position[index]
    
    def __addStatesCallback__(self,req):
        rospy.loginfo("Adding joint state position: ")
        rospy.loginfo(self.__joint_states)
        rospy.loginfo("With name: ")
        rospy.loginfo(req.name)
        self.__joint_states_dict[req.name]=self.__joint_states     
        return  len(self.__joint_states_dict)

    def __save__(self,req):
        if self.__joint_states_dict:
            rospy.loginfo("Saving:")
            rospy.loginfo(self.__joint_states_dict)
            rospy.loginfo("to file "+self.__filename)
            with open(self.__filename, 'w') as outfile:
                yaml.safe_dump(self.__joint_states_dict,outfile,default_flow_style=False)
        return []
        
class JointStateLoader:
    def __init__(self,file_name):
        self.__filename=file_name       
        self.__reload_srv=rospy.Service("~reload",Empty,self.__reload__)
        self.__load__()
    
    def __load__(self):
        with open(self.__filename, 'r') as infile:
            self.__joint_states_dict=yaml.load(infile)
        print("Loaded yaml: ")
        print(yaml.safe_dump(self.__joint_states_dict,default_flow_style=False))
        print("As dict:")
        print(self.__joint_states_dict)

    def __reload__(self,req):
        self.__load__()
        return []
    
    def getStates(self,pose_name,joint_names):
        states=dict()
        joint_states=self.__joint_states_dict[pose_name]
        states={name : joint_states[name] for name in joint_names}
        return states
    def getNames(self):
        return self.__joint_states_dict.keys()
  


   

class JointStateCommander():
    def __init__(self,filename):
        self.__file_loader=JointStateLoader(filename)
        self.__trajectory_commander=rospy.Publisher("position_joint_controller/command",JointTrajectory,queue_size=10)
        self.__current_joint_sub=rospy.Subscriber("joint_states",JointState,self.__currentJointCallback__)
        self.__drive_to_srv=rospy.Service("~drive_to",SetName,self.__driveTo__)
        self.__arm_joint_names=["panda_joint1",
                                "panda_joint2",
                                "panda_joint3",
                                "panda_joint4",
                                "panda_joint5",
                                "panda_joint6",
                                "panda_joint7"]
        self.__joint_states=dict()
        self.__max_velocity=0.1

    def __currentJointCallback__(self,msg):
        for index,name in enumerate(msg.name):
            if name in self.__arm_joint_names:
                self.__joint_states[name]=msg.position[index] 


    
    def __calcTime__(self,target_joints,current_joints):
        target=collections.OrderedDict(sorted(target_joints.items()))
        current=collections.OrderedDict(sorted(current_joints.items()))
        if target.keys()==current.keys():
            values=np.array(target.values(),dtype=np.float)-np.array(current.values(),dtype=np.float)
            values=np.abs(values)
            return rospy.Duration(np.max(values)/self.__max_velocity)
        else:
            raise KeyError


    def __driveTo__(self,req):
        if req.name in self.__file_loader.getNames():     
            joint_states=self.__file_loader.getStates(req.name,self.__arm_joint_names)
        else:
            return 0
        

        point=JointTrajectoryPoint()
        point.positions=joint_states.values()
        point.time_from_start=self.__calcTime__(joint_states,self.__joint_states)       
       
        if point.time_from_start.to_sec() >0.05:
            joint=JointTrajectory()
            joint.joint_names=joint_states.keys()
            joint.points=[point]  
            print(joint)
            self.__trajectory_commander.publish(joint)
            return 1
        return 0      