#pragma once
#include<sensor_msgs/JointState.h>
#include<ros/ros.h>
namespace msg_operators{
    sensor_msgs::JointState operator+(sensor_msgs::JointState one,sensor_msgs::JointState other) 
    {
        if(one.name.size()==0)
        {
            if(other.name.size()==0)
            {
                ROS_WARN_STREAM("Returning empty filtered states!");
                return  sensor_msgs::JointState();
            }  
            else
            {
                return other;
            }          
        }
        if(other.name.size()==0)
        {
            return one;
        }
        for(int i=0;i<other.name.size();i++)
        {
            one.position[i]=one.position[i]+other.position[i];
            one.velocity[i]=one.velocity[i]+other.velocity[i];
            one.effort[i]=one.effort[i]+other.effort[i];
        }
        return one;
    }

    template <typename T>
    sensor_msgs::JointState operator/(sensor_msgs::JointState one,T other) 
    {
        if(one.name.size()==0)
        {
            ROS_WARN_STREAM("Returning empty sensor msg!");
            return one;
        }
        
        for(int i=0;i<one.name.size();i++)
        {
            one.position[i]=one.position[i]/other;
            one.velocity[i]=one.velocity[i]/other;
            one.effort[i]=one.effort[i]/other;
        }
        
        return one;
    }
}