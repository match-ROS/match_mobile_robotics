#pragma once
#include<geometry_msgs/WrenchStamped.h>
#include<sensor_msgs/JointState.h>

#define  TIME_SCALING_FACTOR 10000.0


namespace msg_operators{

    geometry_msgs::WrenchStamped operator+(geometry_msgs::WrenchStamped one,geometry_msgs::WrenchStamped other) 
    {
        geometry_msgs::WrenchStamped ret;
        ret.wrench.force.x=one.wrench.force.x+other.wrench.force.x;
        ret.wrench.force.y=one.wrench.force.y+other.wrench.force.y;
        ret.wrench.force.z=one.wrench.force.z+other.wrench.force.z;
        ret.wrench.torque.x=one.wrench.torque.x+other.wrench.torque.x;
        ret.wrench.torque.y=one.wrench.torque.y+other.wrench.torque.y;
        ret.wrench.torque.z=one.wrench.torque.z+other.wrench.torque.z;
        return ret;
    }

    template <typename T>
    geometry_msgs::WrenchStamped operator/(geometry_msgs::WrenchStamped one,T other) 
    {
        geometry_msgs::WrenchStamped ret;
        ret.wrench.force.x=one.wrench.force.x/other;
        ret.wrench.force.y=one.wrench.force.y/other;
        ret.wrench.force.z=one.wrench.force.z/other;
        ret.wrench.torque.x=one.wrench.torque.x/other;
        ret.wrench.torque.y=one.wrench.torque.y/other;
        ret.wrench.torque.z=one.wrench.torque.z/other;
        return ret;
    }

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