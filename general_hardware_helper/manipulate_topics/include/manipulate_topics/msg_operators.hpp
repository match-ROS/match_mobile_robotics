#pragma once
#include<geometry_msgs/WrenchStamped.h>
#include<sensor_msgs/JointState.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/TwistStamped.h>


namespace msg_operators{

    //Vector3
    geometry_msgs::Vector3 operator+(geometry_msgs::Vector3 one,geometry_msgs::Vector3 other)
    {
        geometry_msgs::Vector3 ret;
        ret.x=one.x+other.x;
        ret.y=one.y+other.y;
        ret.z=one.z+other.z;
    }
    template<typename T>
    geometry_msgs::Vector3 operator/(geometry_msgs::Vector3 one,T other)
    {
        geometry_msgs::Vector3 ret;
        ret.x=one.x/other;
        ret.y=one.y/other;
        ret.z=one.z/other;
    }

    //WrenchStamped
    geometry_msgs::WrenchStamped operator+(geometry_msgs::WrenchStamped one,geometry_msgs::WrenchStamped other) 
    {
        geometry_msgs::WrenchStamped ret;
        ret.wrench.force=one.wrench.force+other.wrench.force;      
        ret.wrench.torque=one.wrench.torque+other.wrench.torque;
        return ret;
    }

    template <typename T>
    geometry_msgs::WrenchStamped operator/(geometry_msgs::WrenchStamped one,T other) 
    {
        geometry_msgs::WrenchStamped ret;
        ret.wrench.force=one.wrench.force/other;       
        ret.wrench.torque=one.wrench.torque/other;
        return ret;
    }

    //Twist
    geometry_msgs::Twist operator+(geometry_msgs::Twist one,geometry_msgs::Twist other)
    {
        geometry_msgs::Twist ret;
        ret.angular=one.angular+other.angular;
        ret.linear=one.linear+other.linear;
        return ret;
    }
    template <typename T>
    geometry_msgs::Twist operator/(geometry_msgs::Twist one,T other)
    {
        geometry_msgs::Twist ret;
        ret.angular=one.angular/other;
        ret.linear=one.linear/other;
        return ret;
    }

    //TwistStamped
    geometry_msgs::TwistStamped operator+(geometry_msgs::TwistStamped one,geometry_msgs::TwistStamped other)
    {
        geometry_msgs::TwistStamped ret;
        ret.twist.angular=one.twist.angular+other.twist.angular;
        ret.twist.linear=one.twist.linear+other.twist.linear;
        return ret;
    }
    template <typename T>
    geometry_msgs::TwistStamped operator/(geometry_msgs::TwistStamped one,T other)
    {
        geometry_msgs::TwistStamped ret;
        ret.twist.angular=one.twist.angular/other;
        ret.twist.linear=one.twist.linear/other;
        return ret;
    }



    
     //JointState
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