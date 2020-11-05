#pragma once
#include<geometry_msgs/WrenchStamped.h>
#include<manipulate_topics/operators/wrench.hpp>
namespace msg_operators
{
    //WrenchStamped
    geometry_msgs::WrenchStamped operator+(geometry_msgs::WrenchStamped one,geometry_msgs::WrenchStamped other) 
    {
        geometry_msgs::WrenchStamped ret;
        ret.wrench=one.wrench+other.wrench;      
        return ret;
    }
    geometry_msgs::WrenchStamped operator-(geometry_msgs::WrenchStamped one,geometry_msgs::WrenchStamped other) 
    {
        geometry_msgs::WrenchStamped ret;
        ret.wrench=one.wrench-other.wrench;      
        return ret;
    }

    template <typename T>
    geometry_msgs::WrenchStamped operator/(geometry_msgs::WrenchStamped one,T other) 
    {
        geometry_msgs::WrenchStamped ret;
        ret.wrench=one.wrench/other;       
        return ret;
    }
    
} // namespace msg_operators