#pragma once
#include<geometry_msgs/WrenchStamped.h>
namespace msg_operators
{
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
    
} // namespace msg_operators