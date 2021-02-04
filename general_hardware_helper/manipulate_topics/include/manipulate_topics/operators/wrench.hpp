#pragma once
#include<geometry_msgs/Wrench.h>
#include<manipulate_topics/operators/vector3.hpp>
namespace msg_operators
{
    //Wrench
    geometry_msgs::Wrench operator+(geometry_msgs::Wrench one,geometry_msgs::Wrench other) 
    {
        geometry_msgs::Wrench ret;
        ret.force=one.force+other.force;      
        ret.torque=one.torque+other.torque;
        return ret;
    }
    geometry_msgs::Wrench operator-(geometry_msgs::Wrench one,geometry_msgs::Wrench other) 
    {
        geometry_msgs::Wrench ret;
        ret.force=one.force-other.force;      
        ret.torque=one.torque-other.torque;
        return ret;
    }

    template <typename T>
    geometry_msgs::Wrench operator/(geometry_msgs::Wrench one,T other) 
    {
        geometry_msgs::Wrench ret;
        ret.force=one.force/other;       
        ret.torque=one.torque/other;
        return ret;
    }
    
} // namespace msg_operators