#pragma once
#include<geometry_msgs/Vector3.h>

namespace msg_operators
{
    geometry_msgs::Vector3 operator+(geometry_msgs::Vector3 one,geometry_msgs::Vector3 other)
    {
        geometry_msgs::Vector3 ret;
        ret.x=one.x+other.x;
        ret.y=one.y+other.y;
        ret.z=one.z+other.z;
        return ret;
    }  
    geometry_msgs::Vector3 operator-(geometry_msgs::Vector3 one,geometry_msgs::Vector3 other)
    {
        geometry_msgs::Vector3 ret;
        ret.x=one.x-other.x;
        ret.y=one.y-other.y;
        ret.z=one.z-other.z;
        return ret;
    }
    template<typename T>
    geometry_msgs::Vector3 operator/(geometry_msgs::Vector3 one,T other)
    {
        geometry_msgs::Vector3 ret;
        ret.x=one.x/other;
        ret.y=one.y/other;
        ret.z=one.z/other;
        return ret;
    }

}

