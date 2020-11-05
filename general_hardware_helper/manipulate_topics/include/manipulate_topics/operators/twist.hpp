#pragma once
#include<manipulate_topics/operators/vector3.hpp>
#include<geometry_msgs/Twist.h>

namespace msg_operators{
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
}
