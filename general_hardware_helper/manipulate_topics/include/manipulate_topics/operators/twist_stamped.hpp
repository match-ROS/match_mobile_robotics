#pragma once
#include <geometry_msgs/TwistStamped.h>
#include <manipulate_topics/operators/twist.hpp>

namespace msg_operators{
    geometry_msgs::TwistStamped operator+(geometry_msgs::TwistStamped one,geometry_msgs::TwistStamped other)
    {
        geometry_msgs::TwistStamped ret;
        ret.twist.angular=one.twist.angular+other.twist.angular;
        ret.twist.linear=one.twist.linear+other.twist.linear;
        return ret;
    }
    geometry_msgs::TwistStamped operator-(geometry_msgs::TwistStamped one,geometry_msgs::TwistStamped other)
    {
        geometry_msgs::TwistStamped ret;
        ret.twist.angular=one.twist.angular-other.twist.angular;
        ret.twist.linear=one.twist.linear-other.twist.linear;
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
}
