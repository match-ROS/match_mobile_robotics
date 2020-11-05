#pragma once
#include <geometry_msgs/TwistStamped.h>
#include <manipulate_topics/operators/twist.hpp>

namespace msg_operators{
    geometry_msgs::TwistStamped operator+(geometry_msgs::TwistStamped one,geometry_msgs::TwistStamped other)
    {
        geometry_msgs::TwistStamped ret;
        ret.twist=one.twist+other.twist;
        return ret;
    }
    geometry_msgs::TwistStamped operator-(geometry_msgs::TwistStamped one,geometry_msgs::TwistStamped other)
    {
        geometry_msgs::TwistStamped ret;
        ret.twist=one.twist-other.twist;;
        return ret;

    }
    template <typename T>
    geometry_msgs::TwistStamped operator/(geometry_msgs::TwistStamped one,T other)
    {
        geometry_msgs::TwistStamped ret;
        ret.twist=one.twist/other;
        return ret;
    }
}
