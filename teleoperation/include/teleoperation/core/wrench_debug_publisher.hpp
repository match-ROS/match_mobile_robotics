#pragma once

#include <ros/ros.h>

#include <geometry_msgs/WrenchStamped.h>

#include <string>

#include "teleoperation/core/types.hpp"

namespace teleoperation
{

class WrenchDebugPublisher
{
public:
  WrenchDebugPublisher() = default;

  void init(ros::NodeHandle& nh,
            ros::NodeHandle& pnh,
            const std::string& enable_param_name,
            const std::string& topic_param_name,
            const std::string& default_topic)
  {
    pnh.param<bool>(enable_param_name, enabled_, false);
    pnh.param<std::string>(topic_param_name, topic_name_, default_topic);
    if (enabled_)
    {
      pub_ = nh.advertise<geometry_msgs::WrenchStamped>(topic_name_, 1);
    }
  }

  void publish(const Wrench3& wrench, const ros::Time& stamp, const std::string& frame_id)
  {
    if (!enabled_) return;

    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.wrench.force.x = wrench.f.x();
    msg.wrench.force.y = wrench.f.y();
    msg.wrench.force.z = wrench.f.z();
    msg.wrench.torque.x = wrench.tau.x();
    msg.wrench.torque.y = wrench.tau.y();
    msg.wrench.torque.z = wrench.tau.z();
    pub_.publish(msg);
  }

private:
  bool enabled_{false};
  std::string topic_name_;
  ros::Publisher pub_;
};

}  // namespace teleoperation
