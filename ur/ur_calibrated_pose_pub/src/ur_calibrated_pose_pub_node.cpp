#include "ros/ros.h"

#include <ur_client_library/comm/parser.h>
#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/comm/producer.h>
#include <ur_client_library/comm/stream.h>
#include <ur_client_library/primary/primary_parser.h>

#include <ur_calibrated_pose_pub/utils/ur_calibration_consumer.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur_calibrated_pose_pub_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get parameter from parameter server
    std::string robot_ip;
    private_nh.param<std::string>("robot_ip", robot_ip, "");
    if(robot_ip == "")
    {
        ROS_ERROR("No robot_ip parameter found");
        return -1;
    }

    urcl::comm::URStream<urcl::primary_interface::PrimaryPackage> stream("UR10_r", urcl::primary_interface::UR_PRIMARY_PORT);
    urcl::primary_interface::PrimaryParser parser;
    urcl::comm::URProducer<urcl::primary_interface::PrimaryPackage> prod(stream, parser);
    ur_launch_hardware::URCalibrationConsumer consumer;

    urcl::comm::INotifier notifier;

    urcl::comm::Pipeline<urcl::primary_interface::PrimaryPackage> pipeline(prod, &consumer, "Pipeline", notifier);
    pipeline.run();
    while (!consumer.isCalibrated())
    {
      ros::Duration(0.1).sleep();
    }

    ros::spin();
}