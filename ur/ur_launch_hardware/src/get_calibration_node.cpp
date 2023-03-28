#pragma once

#include "ros/ros.h"

#include <ur_client_library/comm/parser.h>
#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/comm/producer.h>
#include <ur_client_library/comm/stream.h>
#include <ur_client_library/primary/primary_parser.h>

#include <match_calibrated_ur_fk/match_ur_calibration_consumer.h>

using namespace urcl;
using namespace primary_interface;
// using namespace ur_calibration;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_ur_calibration_node");
    ros::NodeHandle nh;

    comm::URStream<PrimaryPackage> stream("UR10_r", UR_PRIMARY_PORT);
    primary_interface::PrimaryParser parser;
    comm::URProducer<PrimaryPackage> prod(stream, parser);
    ur_launch_hardware::MatchURCalibrationConsumer consumer;

    comm::INotifier notifier;

    comm::Pipeline<PrimaryPackage> pipeline(prod, &consumer, "Pipeline", notifier);
    pipeline.run();
    while (!consumer.isCalibrated())
    {
      ros::Duration(0.1).sleep();
    }
}