#include "ros/ros.h"

#include <XmlRpc.h>

#include <ur_client_library/comm/parser.h>
#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/comm/producer.h>
#include <ur_client_library/comm/stream.h>
#include <ur_client_library/primary/primary_parser.h>

#include <ur_calibrated_pose_pub/utils/ur_calibration_consumer.h>
#include <ur_calibrated_pose_pub/utils/dh_transformation.h>

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


	std::vector<dh_utils::DHTransformation> ideal_dh_transformations_list;
	XmlRpc::XmlRpcValue dh_param_list;
	if(dh_param_list.getType() == XmlRpc::XmlRpcValue::TypeStruct)
	{
		XmlRpc::XmlRpcValue::ValueStruct::const_iterator joint_iterator;
		for(joint_iterator = dh_param_list.begin(); joint_iterator != dh_param_list.end(); joint_iterator++)
		{
			XmlRpc::XmlRpcValue joint_info_xmlrpc = joint_iterator->second;
			if(joint_info_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeStruct)
			{
				double theta = 0.0;
				double d = 0.0;
				double a = 0.0;
				double alpha = 0.0;

				if(joint_info_xmlrpc.hasMember("theta"))
				{
					if(joint_info_xmlrpc["theta"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
					{
						theta = (double)joint_info_xmlrpc["theta"];
					}
					else
					{
						ROS_ERROR("DH parameter list - theta is not well formed");
						return -1;
					}
				}
				else
				{
					ROS_ERROR("DH parameter list - theta is not well formed");
					return -1;
				}
				
				if(joint_info_xmlrpc.hasMember("d"))
				{
					if(joint_info_xmlrpc["d"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
					{
						d = (double)joint_info_xmlrpc["d"];
					}
					else
					{
						ROS_ERROR("DH parameter list - d is not well formed");
						return -1;
					}
				}
				else
				{
					ROS_ERROR("DH parameter list - d is not well formed");
					return -1;
				}

				if(joint_info_xmlrpc.hasMember("a"))
				{
					if(joint_info_xmlrpc["a"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
					{
						a = (double)joint_info_xmlrpc["a"];
					}
					else
					{
						ROS_ERROR("DH parameter list - a is not well formed");
						return -1;
					}
				}
				else
				{
					ROS_ERROR("DH parameter list - a is not well formed");
					return -1;
				}

				if(joint_info_xmlrpc.hasMember("alpha"))
				{
					if(joint_info_xmlrpc["alpha"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
					{
						alpha = (double)joint_info_xmlrpc["alpha"];
					}
					else
					{
						ROS_ERROR("DH parameter list - alpha is not well formed");
						return -1;
					}
				}
				else
				{
					ROS_ERROR("DH parameter list - alpha is not well formed");
					return -1;
				}

				dh_utils::DHTransformation dh_transformation = dh_utils::DHTransformation(theta, d, a, alpha);
				ideal_dh_transformations_list.push_back(dh_transformation);
			}
			else
			{
				ROS_ERROR("DH parameter list is not well formed");
				return -1;
			}
		}
	}
	else
	{
		ROS_ERROR("DH parameter list is not well formed");
		return -1;
	}

	// Get the delta dh parameter directly from the UR controller
	urcl::comm::URStream<urcl::primary_interface::PrimaryPackage> stream(robot_ip, urcl::primary_interface::UR_PRIMARY_PORT);
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

	std::vector<dh_utils::DHTransformation> delta_dh_transformations_list = consumer.getDHTransformationsList();
	for(int counter = 0; counter < delta_dh_transformations_list.size(); counter++)
	{
		ideal_dh_transformations_list[counter].setDeltaTransformation(delta_dh_transformations_list[counter]);
	}



	ros::spin();
}