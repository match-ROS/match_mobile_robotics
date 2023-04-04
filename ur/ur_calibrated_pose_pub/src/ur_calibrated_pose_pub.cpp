#include <ur_calibrated_pose_pub/ur_calibrated_pose_pub.h>

namespace ur_calibrated_pose_pub
{
	URCalibratedPosePub::URCalibratedPosePub(ros::NodeHandle nh,
											 ros::NodeHandle private_nh) : nh_(nh), private_nh_(private_nh)
	{ }

	void URCalibratedPosePub::init()
	{
		this->readParams();
		this->getCalibratedDHParameter();

		// Initialze subscriber, publisher, service servers and service clients
		this->joint_state_subscriber_ = this->nh_.subscribe(this->ur_joint_state_topic_name_,
															1,
															&URCalibratedPosePub::jointStateCallback,
															this);
		this->ur_calibrated_pose_publisher_ = this->nh_.advertise<geometry_msgs::PoseStamped>("ur_calibrated_pose", 1);
	}

	void URCalibratedPosePub::execute()
	{
		ros::Rate publish_rate = ros::Rate(100.0);
		while(ros::ok())
		{
			Eigen::Matrix4d complete_transformation_matrix = Eigen::Matrix4d::Identity();
			for(dh_utils::DHTransformation &dh_transformation: this->ideal_dh_transformations_list_)
			{
				complete_transformation_matrix = complete_transformation_matrix * dh_transformation.getTransformationMatrix();
			}

			// Eigen::Vector3d euler_angle = complete_transformation_matrix.block(0,0,3,3).eulerAngles(0, 1, 2);

			geometry_msgs::PoseStamped ur_calibrated_pose_msg;
			ur_calibrated_pose_msg.header.stamp = ros::Time::now();
			ur_calibrated_pose_msg.header.frame_id = "mur620b/UR10_r/base_link";
			ur_calibrated_pose_msg.pose.position.x = complete_transformation_matrix(0, 3);
			ur_calibrated_pose_msg.pose.position.y = complete_transformation_matrix(1, 3);
			ur_calibrated_pose_msg.pose.position.z = complete_transformation_matrix(2, 3);
			
			// Eigen::Quaternion<float> q;
			// q = Eigen::AngleAxisf(euler_angle(0), Eigen::Vector3f::UnitX()) *
			// 	Eigen::AngleAxisf(euler_angle(1), Eigen::Vector3f::UnitY()) *
			// 	Eigen::AngleAxisf(euler_angle(2), Eigen::Vector3f::UnitZ());
			
			// ur_calibrated_pose_msg.pose.orientation.x = q.x();
			// ur_calibrated_pose_msg.pose.orientation.y = q.y();
			// ur_calibrated_pose_msg.pose.orientation.z = q.z();
			// ur_calibrated_pose_msg.pose.orientation.w = q.w();

			ur_calibrated_pose_msg.pose.orientation.x = 0.0;
			ur_calibrated_pose_msg.pose.orientation.y = 0.0;
			ur_calibrated_pose_msg.pose.orientation.z = 0.0;
			ur_calibrated_pose_msg.pose.orientation.w = 1.0;
						
			ur_calibrated_pose_publisher_.publish(ur_calibrated_pose_msg);


			tf::Transform transform;
			transform.setOrigin(tf::Vector3(complete_transformation_matrix(0, 3),
											complete_transformation_matrix(1, 3),
											complete_transformation_matrix(2, 3)));
			tf::Quaternion q;
			q.setRPY(0.0, 0.0, 0.0);
			transform.setRotation(q);

			this->end_effector_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "mur620b/UR10_r/base_link", "mur620b/UR10_r/calibrated_ee_pose"));


			ros::spinOnce();
			publish_rate.sleep();
		}
	}

	void URCalibratedPosePub::readParams()
	{
		// Get parameter from parameter server
		this->private_nh_.param<std::string>("robot_ip", this->robot_ip_, "");
		if(this->robot_ip_ == "")
		{
			ROS_ERROR("No robot_ip parameter found");
			return;
		}

		this->private_nh_.param<std::string>("ur_joint_state_topic_name", this->ur_joint_state_topic_name_, "");
		if(this->ur_joint_state_topic_name_ == "")
		{
			ROS_ERROR("No ur_joint_state_topic_name parameter found");
			return;
		}

		// Get DH parameters from parameter server
		XmlRpc::XmlRpcValue dh_param_list;
		this->private_nh_.getParam("dh_params", dh_param_list);
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
							theta = (((double)joint_info_xmlrpc["theta"] * M_PI) / 180.0);
						}
						else
						{
							ROS_ERROR("DH parameter list - theta is not well formed");
							return;
						}
					}
					else
					{
						ROS_ERROR("DH parameter list - theta is not well formed");
						return;
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
							return;
						}
					}
					else
					{
						ROS_ERROR("DH parameter list - d is not well formed");
						return;
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
							return;
						}
					}
					else
					{
						ROS_ERROR("DH parameter list - a is not well formed");
						return;
					}

					if(joint_info_xmlrpc.hasMember("alpha"))
					{
						if(joint_info_xmlrpc["alpha"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
						{
							alpha = (((double)joint_info_xmlrpc["alpha"] * M_PI) / 180.0);
						}
						else
						{
							ROS_ERROR_STREAM("DH parameter list - alpha is not well formed.");
							return;
						}
					}
					else
					{
						ROS_ERROR("DH parameter list - alpha is not well formed");
						return;
					}

					dh_utils::DHTransformation dh_transformation = dh_utils::DHTransformation(theta, d, a, alpha);
					this->ideal_dh_transformations_list_.push_back(dh_transformation);
				}
				else
				{
					ROS_ERROR("DH parameter list is not well formed");
					return;
				}
			}
		}
		else
		{
			ROS_ERROR("DH parameter list is not well formed");
			return;
		}
	}

	void URCalibratedPosePub::getCalibratedDHParameter()
	{
		// Get the delta dh parameter directly from the UR controller
		urcl::comm::URStream<urcl::primary_interface::PrimaryPackage> stream(this->robot_ip_, urcl::primary_interface::UR_PRIMARY_PORT);
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
			this->ideal_dh_transformations_list_[counter].setDeltaTransformation(delta_dh_transformations_list[counter]);
		}
	}

	void URCalibratedPosePub::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
	{
		for(int joint_counter = 0; joint_counter < joint_state_msg->name.size(); joint_counter++)
		{
			if(joint_state_msg->name[joint_counter] == "shoulder_pan_joint")
			{
				this->ideal_dh_transformations_list_[0].setJointState(joint_state_msg->position[joint_counter]);
			}
			else if(joint_state_msg->name[joint_counter] == "shoulder_lift_joint")
			{
				this->ideal_dh_transformations_list_[1].setJointState(joint_state_msg->position[joint_counter]);
			}
			else if(joint_state_msg->name[joint_counter] == "elbow_joint")
			{
				this->ideal_dh_transformations_list_[2].setJointState(joint_state_msg->position[joint_counter]);
			}
			else if(joint_state_msg->name[joint_counter] == "wrist_1_joint")
			{
				this->ideal_dh_transformations_list_[3].setJointState(joint_state_msg->position[joint_counter]);
			}
			else if(joint_state_msg->name[joint_counter] == "wrist_2_joint")
			{
				this->ideal_dh_transformations_list_[4].setJointState(joint_state_msg->position[joint_counter]);
			}
			else if(joint_state_msg->name[joint_counter] == "wrist_3_joint")
			{
				this->ideal_dh_transformations_list_[5].setJointState(joint_state_msg->position[joint_counter]);
			}
		}
	}
}
