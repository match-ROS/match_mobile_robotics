#include<ros/ros.h>
#include<mir_hardware_helper/vel_integrator.hpp>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"vel_integrator_node");
    ros::NodeHandle nh;
    VelIntegrator inegrator(nh,"cmd_vel","cmd_vel_integrated");
    ros::spin();
}