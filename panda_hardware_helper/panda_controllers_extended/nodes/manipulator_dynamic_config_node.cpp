#include<panda_controllers_extended/manipulator_dynamics/ee_dynamics_config.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"manipulator_dynamics");
    ros::NodeHandle nh;
    EEDynamicsConfig dynamics(nh);
    ros::spin();

}