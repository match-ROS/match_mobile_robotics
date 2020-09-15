#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace ur_controllers_extended{

class OrientationCompensationController :public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
    public:
        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n) override;
        void update(const ros::Time& time, const ros::Duration& period)override;
        void starting(const ros::Time& time) override;
        void stopping(const ros::Time& time) override;
    private:
        hardware_interface::JointHandle joints_;

};

}//namespace