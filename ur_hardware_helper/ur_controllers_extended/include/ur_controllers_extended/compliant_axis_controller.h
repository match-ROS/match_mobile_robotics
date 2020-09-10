#include <controller_interface/controller.h>
#include <position_controllers/joint_group_position_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace ur_controllers_extended{

class CompliantAxisController :public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
    public:
        bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n) override;
        void update(const ros::Time& time, const ros::Duration& period)override;
        void starting(const ros::Time& time) override;
        void stopping(const ros::Time& time) override;
    private:
        std::vector< std::string > joint_names_;
        int n_joints_;
        std::vector< hardware_interface::JointHandle> joints_;
        bool measure_;
        double stiffness_;
        double last_effort_;
        double position_equi_;
};

}//namespace