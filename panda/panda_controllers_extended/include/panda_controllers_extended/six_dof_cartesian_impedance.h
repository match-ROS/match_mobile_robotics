
#include <eigen3/Eigen/Dense>


#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <dynamic_reconfigure/server.h>
#include <panda_controllers_extended/StiffnessConfig.h>

#include <geometry_msgs/PoseStamped.h>


namespace panda_controllers_extended {

class SixDofCartesianImpedanceController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface>
{
    public:
      
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;   
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;

        void calculationScope(const ros::TimerEvent &);
        void complianceParamCallback(   panda_controllers_extended::StiffnessConfig& config,
                                        uint32_t level);

    private:
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_impedance_;
        hardware_interface::JointHandle joint_handle_free_;

        ros::Subscriber equi_sub_;
        ros::Timer calculation_scope_timer_;
        std::unique_ptr<dynamic_reconfigure::Server
                        <panda_controllers_extended::StiffnessConfig>>
                        dyn_compliance_server_;
            

        Eigen::Matrix<double, 6, 1> tau_d_;
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
        Eigen::Matrix<double, 6, 6> cartesian_damping_;  

        Eigen::Affine3d ee_start_;  
        Eigen::Affine3d ee_target_;
        Eigen::Affine3d last_ee_target_;

        Eigen::Matrix<double, 6, 1> computeError(Eigen::Affine3d current);
        Eigen::Matrix<double, 6, 1> saturateTorqueRate(const Eigen::Matrix<double, 6, 1>& tau_d_calculated);
        void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};
}// end panda_controllers_extended