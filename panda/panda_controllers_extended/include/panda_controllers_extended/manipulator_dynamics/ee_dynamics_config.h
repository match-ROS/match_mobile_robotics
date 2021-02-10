#include <panda_controllers_extended/manipulator_dynamics/ee_dynamics.h> 
#include <dynamic_reconfigure/server.h>
#include <panda_controllers_extended/InertiaConfig.h>

class EEDynamicsConfig:public EEDynamics{
    public:
        EEDynamicsConfig(ros::NodeHandle &nh);
    protected:
        CartesianMatrix updateInertia() override;  
        CartesianVector updateForce() override;     
       
    private:    
        panda_controllers_extended::InertiaConfig config_;
        std::unique_ptr<dynamic_reconfigure::Server<panda_controllers_extended::InertiaConfig>>
                                                                dynamic_server_;
        void inertiaParamCallback(panda_controllers_extended::InertiaConfig& config,
                                    uint32_t level);
};