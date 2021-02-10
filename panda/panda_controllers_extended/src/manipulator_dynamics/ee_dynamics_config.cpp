#include <panda_controllers_extended/manipulator_dynamics/ee_dynamics_config.h>

EEDynamicsConfig::EEDynamicsConfig(ros::NodeHandle &nh):EEDynamics(nh)
{
    this->dynamic_server_=  std::make_unique<
                            dynamic_reconfigure::Server<
                            panda_controllers_extended::InertiaConfig>>
                            (this->nh_);

    this->dynamic_server_-> setCallback(
                            boost::bind(&EEDynamicsConfig::inertiaParamCallback, this, _1, _2));
    this->config_=panda_controllers_extended::InertiaConfig();
}

void EEDynamicsConfig::inertiaParamCallback(panda_controllers_extended::InertiaConfig& config,
                                            uint32_t level)
{
    this->config_=config;
}

EEDynamics::CartesianMatrix EEDynamicsConfig::updateInertia()   
{
    Eigen::Matrix3d mass;
    mass<<      this->config_.Mx,0.0,0.0,
                0.0,this->config_.My,0.0,
                0.0,0.0,this->config_.Mz;
    Eigen::Matrix3d inertia;
    inertia<<   this->config_.Jxx,this->config_.Jxy,this->config_.Jxz,
                this->config_.Jxy,this->config_.Jyy,this->config_.Jyz,
                this->config_.Jxz,this->config_.Jyz,this->config_.Jzz;
    
    CartesianMatrix inertia_matrix;
    inertia_matrix.setZero();
    inertia_matrix.topLeftCorner(3,3) =   mass;
    inertia_matrix.bottomRightCorner(3,3)=inertia;
    ROS_INFO_STREAM(inertia_matrix);
    return inertia_matrix;
}
EEDynamics::CartesianVector EEDynamicsConfig::updateForce()  
{
    return this->inertia_*this->accel_;
}