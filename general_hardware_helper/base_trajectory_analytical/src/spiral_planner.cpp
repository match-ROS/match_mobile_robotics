#include <base_trajectory_analytical/spiral_planner.h>

Spiralplanner::Spiralplanner(ros::NodeHandle &nh):PlannerBase(nh)
{
 
}
void Spiralplanner::set_parameter(double r_offset,double r_growth, double omega)
{
    this->plan.r_offset=r_offset;
    this->plan.r_growth=r_growth;
    this->plan.omega=omega;
}


tf::Vector3 Spiralplanner::get_position(ros::Duration time)
{
    double t=time.toSec();    
    tf::Vector3 pos(sin(this->plan.omega*t)*(this->plan.r_offset+this->plan.r_growth*this->plan.omega*t),
                    -cos(this->plan.omega*t)*(this->plan.r_offset+this->plan.r_growth*this->plan.omega*t),
                    0);
    return pos;

}

tf::Quaternion Spiralplanner::get_orientation(ros::Duration time)
{
    return tf::createQuaternionFromYaw(this->plan.omega*time.toSec());
}

tf::Vector3 Spiralplanner::get_velocity(ros::Duration time)
{
    double t=time.toSec();
    tf::Vector3 vel(cos(this->plan.omega*t)*(this->plan.r_offset+this->plan.r_growth*this->plan.omega*t)+sin(this->plan.omega*t)*this->plan.r_growth*this->plan.omega,
                    sin(this->plan.omega*t)*(this->plan.r_offset+this->plan.r_growth*this->plan.omega*t)-cos(this->plan.omega*t)*this->plan.r_growth*this->plan.omega,
                    0.0);
    return vel;
}
 tf::Vector3 Spiralplanner::get_acceleration(ros::Duration time)
 {
     return tf::Vector3();
 }

double Spiralplanner::get_angular_velocity(ros::Duration time)
{
    return this->plan.omega;    
}

void Spiralplanner::loadChild()
{
    ros::NodeHandle priv("~");
    try{
        priv.getParam(PARAM_ITERATION,this->iterations);
        priv.getParam(PARAM_GROWTH,this->plan.r_growth);
        priv.getParam(PARAM_OMEGA,this->plan.omega);
        priv.getParam(PARAM_R0,this->plan.r_offset);
        ROS_INFO("Loaded %s : R_OFfset: %f  r_growth %f Omega: %f",    
                                                        ros::this_node::getName().c_str(),
                                                        this->plan.r_offset,
                                                        this->plan.r_growth,
                                                        this->plan.omega);
    }
    catch(...)
    {
        ROS_INFO("Error due loading paramter of: %s",ros::this_node::getName().c_str());
    }   
}

void Spiralplanner::check_period(ros::Duration time)
{
    if(this->plan.omega*time.toSec()>2*M_PI)
    {
        this->iterations_counter++;
    }
}
