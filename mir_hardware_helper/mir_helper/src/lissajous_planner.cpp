#include <mir_helper/lissajous_planner.h>

LissajousPlanner::LissajousPlanner(ros::NodeHandle &nh):PlannerBase(nh)
{
}


tf::Vector3 LissajousPlanner::get_position(ros::Duration time)
{
    double t;
    t=time.toSec();
    tf::Vector3 r(0,0,0);
    double x;
    x=this->plan.Ax*sin(this->plan.omegax*t);
    double y;
    y=this->plan.Ay*sin(this->plan.omegax*this->plan.ratio*t+this->plan.dphi);

    return tf::Vector3(x,y,0);
}

tf::Vector3 LissajousPlanner::get_velocity(ros::Duration time)
{
    double t=time.toSec();
    double dx=this->plan.Ax*cos(this->plan.omegax*t)*this->plan.omegax;
    double dy=this->plan.Ay*cos(this->plan.omegax*this->plan.ratio*t+this->plan.dphi)*this->plan.omegax*this->plan.ratio;
    tf::Vector3 vel(dx,
                    dy,
                    0);
    return vel;
}

 tf::Vector3 LissajousPlanner::get_acceleration(ros::Duration time)
 {

    double t=time.toSec();
    double dx=-this->plan.Ax*sin(this->plan.omegax*t)*std::pow(this->plan.omegax,2.0);
    double dy=-this->plan.Ay*sin(this->plan.omegax*this->plan.ratio*t+this->plan.dphi)*std::pow(this->plan.omegax*this->plan.ratio,2.0);
    tf::Vector3 acc(dx,
                    dy,
                    0);
    return acc;
 }

tf::Quaternion LissajousPlanner::get_orientation(ros::Duration time)
{
    double t=time.toSec();
    tf::Vector3 tangent;
    tangent.setX(this->plan.Ax*cos(this->plan.omegax*t)*this->plan.omegax);
    tangent.setY(this->plan.Ay*cos(this->plan.omegax*this->plan.ratio*t+this->plan.dphi)*this->plan.omegax*this->plan.ratio);
    return tf::createQuaternionFromYaw(atan2(tangent.y(),tangent.x()));

}

double LissajousPlanner::get_angular_velocity(ros::Duration time)
{
    tf::Vector3 acc(this->get_acceleration(time));
    tf::Vector3 vel(this->get_velocity(time));
    return (vel.x()*acc.y()-vel.y()*acc.x())/vel.length2();
}


void LissajousPlanner::set_parameter(float omegax,float dphi,int ratio,float Ax,float Ay)
{
    this->plan.Ax=Ax;
    this->plan.Ay=Ay;
    this->plan.omegax=omegax;
    this->plan.dphi=dphi;
    this->plan.ratio=ratio;
}

void LissajousPlanner::loadChild()
{
    ros::NodeHandle priv("~");
    priv.getParam(PARAM_ITERATION,this->iterations);
    priv.getParam(PARAM_AMP_X,this->plan.Ax);
    priv.getParam(PARAM_AMP_Y,this->plan.Ay);
    priv.getParam(PARAM_RATIO,this->plan.ratio);
    priv.getParam(PARAM_PHASE,this->plan.dphi);
    priv.getParam(PARAM_OMEGA,this->plan.omegax);
    ROS_INFO("Loaded %s : AmplifierX %lf AmplifierY %lf Ratio %i PhaseShift %lf Omega: %lf ",   
                                                    ros::this_node::getName().c_str(),
                                                    this->plan.Ax,
                                                    this->plan.Ay,
                                                    this->plan.ratio,
                                                    this->plan.dphi,
                                                    this->plan.omegax);
}

void LissajousPlanner::check_period(ros::Duration time)
{
    if(this->plan.omegax*time.toSec()>(this->iterations_counter+1)*2*M_PI)
    {
        this->iterations_counter++;
    }
}