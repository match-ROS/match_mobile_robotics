#include<mir_helper/primitive_planner.h>

//Implementation of a StepResponse planner that gives a spiral########################################################################################
//#######################################################################################################################################

PrimitivePlanner::PrimitivePlanner(ros::NodeHandle &nh):PlannerBase(nh),time_offset_(0.0),index_of_primitive_(0)
{
    this->list_of_primitives_.push_back(new LinePrimitive(0.1,0.5,4));
    this->list_of_primitives_.push_back(new CirclePrimitive(0.1,0.5,3,1.57));    
    this->list_of_primitives_.push_back(new LinePrimitive(0.1,0.5,4));
    this->list_of_primitives_.push_back(new CirclePrimitive(0.1,0.5,3,1.57));    
    this->list_of_primitives_.push_back(new LinePrimitive(0.1,0.5,4));
    this->list_of_primitives_.push_back(new CirclePrimitive(0.1,0.5,3,1.57));    
    this->list_of_primitives_.push_back(new LinePrimitive(0.1,0.5,4));
    this->list_of_primitives_.push_back(new CirclePrimitive(0.1,0.5,3,1.57));   

    this->current_it_=this->list_of_primitives_.begin();
    this->current_primitive_=*(this->current_it_);
    
    if(!this->checkCompatibility(this->list_of_primitives_))
    {
        ROS_WARN("Incompatibility in primitive list occured!");
    }
}
PrimitivePlanner::~PrimitivePlanner()
{
    delete this->current_primitive_;
    this->list_of_primitives_.clear();
}   
void PrimitivePlanner::loadChild()
{
}
tf::Vector3 PrimitivePlanner::get_position(ros::Duration time)
{
    std::vector<tf::Transform> pos=this->current_primitive_->getPosition();
    
    return pos.at(this->lookupIndexTime(time.toSec())).getOrigin();
}
tf::Quaternion PrimitivePlanner::get_orientation(ros::Duration time)
{
    std::vector<tf::Transform> pos=this->current_primitive_->getPosition();
  
    return pos.at(this->lookupIndexTime(time.toSec())).getRotation();
}
tf::Vector3 PrimitivePlanner::get_velocity(ros::Duration time)
{
    std::vector<tf::Vector3> vel=this->current_primitive_->getVelocity();

    return vel.at(this->lookupIndexTime(time.toSec()));
}
double PrimitivePlanner::get_angular_velocity(ros::Duration time)
{
    std::vector<double> vel=this->current_primitive_->getAngularVelocity();
  
    return vel.at(this->lookupIndexTime(time.toSec()));
}
tf::Vector3 PrimitivePlanner::get_acceleration(ros::Duration time)
{

}

int PrimitivePlanner::lookupIndexTime(double time)
{
    double complete_time=this->current_primitive_->getTime();
    int index=(int)floor((time-this->time_offset_)/complete_time*this->current_primitive_->getSize());
    if(index<0)
    {
        ROS_WARN("Index below zero!");
        index=0;
    }
    if(index>=this->current_primitive_->getSize())
    {
        index=this->current_primitive_->getSize()-1;
        ROS_WARN("Reached end of primitive!");
    }
    return index;
}
bool PrimitivePlanner::checkCompatibility(std::list<Primitive*> list)
{
    list.front()->interpolate(0.01);
    tf::Transform last_position=list.front()->getPosition().back();
    double last_vel=list.front()->getVelocity().back().length();
    bool succeed=true;

    for(auto element=list.begin()++;element!=list.end();element++)
    {
        (*element)->start_vel_=last_vel;
        (*element)->start_point_=last_position;
        if(!(*element)->interpolate(0.01))
        {
            succeed=false;
        }
        last_position=(*element)->getPosition().back();
        last_vel=(*element)->getVelocity().back().length();
    }     
    return succeed;
}
void PrimitivePlanner::check_period(ros::Duration time)
{
    if(time.toSec()>=this->current_primitive_->getTime()+this->time_offset_)
    {
       
       if(this->current_it_!=this->list_of_primitives_.end())
       {
           ROS_INFO_STREAM("Changing primitive!");
           this->time_offset_+=this->current_primitive_->getTime();
           this->current_it_++;
           this->current_primitive_=*(this->current_it_);
       }
       else
       {
           this->stop();
       }
       
    }
}

PrimitivePlanner::Primitive::Primitive():start_point_(tf::Transform::getIdentity()),
                                         start_vel_(0.0),
                                         start_ang_vel_(0.0),
                                         time_(0.0)
{
}
std::vector<tf::Transform> PrimitivePlanner::Primitive::getPosition()
{
    return this->positions_;
}  
std::vector<tf::Vector3> PrimitivePlanner::Primitive::getVelocity()
{
    return this->velocities_;
}
std::vector<double> PrimitivePlanner::Primitive:: getAngularVelocity()
{
    return this->angular_velocities_;
}
std::vector<tf::Vector3> PrimitivePlanner::Primitive::getAccecleration()
{
    return this->accelerations_;
}
int PrimitivePlanner::Primitive::getTime()
{
    return this->time_;
}
int PrimitivePlanner::Primitive::getSize()
{
    return this->positions_.size();
}







PrimitivePlanner::LinePrimitive::LinePrimitive(double accel,double vel_max,double length):
                                                                                            accel_(accel),
                                                                                            vel_max_(vel_max),
                                                                                            length_(length),
                                                                                            Primitive()
{
    
}

bool PrimitivePlanner::LinePrimitive::interpolate(double linspace)
{
    this->positions_.clear();
    this->velocities_.clear();
    this->accelerations_.clear();
    this->angular_velocities_.clear();
   

    tf::Transform rot=tf::Transform(this->start_point_.getRotation(),tf::Vector3(0.0,0.0,0.0));
    this->accelerations_.push_back(this->accel_*(rot*tf::Vector3(1.0,0.0,0.0)));
    this->velocities_.push_back(this->start_vel_*(rot*tf::Vector3(1.0,0.0,0.0)));   
    this->positions_.push_back(this->start_point_);

    
    double length=0.0;
    double time=0.0;

    bool vel_reached=false;
    bool abort=false;
    int iterations=0;

    double vel=this->start_vel_;
    

    while(!abort && iterations<100000)                                                                                                                                                                                                         
    {   
        
        double d_vel=this->accel_*linspace;        
        if(vel+d_vel>this->vel_max_)
        {
            d_vel=this->vel_max_-vel;
            vel_reached=true;
        }
        
        double d_l=(vel+0.5*d_vel)*linspace;
        if(length+d_l>this->length_)
        {
            double scale=(this->length_-length)/d_l;
            d_l=scale*d_l;
            linspace=scale*linspace;
            abort=true;
        }
        vel+=d_vel;               
        length+=d_l;
        time+=linspace;

       
        this->accelerations_.push_back(this->accel_*(rot*tf::Vector3(1.0,0.0,0.0)));
        this->velocities_.push_back(vel*(rot*tf::Vector3(1.0,0.0,0.0)));        
        this->positions_.push_back(tf::Transform((this->start_point_).getRotation(),
                                                  this->start_point_*(length*(tf::Vector3(1.0,0.0,0.0)))));


       
        iterations++;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    }
    for(int i=0;i<this->velocities_.size();i++)
    {
        this->angular_velocities_.push_back(0.0);
    }
    this->time_=time;
    return vel_reached;                                                                                                                                                                                                                                                                                                                                                                                                                                                 
}



PrimitivePlanner::CirclePrimitive::CirclePrimitive(double accel, double vel_max, double radius, double angle):
                                                     angle_(angle),
                                                     radius_(radius),
                                                     accel_(accel),
                                                     vel_max_(vel_max)
{
    
}
bool PrimitivePlanner::CirclePrimitive::interpolate(double linspace)
{
    this->positions_.clear();
    this->velocities_.clear();
    this->accelerations_.clear();
    this->angular_velocities_.clear();
    
    tf::Transform rot=tf::Transform(tf::createQuaternionFromYaw(0)*this->start_point_.getRotation(),tf::Vector3(0.0,0.0,0.0));
    this->velocities_.push_back(this->start_vel_*(rot*tf::Vector3(1.0,0.0,0.0)));
    this->accelerations_.push_back(this->accel_*(rot*tf::Vector3(1.0,0.0,0.0)));    
    this->positions_.push_back(tf::Transform((this->start_point_*rot).getRotation(),
                                                this->start_point_.getOrigin()));

    
    double length=0.0;
    double time=0.0;

    bool vel_reached=false;
    bool abort=false;
    int iterations=0;

    double ang_acc=this->accel_/this->radius_;
    double omega=this->start_vel_/this->radius_;
    double omega_max=this->vel_max_/this->radius_;


    while(!abort && iterations<100000)                                                                                                                                                                                                         
    {   
        double d_omega=ang_acc*linspace;        
        if(omega+d_omega>omega_max)
        {
            d_omega=omega_max-omega;
            vel_reached=true;
        }
        
        double d_l=(omega+0.5*d_omega)*linspace;
        if(length+d_l>this->angle_)
        {
            double scale=(this->angle_-length)/d_l;
            d_l=scale*d_l;
            linspace=scale*linspace;
            abort=true;
        }
        omega+=d_omega;               
        length+=d_l;
       
        rot.setRotation(this->start_point_.getRotation()*tf::createQuaternionFromYaw(length));
        tf::Vector3 d_pos=d_l*this->radius_*(rot*tf::Vector3(1.0,0.0,0.0));
        
        this->accelerations_.push_back(ang_acc*this->radius_*(rot*tf::Vector3(1.0,0.0,0.0)));
        this->velocities_.push_back(omega*this->radius_*(rot*tf::Vector3(1.0,0.0,0.0)));        
        this->positions_.push_back(tf::Transform(rot.getRotation(),
                                            this->positions_.back().getOrigin()+d_pos));
       
        time+=linspace;
        iterations++;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    }
    for(int i=0;i<this->velocities_.size();i++)
    {
        this->angular_velocities_.push_back(omega);
    }
    this->time_=time;
    return vel_reached;                                                                                                                                                                                                                                                                                                                                                                                                                        
}

