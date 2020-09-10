#include <mir_helper/planner_base.h>

#define PARAM_R0 "r0"
#define PARAM_GROWTH "growth"
#define PARAM_OMEGA "omega"

class Spiralplanner:public PlannerBase{
    public:
        Spiralplanner(ros::NodeHandle &nh);
        struct SpiralPlan
        {
            float r_offset;
            float r_growth;           
            float omega;
        };
        //Sets the parameter of the planner
        void set_parameter(double r_offset,double r_growth, double omega);
        void loadChild();
    private:
        SpiralPlan plan;
        tf::Vector3 get_position(ros::Duration time);
        tf::Quaternion get_orientation(ros::Duration time);
        tf::Vector3 get_velocity(ros::Duration time);
        double get_angular_velocity(ros::Duration time);
        tf::Vector3 get_acceleration(ros::Duration time);
        void check_period(ros::Duration time);     
};