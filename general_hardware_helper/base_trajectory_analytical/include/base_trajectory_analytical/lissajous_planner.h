#pragma once

#include <base_trajectory_analytical/planner_base.h>

#define PARAM_OMEGA "omega"
#define PARAM_RATIO "ratio"
#define PARAM_PHASE "phaseshift"
#define PARAM_AMP_X "amplifier_x"
#define PARAM_AMP_Y "amplifier_y"

//Planner Class for generating a lissajous figure path in map frame. 
class LissajousPlanner:public PlannerBase{
    public:
        LissajousPlanner(ros::NodeHandle &nh);
        struct  LissajousPlan{
            float dphi;          //phase shif
            float omegax;       //frequenzy in x direction
            int ratio;          //ratio fy/fx
            float Ax;           //Magnitude x direction
            float Ay;            //Magnitude y directions
        };
        void set_parameter(float omegax,float dphi=0.0, int ratio=2,float Ax=3.0,float Ay=3.0);
        void loadChild();
    
    private:
        LissajousPlan plan;
        //Calclulation of the curren pose dependent on time
        tf::Vector3 get_position(ros::Duration time);
        tf::Quaternion get_orientation(ros::Duration time);
        tf::Vector3 get_velocity(ros::Duration time);
        tf::Vector3 get_acceleration(ros::Duration time);
        double get_angular_velocity(ros::Duration time);
        void check_period(ros::Duration time);           
};


