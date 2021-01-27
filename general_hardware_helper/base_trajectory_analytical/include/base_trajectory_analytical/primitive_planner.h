#pragma once

#include <base_trajectory_analytical/planner_base.h>

class PrimitivePlanner:public PlannerBase{
    public:
        PrimitivePlanner(ros::NodeHandle &nh);
        ~PrimitivePlanner();
        class Primitive
        {
            public:
                Primitive();
               
                tf::Transform start_point_;
                double start_vel_;
                double start_ang_vel_;
                double time_;

                std::vector<tf::Transform> positions_;
                std::vector<tf::Vector3> velocities_;
                std::vector<double> angular_velocities_;
                std::vector<tf::Vector3> accelerations_;     
                
                   
                std::vector<tf::Transform> getPosition();    
                std::vector<tf::Vector3> getVelocity();  
                std::vector<double> getAngularVelocity();
                std::vector<tf::Vector3> getAccecleration();
                
                int getTime();
                int getSize(); 

                virtual bool interpolate(double linspace)=0;      
        };
        class LinePrimitive: public Primitive
        {
            public:
                LinePrimitive(double accel,double vel_max,double length);
                double accel_;
                double vel_max_; 
                double length_;                    
                bool interpolate(double linspace);    
        };
        class CirclePrimitive: public Primitive
        {
            public:
                CirclePrimitive(double accel, double vel_max, double radius, double angle);
                double angle_;
                double radius_;
                double accel_;
                double vel_max_;
                bool interpolate(double linspace);                  
        };

        struct PrimitivePlan
        {  
            double ang_acc_lim_;
            double trans_acc__lim_;
            double ang_vel_lim_;
            double trans_vel__lim_;
            std::vector<Primitive> primitives_;
            PrimitivePlan()
            {;}
        };

        void loadChild();
    private:
        Primitive* current_primitive_;
        std::list<Primitive*>::iterator current_it_;
        std::list<Primitive*> list_of_primitives_;
        PrimitivePlan plan;
        int index_of_primitive_;
        double time_offset_;

        bool checkCompatibility(std::list<Primitive*> list);

        int lookupIndexTime(double time);
        
        tf::Vector3 get_position(ros::Duration time);

        tf::Quaternion get_orientation(ros::Duration time);

        tf::Vector3 get_velocity(ros::Duration time);

        double get_angular_velocity(ros::Duration time);

        tf::Vector3 get_acceleration(ros::Duration time);

        void check_period(ros::Duration time);     
};