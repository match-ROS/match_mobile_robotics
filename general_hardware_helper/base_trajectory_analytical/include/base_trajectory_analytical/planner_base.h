#pragma once

#include<ros/ros.h>
#include<tf/tf.h>
#include<math.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include<std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>


//Generic path planning class
#define PARAM_ITERATION "iterations"
#define PARAM_REFERENCE "reference"
/**
 * @brief A generic planner class that provides general interface functions and architecture
 * 
 */
class PlannerBase{
   
    public:
       
        /**
         * @brief Construct a new Planner object
         * 
         * @param nh Ros Nodehandle
         */
        PlannerBase(ros::NodeHandle &nh);
        /**
         * @brief Starts the planner
         * 
         */
        void start();
         /**
         * @brief Pauses the planner
         * 
         */
        void pause();

        /**
         * @brief Set the Start Pose
         * 
         * @param pose pose to be set
         */
        void setStartPose(tf::Pose pose);
        
        /**
         * @brief Load parameters from the parameter server
         * 
         */
        void load();
        
        /**
         * @brief Load child parameter from the parameter server (To be changed)
         * 
         */
        virtual void loadChild()=0;
    
    protected:
        int iterations_counter;                                 ///Counter for the iterations of periodic planner functions
        ros::NodeHandle nh;
        int iterations;        
        tf::Transform start_reference;                          //Transformation for modifieing offset to initial pose
        nav_msgs::Path trajectory_;

    private:
        void resetCallback(geometry_msgs::PoseWithCovarianceStamped msg);
        void calc_values(ros::Duration local_time);
        void simulate();
        ros::Timer tim_sampling;                                //Timer for publishing trajectory
        ros::Publisher pub_current_odometry;
        ros::Publisher pub_current_vel;
        ros::Publisher pub_current_pose;
        ros::Publisher pub_path;
        ros::Subscriber sub_reset_pose;

        ros::ServiceServer set_start_service;                   //Service for starting the planner
        ros::ServiceServer set_stop_service;                    //Service for shutting the planner down

        bool is_planning;                                       //Flag if the planner is planning at the moment
     
                                              //Number of Iterations to do

        std::string frame_name;                                 //Frame in wich the pose is calculated

        ros::Duration paused;                                   //Time the planner paused
        ros::Time start_time;                                   //Time at wich the planning procedure started

        tf::Pose start_pose;
        tf::Vector3 planned_vel;

        tf::Vector3 vel;
        tf::Vector3 pos;
        tf::Quaternion orientation;
        double ang_vel;
        
   

        //Planning Scope as handle for timer event
        void plan(const ros::TimerEvent& events);
        void publish();

        ///Gets the transformation form the given pose to the pose at timestamp zero
        ///@param pose : Pose from wich the planner should start
        tf::Transform getTransform(tf::Pose pose); 

        ///Transforming all values that are not rotation or translation invariant
        ///@param trafo : The Transformation applied to all values
        void transformValues(tf::Transform trafo,tf::Vector3 &pos,tf::Vector3 &vel,tf::Quaternion &ori);       

        ///service procedure for satrting the planner
        ///@param req : Reqest the service is getting
        ///@param res : Response the service is sending
        bool srv_start( std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
        ///service procedure for stopping the planner
        ///@param req : Reqest the service is getting
        ///@param res : Response the service is sending
        bool srv_stop( std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);

        

        //Virtual function to overload by child classes####################################  
        //Determines the velocity at a specific time
        //@param time       
        virtual tf::Vector3 get_position(ros::Duration time)=0;
        virtual tf::Quaternion get_orientation(ros::Duration time)=0;
        virtual tf::Vector3 get_velocity(ros::Duration time)=0;
        virtual tf::Vector3 get_acceleration(ros::Duration time)=0;
        virtual double get_angular_velocity(ros::Duration time)=0;
        virtual void check_period(ros::Duration time)=0;         
        
};
