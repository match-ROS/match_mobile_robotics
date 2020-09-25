#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>




class MoveBase{
    public:
        MoveBase(ros::NodeHandle &nh);

    protected:
        ros::NodeHandle nh_;
        virtual void init();
        virtual void scope(const ros::TimerEvent &);
        tf2::Transform pose_; 
        tf2::Transform initial_pose_; 
        tf2::Transform planning_offset_;
    private:
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_;
        ros::Timer scope_timer_;

        ros::Publisher pose_pub_;
        
        std::string ee_frame_;
        std::string base_frame_;
        std::string planning_frame_;

      
               

};