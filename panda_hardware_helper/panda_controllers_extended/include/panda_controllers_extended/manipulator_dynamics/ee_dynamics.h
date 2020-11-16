#include<ros/ros.h>
#include<eigen3/Eigen/Dense>
#include<geometry_msgs/TwistStamped.h>
#include<geometry_msgs/WrenchStamped.h>

class EEDynamics{
    public:
        EEDynamics(ros::NodeHandle &nh);

    protected:
        typedef Eigen::Matrix<double,6,6> CartesianMatrix;
        typedef Eigen::Matrix<double,6,1> CartesianVector;

        CartesianMatrix inertia_;
        CartesianVector force_;
        CartesianVector accel_;
        

        ros::NodeHandle nh_;
        void update(const ros::TimerEvent&);
        virtual CartesianMatrix updateInertia()=0;  
        virtual CartesianVector updateForce()=0;        
    
    private:        
        
        ros::Subscriber base_accel_sub_;
        ros::Publisher force_pub_;
        ros::Timer update_timer_;

        void accelCallback(geometry_msgs::TwistStamped msg);
};