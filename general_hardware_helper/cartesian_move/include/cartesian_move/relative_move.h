
#include<cartesian_move/move_base.h>
#include<cartesian_move/SetPose.h>

class RelativeMove:public MoveBase{
    public:
        RelativeMove(ros::NodeHandle &nh);
        void init();
    private:    
        void inputMsgCallback(geometry_msgs::PoseStamped msg);
        bool inputSrvCallback(cartesian_move::SetPoseRequest &req,cartesian_move::SetPoseResponse &res);
        ros::Subscriber relative_subscriber_;
        ros::ServiceServer relative_service_;
        tf2::Transform planning_rotation_;
       
};