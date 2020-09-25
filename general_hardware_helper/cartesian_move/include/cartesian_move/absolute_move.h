
#include<cartesian_move/move_base.h>

class AbsoluteMove:public MoveBase{
    public:
        AbsoluteMove(ros::NodeHandle &nh);
    
    private:
        void inputMsgCallback(geometry_msgs::PoseStamped msg);
        bool inputSrvCallback(cartesian_move::SetPoseRequest &req,cartesian_move::SetPoseResponse &res);
       
};