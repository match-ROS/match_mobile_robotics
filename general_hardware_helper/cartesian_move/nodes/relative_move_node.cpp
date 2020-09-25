#include<ros/ros.h>
#include<cartesian_move/relative_move.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"relative_move");
    ros::NodeHandle nh;
    RelativeMove relative(nh);
    relative.init();
    ros::spin();

}