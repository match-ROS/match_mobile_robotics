#include<cartesian_move/cartesian_interpolator.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"cartesian_interpolator");
    ros::NodeHandle nh;
    CartesianInterpolator interpolator(nh);
    ros::spin();
}