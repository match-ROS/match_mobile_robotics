#include<cartesian_move/cartesian_relative_interpolator.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"cartesian_interpolator");
    ros::NodeHandle nh;
    CartesianRelativeInterpolator interpolator(nh);
    ros::spin();
}