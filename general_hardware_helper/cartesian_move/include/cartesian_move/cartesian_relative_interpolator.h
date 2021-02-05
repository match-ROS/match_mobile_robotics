#include <cartesian_move/cartesian_interpolator.h>


class CartesianRelativeInterpolator: public CartesianInterpolator{
    public:
        CartesianRelativeInterpolator(ros::NodeHandle &nh);
    protected:
        std::vector<geometry_msgs::PoseStamped> calcPath(   geometry_msgs::PoseStamped source,
                                                            geometry_msgs::PoseStamped target) override;
};
