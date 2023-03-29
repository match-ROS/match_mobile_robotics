# include <ur_calibrated_pose_pub/utils/dh_transformation.h>

namespace dh_utils
{
    DHTransformation::DHTransformation()
    {

    }

    DHTransformation::DHTransformation(double theta, double d, double a, double alpha)
    {
        this->d_ = d;
        this->a_ = a;
        this->theta_ = theta;
        this->alpha_ = alpha;
    }

    // Getter Methods
    double DHTransformation::getTheta()
    {
        return this->theta_;
    }

    double DHTransformation::getd()
    {
        return this->d_;
    }

    double DHTransformation::geta()
    {
        return this->a_;
    }

    double DHTransformation::getAlpha()
    {
        return this->alpha_;
    }

    // Eigen::Matrix4d dh_transformation(double d, double a, double theta, double alpha)
    // {
    //     Eigen::Matrix4d T;
    //     T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
    //         sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
    //         0, sin(alpha), cos(alpha), d,
    //         0, 0, 0, 1;
    //     return T;
    // }
}