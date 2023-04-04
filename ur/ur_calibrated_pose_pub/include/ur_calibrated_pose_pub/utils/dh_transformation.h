#ifndef DH_PARAMETER_SET_H_INCLUDED
#define DH_PARAMETER_SET_H_INCLUDED

#include <memory>
#include <Eigen/Dense>

namespace dh_utils
{
    class DHTransformation
    {
    public:
        DHTransformation();
        DHTransformation(double theta, double d, double a, double alpha);
        virtual ~DHTransformation() = default;

        double getTheta();
        double getd();
        double geta();
        double getAlpha();

        double getCalibratedTheta();
        double getCalibratedd();
        double getCalibrateda();
        double getCalibratedAlpha();

        Eigen::Matrix4d getTransformationMatrix();

        void setDeltaTransformation(DHTransformation delta_transformation);
        void setTransformationMatrices();
        
    private:
        double theta_;
        double d_;
        double a_;
        double alpha_;

        Eigen::Matrix4d theta_transformation_matrix_;
        Eigen::Matrix4d d_transformation_matrix_;
        Eigen::Matrix4d a_transformation_matrix_;
        Eigen::Matrix4d alpha_transformation_matrix_;
        Eigen::Matrix4d transformation_matrix_;

        std::shared_ptr<DHTransformation> delta_transformation_ = nullptr;
    };
}  // namespace dh_utils
#endif  // ifndef DH_PARAMETER_SET_H_INCLUDED
