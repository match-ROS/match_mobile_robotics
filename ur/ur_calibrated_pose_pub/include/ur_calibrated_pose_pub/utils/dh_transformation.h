#ifndef DH_PARAMETER_SET_H_INCLUDED
#define DH_PARAMETER_SET_H_INCLUDED

#include <memory>

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

        void setDeltaTransformation(DHTransformation delta_transformation);
        
    private:
        double theta_;
        double d_;
        double a_;
        double alpha_;

        std::shared_ptr<DHTransformation> delta_transformation_ = nullptr;
    };
}  // namespace dh_utils
#endif  // ifndef DH_PARAMETER_SET_H_INCLUDED
