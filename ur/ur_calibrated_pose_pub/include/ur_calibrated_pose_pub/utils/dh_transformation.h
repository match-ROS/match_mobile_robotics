#ifndef DH_PARAMETER_SET_H_INCLUDED
#define DH_PARAMETER_SET_H_INCLUDED

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
        
    private:
        double theta_;
        double d_;
        double a_;
        double alpha_;
    };
}  // namespace dh_utils
#endif  // ifndef DH_PARAMETER_SET_H_INCLUDED
