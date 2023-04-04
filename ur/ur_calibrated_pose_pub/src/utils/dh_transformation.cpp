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

	double DHTransformation::getCalibratedTheta()
	{
		return this->theta_ + this->delta_transformation_->getTheta();
	}

	double DHTransformation::getCalibratedd()
	{
		return this->d_ + this->delta_transformation_->getd();
	}

	double DHTransformation::getCalibrateda()
	{
		return this->a_ + this->delta_transformation_->geta();
	}

	double DHTransformation::getCalibratedAlpha()
	{
		return this->alpha_	+ this->delta_transformation_->getAlpha();
	}

	void DHTransformation::setDeltaTransformation(DHTransformation delta_transformation)
	{
		this->delta_transformation_ = std::make_shared<DHTransformation>(delta_transformation);
	}

	void DHTransformation::setTransformationMatrices()
	{
		this->theta_transformation_matrix_ << cos(this->getCalibratedTheta()), -sin(this->getCalibratedTheta()), 0, 0,
			sin(this->getCalibratedTheta()), cos(this->getCalibratedTheta()), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		this->d_transformation_matrix_ << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, this->getCalibratedd(),
			0, 0, 0, 1;

		this->a_transformation_matrix_ << 1, 0, 0, this->getCalibrateda(),
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		this->alpha_transformation_matrix_<< 1, 0, 0, 0,
			0, cos(this->getCalibratedAlpha()), -sin(this->getCalibratedAlpha()), 0,
			0, sin(this->getCalibratedAlpha()), cos(this->getCalibratedAlpha()), 0,
			0, 0, 0, 1;

		// According to the UR documentation the transformation matrix is calculated as follows:
		// trans in Z, rot in Z, trans in X and rot in X
		this->transformation_matrix_ = this->d_transformation_matrix_ * this->theta_transformation_matrix_ * this->a_transformation_matrix_ * this->alpha_transformation_matrix_;	
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