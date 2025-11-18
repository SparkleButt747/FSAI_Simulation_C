#include "kalman_filter.hpp"

namespace fsai::vision
{
	class Kalman_filter
	{
	public:
		using Vec4 = Eigen::Matrix<double,4,1>;
		using Vec2 = Eigen::Matrix<double,2,1>;
		using Mat4 = Eigen::Matrix<double,4,4>;
		using Mat2 = Eigen::Matrix<double,2,2>;
		using Mat2x4 = Eigen::Matrix<double,2,4>;
		using Mat4x2 = Eigen::Matrix<double,4,2>;

		explicit Kalman_filter(const double dt) : dt_(dt)
		{
			F_.setIdentity();
			F_(0,2) = dt_;
			F_(1,3) = dt_;

			H_.setZero();
			H_(0,0) = 1;
			H_(1,1) = 1;

			Q_.setZero();
			// How accurate do we think the x, and y are
			Q_(0,0) = 0.01;
			Q_(1,1) = 0.01;
			// How accurate do we think the velocity x and y are
			Q_(2,2) = 0.5;
			Q_(3,3) = 0.5;

			R_.setIdentity();
			// How accurate do we think the measurements are?
			R_ *=0.05;

			P_.setIdentity();
			P_*=1.0;

			x_.setZero();
		}

		void initialize(const double x, const double y)
		{
			x_ << x, y, 0, 0;
		}

		void predict()
		{
			x_ = F_ * x_;
			P_ = F_ * P_ * F_.transpose() + Q_;
		}

		void update(const double mx, const double my)
		{
			Vec2 z;
			z << mx, my;

			const Vec2 y = z - H_ * x_;
			const Mat2 S = H_ * P_ * H_.transpose() + R_;
			Mat4x2 K = P_ * H_.transpose() * S.inverse();

			x_ = x_ + K * y;

			const Mat4 I = Mat4::Identity();
			// Joseph form
			P_ = (I - K * H_) * P_ * (I - K * H_).transpose() + K * R_ * K.transpose();
		}

		[[nodiscard]] Vec4 state() const
		{
			return x_;
		}
		[[nodiscard]] Mat4 covariance() const
		{
			return P_;
		}

	private:
		double dt_;
		Vec4 x_;
		Mat4 P_;
		Mat4 F_;
		Mat2x4 H_;
		Mat4 Q_;
		Mat2 R_;
	};
}



