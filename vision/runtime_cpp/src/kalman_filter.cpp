#include "kalman_filter.hpp"

namespace fsai::vision
{
		Kalman_filter::Kalman_filter(const double dt) : dt_(dt)
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
			P_.block<2,2>(0,0) *= 0.05;
			P_.block<2,2>(2,2) *= 1e6;

			x_.setZero();
		}

		void Kalman_filter::initialize(const double x, const double y)
		{
			x_ << x, y, 0, 0;
		}

		void Kalman_filter::predict()
		{
			x_ = F_ * x_;
			P_ = F_ * P_ * F_.transpose() + Q_;
		}

		void Kalman_filter::update(const double mx, const double my)
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

		Kalman_filter::Vec4 Kalman_filter::state() const
		{
			return x_;
		}
		Kalman_filter::Mat4 Kalman_filter::covariance() const
		{
			return P_;
		}
}



