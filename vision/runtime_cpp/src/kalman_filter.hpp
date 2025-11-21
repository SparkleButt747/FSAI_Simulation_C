/**
 * @brief 2D constant-velocity Kalman filter for smoothing noisy cone position measurements.
 */
#pragma once
#include <Eigen/Dense>
namespace fsai::vision {
	class Kalman_filter {
	public:
		// Internal matrix typedefs
		using Vec4 = Eigen::Matrix<double,4,1>;
		using Vec2 = Eigen::Matrix<double,2,1>;
		using Mat4 = Eigen::Matrix<double,4,4>;
		using Mat2 = Eigen::Matrix<double,2,2>;
		using Mat2x4 = Eigen::Matrix<double,2,4>;
		using Mat4x2 = Eigen::Matrix<double,4,2>;

		/**
		 * @brief Constructor for the Kalman filter with a fixed time step.
		 * @param dt Time step delta time between updates, in seconds.
		 */
		explicit Kalman_filter(double dt);

		/**
		 * @brief Initializes the filter state with an initial position.
		 * @param x Initial x-coordinate.
		 * @param y Initial y-coordinate.
		 *
		 * P.S. The initial velocity is set to zero because cones are stationary.
		 */
		void initialize(double x, double y);

		/**
		 * @brief Predicts the next state
		 */
		void predict();

		/**
		 * @brief Updates the state estimate using a new observed x and y measurement.
		 * @param mx new x position.
		 * @param my new y position.
		 */
		void update(double mx, double my);

		/**
		 * @brief Returns the current estimated state.
		 * @return 4 x 1 Vector x y vx vy.
		 */
		[[nodiscard]] Vec4 state() const;

		/**
		 * @brief Returns the current state covariance matrix.
		 * @return 4×4 covariance matrix.
		 */
		[[nodiscard]] Mat4 covariance() const;

	private:
		// Delta time
		double dt_;
		// State
		Vec4 x_;
		// Covariance
		Mat4 P_;
		// State transition
		Mat4 F_;
		// Measurement
		Mat2x4 H_;
		// Noise process
		Mat4 Q_;
		// Noise measurement
		Mat2 R_;
	};
}