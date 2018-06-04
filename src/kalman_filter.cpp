#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd y = z - H_ * x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = P_ * Ht * Si;

	MatrixXd I;
	x_ = x_ + (K * y);
	long x_size = x_.size();
	I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);
	VectorXd h_x(3);

	float a = sqrt(px * px + py * py);
	float b = 0;
	float c = 0;
	// check if divided by zero
	if (fabs(px) < 0.0001)
	{
		std::cout << "When converting x_ from polar to cartesian coordinates - Division by Zero" << std::endl;
	}
	else
	{
		b = atan2(py, px);
	}
	// check if divided by zero
	if (a < 0.0001)
	{
		std::cout << "When converting x_ from polar to cartesian coordinates - Division by Zero" << std::endl;
	}
	else
	{
		c = (px * vx + py * vy) / a;
	}

	h_x << a, b, c;
	std::cout << h_x << std::endl;
	VectorXd y = z - h_x;
	
	//Normalize angles
	if (y[1] < -M_PI)
	{
		y[1] += 2 * M_PI;
	}
	if (y[1] > M_PI)
	{
		y[1] -= 2 * M_PI;
	}

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = P_ * Ht * Si;

	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}
