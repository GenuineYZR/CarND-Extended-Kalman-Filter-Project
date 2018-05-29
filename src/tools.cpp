#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	for (int i = 0; i < estimations.size(); i++) {
		VectorXd residual = estimations(i) - ground_truth(i);
		residual = residual.array() * residual.array();
		rmse += residual;
	}
	rmse = rmse / estimations.size();

	rmse = sqrt(rmse);

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj_(3, 4);

	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float a = px * px + py * py;
	float b = sqrt(a);
	float c = a * b;

	if (fabs(a) < 0.0001) {
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj_;
	}

	Hj_ << px / b, py / b, 0, 0,
		-py / a, px / a, 0, 0,
		py * (vx * py - vy * px) / c, px * (vy * px - vx * py) / c, px / b, py / b;

	return Hj_;
}
