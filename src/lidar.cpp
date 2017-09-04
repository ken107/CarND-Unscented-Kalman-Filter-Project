#include "lidar.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

Lidar::Lidar(const double std_laspx, const double std_laspy) : std_laspx(std_laspx), std_laspy(std_laspy) {
}

void Lidar::Initialize(const MeasurementPackage& m, VectorXd& x, MatrixXd& P) {
  //initialize state vector
  x << m.raw_measurements_[0], m.raw_measurements_[1], 0, 0, 0;

  //initialize state covariance matrix
	P << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 1000, 0, 0,
      0, 0, 0, 1000, 0,
      0, 0, 0, 0, 1000;
}

void Lidar::Update(const MeasurementPackage& m, VectorXd& x, MatrixXd& P) {
  //ground truth
  VectorXd z = VectorXd(2);
  z << m.raw_measurements_[0], m.raw_measurements_[1];

  //measurement function
  MatrixXd H(2,5);
  H << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0;

  //noise covariance matrix
  MatrixXd R(2,2);
  R << std_laspx, 0,
      0, std_laspy;

  //Kalman gain
  MatrixXd Ht = H.transpose();
  VectorXd y = z - H * x;
  MatrixXd S = H * P * Ht + R;
  MatrixXd K = P * Ht * S.inverse();

  //new estimate
  x = x + (K * y);
  long x_size = x.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P = (I - K * H) * P;
}
