#ifndef LIDAR_H
#define LIDAR_H

#include "Eigen/Dense"
#include "measurement_package.h"


class Lidar {
private:
  double std_laspx, std_laspy;

public:
  Lidar(const double std_laspx, const double std_laspy);
  void Initialize(const MeasurementPackage& m, Eigen::VectorXd& x, Eigen::MatrixXd& P);
  void Update(const MeasurementPackage& m, Eigen::VectorXd& x, Eigen::MatrixXd& P);
};

#endif
