#ifndef RADAR_H
#define RADAR_H

#include "Eigen/Dense"
#include "measurement_package.h"


class Radar {
private:
  double std_radr, std_radphi, std_radrd;

public:
  Radar(const double std_radr, const double std_radphi, const double std_radrd);
  void Initialize(const MeasurementPackage& m, Eigen::VectorXd& x, Eigen::MatrixXd& P);
  void Update(const MeasurementPackage& m, Eigen::VectorXd& x, Eigen::MatrixXd& P);
};

#endif
