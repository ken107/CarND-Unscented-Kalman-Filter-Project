#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include "ctrv.h"
#include "lidar.h"
#include "radar.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:
  UKF();
  void ProcessMeasurement(const MeasurementPackage&);

private:
  CTRV ctrv_;
  Lidar lidar_;
  Radar radar_;
  VectorXd x_;
  MatrixXd P_;
  long long time_us_;
  bool use_lidar_;
  bool use_radar_;

  void ProcessRadarMeasurement(const MeasurementPackage&);
  void ProcessLidarMeasurement(const MeasurementPackage&);
  friend int main();
};

#endif /* UKF_H */
