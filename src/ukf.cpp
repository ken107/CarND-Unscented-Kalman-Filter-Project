#include "ukf.h"

UKF::UKF() : ctrv_(30, 30), lidar_(.15, .15), radar_(.3, .03, .3), x_(5), P_(5, 5), time_us_(0), use_lidar_(true), use_radar_(false) {
}

void UKF::ProcessMeasurement(const MeasurementPackage& m) {
  if (m.sensor_type_ == MeasurementPackage::LASER) ProcessLidarMeasurement(m);
  else if (m.sensor_type_ == MeasurementPackage::RADAR) ProcessRadarMeasurement(m);
}

void UKF::ProcessLidarMeasurement(const MeasurementPackage& m) {
  if (!use_lidar_) return;


}

void UKF::ProcessRadarMeasurement(const MeasurementPackage& m) {
  if (!use_radar_) return;

  if (time_us_ == 0) {
    time_us_ = m.timestamp_;
    lidar_.Initialize(m, x_, P_);
  }
  else {
    float delta_t = (m.timestamp_ - time_us_) / 1000000.0;
    time_us_ = m.timestamp_;

    MatrixXd Xsig_pred;
    ctrv_.Predict(delta_t, x_, P_, Xsig_pred);

    lidar_.Update(m, x_, P_);
  }
}
