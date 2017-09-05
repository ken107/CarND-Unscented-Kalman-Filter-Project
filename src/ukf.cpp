#include "ukf.h"
#include "tools.h"

UKF::UKF() : ctrv_(1, M_PI/8), lidar_(.15, .15), radar_(.3, .03, .3), x_(5), P_(5, 5), time_us_(0), use_lidar_(true), use_radar_(true) {
}

void UKF::ProcessMeasurement(const MeasurementPackage& m) {
  if (m.sensor_type_ == MeasurementPackage::LASER && !use_lidar_ ||
    m.sensor_type_ == MeasurementPackage::RADAR && !use_radar_) return;

  if (time_us_ == 0) {
    time_us_ = m.timestamp_;

    //initialize
    if (m.sensor_type_ == MeasurementPackage::LASER) lidar_.Initialize(m, x_, P_);
    else if (m.sensor_type_ == MeasurementPackage::RADAR) radar_.Initialize(m, x_, P_);
    else throw std::invalid_argument("Bad seensor type");
  }
  else {
    float delta_t = (m.timestamp_ - time_us_) / 1000000.0;
    time_us_ = m.timestamp_;

    //prediction
    MatrixXd Xsig_pred;
    ctrv_.Predict(delta_t, x_, P_, Xsig_pred);

    //update
    if (m.sensor_type_ == MeasurementPackage::LASER) lidar_.Update(m, x_, P_);
    else if (m.sensor_type_ == MeasurementPackage::RADAR) radar_.Update(m, x_, P_, Xsig_pred);
    else throw std::invalid_argument("Bad seensor type");
  }
}
