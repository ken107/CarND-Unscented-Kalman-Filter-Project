#include "ukf.h"

UKF::UKF() : ctrv_(30, 30), lidar_(.15, .15), radar_(.3, .03, .3), x_(5), P_(5, 5), time_us_(0) {
}

void UKF::ProcessMeasurement(const MeasurementPackage& m) {
  if (time_us_ == 0) {
    time_us_ = m.timestamp_;

    //initialize
    if (m.sensor_type_ == MeasurementPackage::LASER) lidar_.Initialize(m, x_, P_);
    else if (m.sensor_type_ == MeasurementPackage::RADAR) radar_.Initialize(m, x_, P_);
    else throw std::invalid_argument("Invalid sensor type");
  }
  else {
    float delta_t = (m.timestamp_ - time_us_) / 1000000.0;
    time_us_ = m.timestamp_;

    //prediction
    ctrv_.Predict(delta_t, x_, P_);

    //update
    if (m.sensor_type_ == MeasurementPackage::LASER) lidar_.Update(m, x_, P_);
    else if (m.sensor_type_ == MeasurementPackage::RADAR) radar_.Update(m, x_, P_);
  }
}
