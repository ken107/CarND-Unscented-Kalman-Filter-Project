#ifndef CTRV_H
#define CTRV_H

#include "Eigen/Dense"


class CTRV {
private:
  double std_a_, std_yawdd_;

public:
  CTRV(const double std_a, const double std_yawdd);
  void Predict(const double delta_t, Eigen::VectorXd& x, Eigen::MatrixXd& P);
};

#endif
