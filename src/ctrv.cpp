#include "ctrv.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

CTRV::CTRV(const double std_a, const double std_yawdd) : std_a_(std_a), std_yawdd_(std_yawdd) {
}

void CTRV::Predict(const double delta_t, VectorXd& x, MatrixXd& P) {
}
