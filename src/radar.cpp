#include "radar.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

Radar::Radar(const double std_radr, const double std_radphi, const double std_radrd) : std_radr(std_radr), std_radphi(std_radphi), std_radrd(std_radrd) {
}

void Radar::Initialize(const MeasurementPackage& m, VectorXd& x, MatrixXd& P) {

}

void Radar::Update(const MeasurementPackage& m, VectorXd& x, MatrixXd& P) {

}
