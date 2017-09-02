#include "lidar.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

Lidar::Lidar(const double std_laspx, const double std_laspy) : std_laspx_(std_laspx), std_laspy_(std_laspy) {
}

void Lidar::Initialize(const MeasurementPackage& m, VectorXd& x, MatrixXd& P) {

}

void Lidar::Update(const MeasurementPackage& m, VectorXd& x, MatrixXd& P) {

}
