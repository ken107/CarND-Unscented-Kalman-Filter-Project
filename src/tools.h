#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
  static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&);
  static void Print(const char*, const Eigen::VectorXd&);
  static void Print(const char*, const Eigen::MatrixXd&);
};

#endif /* TOOLS_H_ */
