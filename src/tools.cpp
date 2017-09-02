#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using namespace std;

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  //validation
  if (estimations.size() != ground_truth.size() || estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  //accumulate squared residuals
  for (int i=0; i<estimations.size(); i++) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  //mean and root
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

void Tools::Print(const char* label, const VectorXd& m) {
  printf("%8s[", label);
  for (int i=0; i<m.size(); i++) {
    printf("%.1f", m(i));
    if (i+1<m.size()) printf("\t");
  }
  printf("]\n");
}

void Tools::Print(const char* label, const MatrixXd& m) {
  for (int i=0; i<m.rows(); i++) {
    printf("%8s[", i==0?label:"");
    for (int j=0; j<m.cols(); j++) {
      printf("%.1f", m(i,j));
      if (j+1<m.cols()) printf("\t");
    }
    printf("]\n");
  }
}
