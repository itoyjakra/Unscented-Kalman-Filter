#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::PolarToCart(const VectorXd& polar)
{
    VectorXd cartesian(4);
    float rho = polar(0);
    float phi = polar(1);
    float rho_dot = polar(2);

    cartesian << rho * cos(phi), 
                 rho * sin(phi), 
                 rho_dot * rho_dot,
                 phi,
                 0.0;

    return cartesian;
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
}
