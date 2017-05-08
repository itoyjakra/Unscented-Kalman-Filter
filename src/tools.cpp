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
                              const vector<VectorXd> &ground_truth) 
{
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  VectorXd c;

  if (estimations.size() != ground_truth.size())
  {
    throw "lengths of vectors are different for RMSE calculations";
  }
  else if (estimations.size() == 0)
  {
    throw "estimate is unavailable";
  }
  else if (ground_truth.size() == 0)
  {
    throw "ground truth is unavailable";
  }
  else
  {
    for (int i=0; i<estimations.size(); i++)
    {
      c = estimations[i] - ground_truth[i];
      rmse.array() = rmse.array() + c.array() * c.array();
    }
  }
  rmse = rmse / estimations.size();
  rmse = sqrt(rmse.array());

  return rmse;
}

  /**
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  TODO:
    * Calculate the RMSE here.
  */
