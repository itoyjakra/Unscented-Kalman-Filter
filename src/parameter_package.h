#ifndef PARAMETER_PACKAGE_H_
#define PARAMETER_PACKAGE_H_

class ParameterPackage {
public:
  double STD_A;         // initial value for std. dev. for acceleration
  double STD_YAWDD;     // initial value for std. dev. for yaw acceleration
  double MULT_P1;       // initial value for the first two diagonal elements of process cov. matrix
  double MULT_P2;       // initial value for the last three diagonal elements of process cov. matrix
};

#endif 
