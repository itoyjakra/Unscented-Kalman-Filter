#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  previous_timestamp_ = 0;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  x_ = VectorXd(n_x_); // state vector
  P_ = MatrixXd(n_x_, n_x_); // state covariance matrix

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
    if (!is_initialized_)
    {
        VectorXd measured_pos = meas_package.raw_measurements_;
        x_ = VectorXd::Zero(n_x_);
        P_ = MatrixXd::Identity(n_x_, n_x_);

        switch (meas_package.sensor_type_)
        {
            case MeasurementPackage::LASER:
                x_(0) = measured_pos(0);
                x_(1) = measured_pos(1);
                break;
            case MeasurementPackage::RADAR:
                x_ = tools.PolarToCart(measured_pos);
                break;
            default:
                std::cout << "Wrong measurement type!!!" << std::endl;
                break;
        }
        is_initialized_ = true;
        previous_timestamp_ = meas_package.timestamp_;

        return;
    }

    double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	
    previous_timestamp_ = meas_package.timestamp_;

    Prediction(dt);
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        UpdateLidar(meas_package);
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
        UpdateRadar(meas_package);
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) 
{
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
    MatrixXd Xsig_out = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    GenerateSigmaPoints(&Xsig_out);
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}

void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) 
{
    //create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);


    x_aug = VectorXd::Zero(n_aug_);
    x_aug.head(n_x_) = x_;
    Xsig_aug.col(0) = x_aug;

    P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    MatrixXd Noise = MatrixXd(2, 2);
    Noise << std_a_ * std_a_, 0, 0, std_yawdd_ * std_yawdd_;
    P_aug.bottomRightCorner(2, 2) = Noise;

    MatrixXd A = P_aug.llt().matrixL();

    int f = n_aug_ + lambda_;
    for (int i=0; i<n_aug_; i++)
    {
        Xsig_aug.col(i+1) = x_aug + sqrt(f) * A.col(i);
        Xsig_aug.col(i+n_aug_+1) = x_aug - sqrt(f) * A.col(i);
    }

    //print result
    std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

    //write result
    *Xsig_out = Xsig_aug;
}
