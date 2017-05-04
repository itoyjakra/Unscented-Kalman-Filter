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
    n_z_ = 3;
    lambda_ = 3 - n_aug_;
    previous_timestamp_ = 0;

    weights_= VectorXd(2 * n_aug_ + 1);
    weights_.fill(0.5/(lambda_ + n_aug_));
    weights_(0)= lambda_/(lambda_ + n_aug_);
    assert (fabs(weights_.sum() - 1.0) < 1.0e-6);

    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  x_ = VectorXd(n_x_); // state vector
  P_ = MatrixXd(n_x_, n_x_); // state covariance matrix

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.1; //30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.1; //30;

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
    std::cout << "step 1" << std::endl;
    std::cout << Xsig_out.rows() << std::endl;
    std::cout << Xsig_out.cols() << std::endl;
    //MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    SigmaPointPrediction(Xsig_out, delta_t);
    std::cout << "step 2" << std::endl;

    VectorXd x_out = VectorXd(n_x_);
    MatrixXd P_out = MatrixXd(n_x_, n_x_);
    PredictMeanAndCovariance(&x_out, &P_out);
    std::cout << "step 3" << std::endl;

    VectorXd z_out = VectorXd(n_z_);
    MatrixXd S_out = MatrixXd(n_z_, n_z_);
    PredictRadarMeasurement(&z_out, &S_out);
    std::cout << "step 4" << std::endl;
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

void UKF::SigmaPointPrediction(MatrixXd Xsig_aug, double delta_t)
{
    double small = 1.0e-6;

    double half_dtsq = 0.5 * delta_t * delta_t;

    double px;
    double py;
    double v;
    double psi;
    double psi_dot;
    double nu_a;
    double nu_psi_dot2;

    VectorXd detrm_col = VectorXd(n_x_);         // deterministic part
    VectorXd noise_col = VectorXd(n_x_);         // noise part

    for (int i=0; i<2*n_aug_ + 1; i++)
    {
        VectorXd col = Xsig_aug.col(i);
        px = col(0);
        py = col(1);
        v = col(2);
        psi = col(3);
        psi_dot = col(4);
        nu_a = col(5);
        nu_psi_dot2 = col(6);

        if (col(n_x_ - 1) < small)
        {
            detrm_col(0) = v * cos(psi) * delta_t;
            detrm_col(1) = v * sin(psi) * delta_t;
            detrm_col(2) = 0;
            detrm_col(3) = 0;
            detrm_col(4) = 0;
        }
        else
        {
            detrm_col(0) = v * (sin(psi + psi_dot * delta_t) - sin(psi)) / psi_dot;
            detrm_col(1) = v * (- cos(psi + psi_dot * delta_t) + cos(psi)) / psi_dot;
            detrm_col(2) = 0;
            detrm_col(3) = psi_dot * delta_t;
            detrm_col(4) = 0;
        }
        noise_col << half_dtsq * cos(psi) * nu_a,
                    half_dtsq * sin(psi) * nu_a,
                    delta_t * nu_a,
                    half_dtsq * nu_psi_dot2,
                    delta_t * nu_psi_dot2;

        Xsig_pred_.col(i) = col.head(n_x_) + detrm_col + noise_col;
    }
}

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out)
{
    VectorXd x = VectorXd(n_x_);
    MatrixXd P = MatrixXd(n_x_, n_x_);
    MatrixXd Xsig_temp1 = MatrixXd(Xsig_pred_.rows(), Xsig_pred_.cols());
    MatrixXd Xsig_temp2 = MatrixXd(Xsig_pred_.rows(), Xsig_pred_.cols());


    x = Xsig_pred_ * weights_;

    Xsig_temp1 = Xsig_pred_;
    Xsig_temp1.colwise() -= x; //Xsig_temp1.rowwise().mean();
    Xsig_temp2 = Xsig_temp1;
    for (int i=0; i<n_x_; i++)
        for (int j=0; j<2*n_aug_+1; j++)
            Xsig_temp2(i, j) *= weights_(j);
    P = Xsig_temp2 * Xsig_temp1.transpose();

    std::cout << "Predicted state" << std::endl;
    std::cout << x << std::endl;
    std::cout << "Predicted covariance matrix" << std::endl;
    std::cout << P << std::endl;

    *x_out = x;
    *P_out = P;
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out)
{
    double small = 1.0e-6;
    MatrixXd Zsig = MatrixXd(n_x_, 2*n_aug_+1);
    VectorXd z_pred = VectorXd(n_z_);
    MatrixXd S = MatrixXd(n_z_, n_z_);

    MatrixXd R = MatrixXd(n_z_, n_z_);

    double px, py, v, psi, psi_dot, rho, phi, rho_dot;
    MatrixXd z_temp = MatrixXd(n_z_, 2*n_aug_+1);
    VectorXd z_col = VectorXd(n_z_);
    for (int i=0; i<2*n_aug_+1; i++)
    {
        px = Xsig_pred_(0, i);
        py = Xsig_pred_(1, i);
        v = Xsig_pred_(2, i);
        psi = Xsig_pred_(3, i);
        psi_dot = Xsig_pred_(4, i);

        rho = sqrt(px*px + py*py);
        phi = atan2(py, px);
        if (rho < small)
            rho_dot = 0;
        else
            rho_dot = v * (px * cos(psi) + py * sin(psi)) / rho;

        z_col << rho, phi, rho_dot;
        z_temp.col(i) = z_col;
    }
    z_pred = z_temp * weights_;

    S = MatrixXd::Zero(n_z_, n_z_);
    for (int i=0; i<2*n_aug_+1; i++)
    {
        S += weights_(i) * (z_temp.col(i) - z_pred) * (z_temp.col(i) - z_pred).transpose();
    }

    R = MatrixXd::Zero(n_z_, n_z_);
    R(0, 0) = std_radr_ * std_radr_;
    R(1, 1) = std_radphi_ * std_radphi_;
    R(2, 2) = std_radrd_ * std_radrd_;

    S += R;

  //print result
  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;

  //write result
  *z_out = z_pred;
  *S_out = S;
}

void UKF::UpdateState(VectorXd* x_out, MatrixXd* P_out)
{
    double small = 1.0e-6;

    VectorXd x = VectorXd(n_x_);
    MatrixXd P = MatrixXd(n_x_, n_x_);
    MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
    VectorXd z_pred = VectorXd(n_z_);
    MatrixXd S = MatrixXd(n_z_, n_z_);
    VectorXd z = VectorXd(n_z_);
    MatrixXd Tc = MatrixXd(n_x_, n_z_);


    Tc = MatrixXd::Zero(n_x_, n_z_);

	for (int i=0; i<2*n_aug_+1; i++)
        Tc += weights_(i) * (Xsig_pred_.col(i) - x) * (Zsig.col(i) - z_pred).transpose();

    std::cout << "---Tc---" << std::endl;
    std::cout << Tc << std::endl;

    MatrixXd K = MatrixXd(n_x_, n_z_);
    K = Tc * S.inverse();
    std::cout << "---K---" << std::endl;
    std::cout << K << std::endl;

    x += K * (z - z_pred);
    P -= K * S * K.transpose();


  //print result
  std::cout << "Updated state x: " << std::endl << x << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

  //write result
  *x_out = x;
  *P_out = P;


}
