#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

UKF::UKF() {
    is_initialized_ = false;
    n_x_ = 5;
    n_aug_ = 7;
    n_z_ = 3;
    lambda_ = 3 - n_aug_;
    previous_timestamp_ = 0;
    n_cols_sigma_ = 2 * n_aug_ + 1;

    weights_= VectorXd(n_cols_sigma_);
    weights_.fill(0.5/(lambda_ + n_aug_));
    weights_(0)= lambda_/(lambda_ + n_aug_);
    assert (fabs(weights_.sum() - 1.0) < 1.0e-6);

    Xsig_pred_ = MatrixXd::Zero(n_x_, n_cols_sigma_);
    z_pred_ = VectorXd(n_z_);
    S_pred_ = MatrixXd(n_z_, n_z_);
    Zsig_ = MatrixXd::Zero(n_z_, n_cols_sigma_);

    use_laser_ = false;
    use_radar_ = true;

    x_ = VectorXd(n_x_);        // state vector
    P_ = MatrixXd(n_x_, n_x_);  // state covariance matrix

    x_pred_ = VectorXd(n_x_);        // predicted state vector
    P_pred_ = MatrixXd(n_x_, n_x_);  // predicted state covariance matrix

    // Process noise standard deviation
    std_a_ = 0.1;               // longitudinal accereration
    std_yawdd_ = 0.1;           // yaw acceleration

    // Laser measurement noise standard deviation
    std_laspx_ = 0.15;          // position 1
    std_laspy_ = 0.15;          // position 2

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;            // radius
    std_radphi_ = 0.03;         // angle 
    std_radrd_ = 0.3;           // rate of change of radius

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
    if ((meas_package.sensor_type_ == MeasurementPackage::LASER) & use_laser_)
        UpdateLidar(meas_package);
    else if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) & use_radar_)
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
    MatrixXd Xsig_out = MatrixXd(n_aug_, n_cols_sigma_);
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
    x_pred_ = x_out;
    P_pred_ = P_out;

    std::cout << "step 3" << std::endl;

    VectorXd z_out = VectorXd(n_z_);
    MatrixXd S_out = MatrixXd(n_z_, n_z_);
    PredictRadarMeasurement();
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
void UKF::UpdateRadar(MeasurementPackage meas_package) 
{
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
    VectorXd x_out = VectorXd(n_x_);
    MatrixXd P_out = MatrixXd(n_x_, n_x_);
    UpdateState(&x_out, &P_out, meas_package.raw_measurements_);
    x_ = x_out;
    P_ = P_out;
}

void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) 
{
    //create augmented mean vector
    VectorXd x_aug = VectorXd::Zero(n_aug_);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, n_cols_sigma_);

    x_aug.head(n_x_) = x_;
    Xsig_aug.col(0) = x_aug;

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

    for (int i=0; i<n_cols_sigma_; i++)
    {
        VectorXd col = Xsig_aug.col(i);
        px = col(0);
        py = col(1);
        v = col(2);
        psi = col(3);
        psi_dot = col(4);
        nu_a = col(5);
        nu_psi_dot2 = col(6);

        if (fabs(psi_dot) < small)
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
    std::cout << "IN SigmaPointPrediction..." << std::endl;
    std::cout << Xsig_pred_ << std::endl;

}

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out)
{
    VectorXd x = VectorXd::Zero(n_x_);
    MatrixXd P = MatrixXd::Zero(n_x_, n_x_);

    x = Xsig_pred_ * weights_;

	for (int i=0; i<n_cols_sigma_; i++)
    {
        VectorXd x_diff = Xsig_pred_.col(i) - x;
        while (x_diff(3) > M_PI) x_diff(3) -= 2 * M_PI;
        while (x_diff(3) < M_PI) x_diff(3) += 2 * M_PI;

        P += weights_(i) * x_diff * x_diff.transpose();
    }

    *x_out = x;
    *P_out = P;

    std::cout << "In PredictMeanAndCovariance..." << std::endl;
    std::cout << "Predicted state" << std::endl;
    std::cout << x << std::endl;
    std::cout << "Predicted covariance matrix" << std::endl;
    std::cout << P << std::endl;

}

void UKF::PredictRadarMeasurement()
{
    double small = 1.0e-6;

    MatrixXd R = MatrixXd(n_z_, n_z_);

    double px, py, v, psi, psi_dot, rho, phi, rho_dot;
    VectorXd z_col = VectorXd(n_z_);
    for (int i=0; i<n_cols_sigma_; i++)
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
        Zsig_.col(i) = z_col;
    }
    z_pred_ = Zsig_ * weights_;

    S_pred_ = MatrixXd::Zero(n_z_, n_z_);
    for (int i=0; i<n_cols_sigma_; i++)
    {
        VectorXd Z_diff = Zsig_.col(i) - z_pred_;
        while (Z_diff(1) > M_PI) Z_diff(1) -= 2 * M_PI;
        while (Z_diff(1) < M_PI) Z_diff(1) += 2 * M_PI;
        S_pred_ += weights_(i) * Z_diff * Z_diff.transpose();
        //S_pred_ += weights_(i) * (Zsig_.col(i) - z_pred_) * (Zsig_.col(i) - z_pred_).transpose();
    }

    R = MatrixXd::Zero(n_z_, n_z_);
    R(0, 0) = std_radr_ * std_radr_;
    R(1, 1) = std_radphi_ * std_radphi_;
    R(2, 2) = std_radrd_ * std_radrd_;

    S_pred_ += R;

  //print result
    std::cout << "In PredictRadarMeasurement..." << std::endl;
  std::cout << "z_pred: " << std::endl << z_pred_ << std::endl;
  std::cout << "S: " << std::endl << S_pred_ << std::endl;

}

void UKF::UpdateState(VectorXd* x_out, MatrixXd* P_out, VectorXd z)
{
    double small = 1.0e-6;

    VectorXd x = VectorXd(n_x_);
    MatrixXd P = MatrixXd(n_x_, n_x_);
    MatrixXd Tc = MatrixXd(n_x_, n_z_);


    Tc = MatrixXd::Zero(n_x_, n_z_);

	for (int i=0; i<n_cols_sigma_; i++)
        Tc += weights_(i) * (Xsig_pred_.col(i) - x_) * (Zsig_.col(i) - z_pred_).transpose();

    std::cout << "---Tc---" << std::endl;
    std::cout << Tc << std::endl;

    MatrixXd K = MatrixXd(n_x_, n_z_);
    K = Tc * S_pred_.inverse();
    std::cout << "---K---" << std::endl;
    std::cout << K << std::endl;

    //x += K * (z - z_pred_);
    //P -= K * S_pred_ * K.transpose();

    x = x_pred_ + K * (z - z_pred_);
    P = P_pred_ - K * S_pred_ * K.transpose();

  //print result
  std::cout << "Updated state x: " << std::endl << x << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

  //write result
  *x_out = x;
  *P_out = P;
}
// TODO force psi within (-pi, pi) range
