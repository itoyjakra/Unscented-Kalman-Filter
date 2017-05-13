#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
const double SMALL = 1.0e-6;

UKF::UKF(ParameterPackage param_pack) {
    is_initialized_ = false;
    n_x_ = 5;
    n_aug_ = 7;
    n_z_radar_ = 3;
    n_z_laser_ = 2;
    lambda_ = 3 - n_aug_;
    previous_timestamp_ = 0;
    n_cols_sigma_ = 2 * n_aug_ + 1;

    weights_= VectorXd(n_cols_sigma_);
    weights_.fill(0.5/(lambda_ + n_aug_));
    weights_(0)= lambda_/(lambda_ + n_aug_);
    assert (fabs(weights_.sum() - 1.0) < 1.0e-6);

    Xsig_pred_ = MatrixXd::Zero(n_x_, n_cols_sigma_);
    z_radar_pred_ = VectorXd(n_z_radar_);
    S_radar_pred_ = MatrixXd(n_z_radar_, n_z_radar_);
    Zsig_radar_ = MatrixXd::Zero(n_z_radar_, n_cols_sigma_);
    z_laser_pred_ = VectorXd(n_z_laser_);
    S_laser_pred_ = MatrixXd(n_z_laser_, n_z_laser_);
    Zsig_laser_ = MatrixXd::Zero(n_z_laser_, n_cols_sigma_);

    use_laser_ = true;
    use_radar_ = true;

    x_ = VectorXd(n_x_);                // state vector
    P_ = MatrixXd(n_x_, n_x_);          // state covariance matrix

    x_pred_ = VectorXd(n_x_);           // predicted state vector
    P_pred_ = MatrixXd(n_x_, n_x_);     // predicted state covariance matrix

    // Process noise standard deviation
    std_a_ = param_pack.STD_A; //0.3;                       // longitudinal accereration
    std_yawdd_ = param_pack.STD_YAWDD; //0.3;                   // yaw acceleration

    // Laser measurement noise standard deviation
    std_laspx_ = 0.15;                  // position 1
    std_laspy_ = 0.15;                  // position 2

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;                    // radius
    std_radphi_ = 0.03;                 // angle 
    std_radrd_ = 0.3;                   // rate of change of radius

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
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

    bool condition1 = (meas_package.sensor_type_ == MeasurementPackage::LASER) & use_laser_;
    bool condition2 = (meas_package.sensor_type_ == MeasurementPackage::RADAR) & use_radar_;
    if (condition1 | condition2)
    {
        Prediction(dt, meas_package.sensor_type_);
        Update(meas_package);
    }
}

void UKF::Prediction(double delta_t, int sensor_type) 
{
    MatrixXd Xsig_out = MatrixXd(n_aug_, n_cols_sigma_);
    GenerateSigmaPoints(&Xsig_out);

    SigmaPointPrediction(Xsig_out, delta_t);

    VectorXd x_out = VectorXd(n_x_);
    MatrixXd P_out = MatrixXd(n_x_, n_x_);
    PredictMeanAndCovariance(&x_out, &P_out);
    x_pred_ = x_out;
    P_pred_ = P_out;

    if (sensor_type == MeasurementPackage::RADAR)
        PredictRadarMeasurement();
    else if (sensor_type == MeasurementPackage::LASER)
        PredictLidarMeasurement();
}

void UKF::Update(MeasurementPackage meas_package)
{
    VectorXd x_out = VectorXd(n_x_);
    MatrixXd P_out = MatrixXd(n_x_, n_x_);
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        UpdateState_Lidar(&x_out, &P_out, meas_package.raw_measurements_);
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
        UpdateState_Radar(&x_out, &P_out, meas_package.raw_measurements_);
    x_ = x_pred_ + x_out;
    P_ = P_pred_ - P_out;
}

void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) 
{
    VectorXd x_aug = VectorXd::Zero(n_aug_);
    MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
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

    *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(MatrixXd Xsig_aug, double delta_t)
{
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

        if (fabs(psi_dot) < SMALL)
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
    VectorXd x = VectorXd::Zero(n_x_);
    MatrixXd P = MatrixXd::Zero(n_x_, n_x_);

    x = Xsig_pred_ * weights_;

	for (int i=0; i<n_cols_sigma_; i++)
    {
        VectorXd x_diff = Xsig_pred_.col(i) - x;
        x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));
        P += weights_(i) * x_diff * x_diff.transpose();
    }

    *x_out = x;
    *P_out = P;

}

void UKF::PredictLidarMeasurement()
{
    MatrixXd R = MatrixXd(n_z_laser_, n_z_laser_);

    double px, py, v, psi, psi_dot, rho, phi, rho_dot;
    VectorXd z_col = VectorXd(n_z_laser_);
    for (int i=0; i<n_cols_sigma_; i++)
    {
        px = Xsig_pred_(0, i);
        py = Xsig_pred_(1, i);
        v = Xsig_pred_(2, i);
        psi = Xsig_pred_(3, i);
        psi_dot = Xsig_pred_(4, i);

        z_col << px, py;
        Zsig_laser_.col(i) = z_col;
    }
    z_laser_pred_ = Zsig_laser_ * weights_;

    S_laser_pred_ = MatrixXd::Zero(n_z_laser_, n_z_laser_);
    for (int i=0; i<n_cols_sigma_; i++)
    {
        VectorXd Z_diff = Zsig_laser_.col(i) - z_laser_pred_;
        S_laser_pred_ += weights_(i) * Z_diff * Z_diff.transpose();
    }

    R = MatrixXd::Zero(n_z_laser_, n_z_laser_);
    R(0, 0) = std_laspx_ * std_laspx_;
    R(1, 1) = std_laspy_ * std_laspy_;

    S_laser_pred_ += R;

}

void UKF::PredictRadarMeasurement()
{
    MatrixXd R = MatrixXd(n_z_radar_, n_z_radar_);

    double px, py, v, psi, psi_dot, rho, phi, rho_dot;
    VectorXd z_col = VectorXd(n_z_radar_);
    for (int i=0; i<n_cols_sigma_; i++)
    {
        px = Xsig_pred_(0, i);
        py = Xsig_pred_(1, i);
        v = Xsig_pred_(2, i);
        psi = Xsig_pred_(3, i);
        psi_dot = Xsig_pred_(4, i);

        rho = sqrt(px*px + py*py);
        phi = atan2(py, px);
        if (rho < SMALL)
            rho_dot = 0;
        else
            rho_dot = v * (px * cos(psi) + py * sin(psi)) / rho;

        z_col << rho, phi, rho_dot;
        Zsig_radar_.col(i) = z_col;
    }
    z_radar_pred_ = Zsig_radar_ * weights_;

    S_radar_pred_ = MatrixXd::Zero(n_z_radar_, n_z_radar_);
    for (int i=0; i<n_cols_sigma_; i++)
    {
        VectorXd Z_diff = Zsig_radar_.col(i) - z_radar_pred_;
        Z_diff(1) = atan2(sin(Z_diff(1)), cos(Z_diff(1)));
        S_radar_pred_ += weights_(i) * Z_diff * Z_diff.transpose();
    }

    R = MatrixXd::Zero(n_z_radar_, n_z_radar_);
    R(0, 0) = std_radr_ * std_radr_;
    R(1, 1) = std_radphi_ * std_radphi_;
    R(2, 2) = std_radrd_ * std_radrd_;

    S_radar_pred_ += R;

}

void UKF::UpdateState_Lidar(VectorXd* x_out, MatrixXd* P_out, VectorXd z)
{
    VectorXd x = VectorXd(n_x_);
    MatrixXd P = MatrixXd(n_x_, n_x_);
    MatrixXd Tc = MatrixXd(n_x_, n_z_laser_);


    Tc = MatrixXd::Zero(n_x_, n_z_laser_);

	for (int i=0; i<n_cols_sigma_; i++)
    {
        VectorXd z_diff = Zsig_laser_.col(i) - z_laser_pred_;

        VectorXd x_diff = Xsig_pred_.col(i) - x_pred_;
        x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));

        Tc += weights_(i) * x_diff * z_diff.transpose();
    }

    MatrixXd K = MatrixXd(n_x_, n_z_laser_);
    K = Tc * S_laser_pred_.inverse();

    VectorXd z_diff = z - z_laser_pred_;
    x = K * z_diff;
    P = K * S_laser_pred_ * K.transpose();
    NIS_laser_ = z_diff.transpose() * S_laser_pred_.inverse() * z_diff;

    x(3) = atan2(sin(x(3)), cos(x(3)));
    assert (fabs(x(3) < M_PI));

    *x_out = x;
    *P_out = P;
}

void UKF::UpdateState_Radar(VectorXd* x_out, MatrixXd* P_out, VectorXd z)
{
    VectorXd x = VectorXd(n_x_);
    MatrixXd P = MatrixXd(n_x_, n_x_);
    MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);


    Tc = MatrixXd::Zero(n_x_, n_z_radar_);

	for (int i=0; i<n_cols_sigma_; i++)
    {
        VectorXd z_diff = Zsig_radar_.col(i) - z_radar_pred_;
        z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));

        VectorXd x_diff = Xsig_pred_.col(i) - x_pred_;
        x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));

        Tc += weights_(i) * x_diff * z_diff.transpose();
    }

    MatrixXd K = MatrixXd(n_x_, n_z_radar_);
    K = Tc * S_radar_pred_.inverse();

    VectorXd z_diff = z - z_radar_pred_;
    z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));

    x = K * z_diff;
    P = K * S_radar_pred_ * K.transpose();
    NIS_radar_ = z_diff.transpose() * S_radar_pred_.inverse() * z_diff;

    x(3) = atan2(sin(x(3)), cos(x(3)));
    assert (fabs(x(3) < M_PI));

    *x_out = x;
    *P_out = P;
}
