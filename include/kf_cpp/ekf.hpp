#ifndef EKF_HPP
#define EKF_HPP

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

class ExtendedKalmanFilter
{
public:
    ExtendedKalmanFilter(int dim_x, int dim_z, int dim_u = 0);

    void predict(const Eigen::MatrixXd& FJacobian, const Eigen::VectorXd& Fx, const Eigen::VectorXd& u = Eigen::VectorXd::Zero(0));
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& HJacobian, const Eigen::VectorXd& Hx, const Eigen::MatrixXd& R = Eigen::MatrixXd::Zero(0, 0));

    // Getters and setters for accessing the state, covariance, F, H, and R matrices
    Eigen::VectorXd getState() const { return x_; }
    Eigen::MatrixXd getCovariance() const { return P_; }
    Eigen::MatrixXd getF() const { return F_; }
    Eigen::MatrixXd getH() const { return H_; }
    Eigen::MatrixXd getR() const { return R_; }
    void setState(const Eigen::VectorXd& state) { x_ = state; }
    void setF(const Eigen::MatrixXd& F) { F_ = F; }
    void setH(const Eigen::MatrixXd& H) { H_ = H; }
    void setR(const Eigen::MatrixXd& R) { R_ = R; }

    // Setter for process noise and measurement noise
    void setProcessNoise(const Eigen::MatrixXd& Q) { Q_ = Q; }
    void setMeasurementNoise(const Eigen::MatrixXd& R) { R_ = R; }

private:
    int dim_x_;
    int dim_z_;
    int dim_u_;

    Eigen::VectorXd x_;   // State vector
    Eigen::MatrixXd P_;   // Covariance matrix
    Eigen::MatrixXd F_;   // State transition matrix
    Eigen::MatrixXd G_;   // Control transition matrix
    Eigen::MatrixXd Q_;   // Process noise matrix
    Eigen::MatrixXd H_;   // Measurement function matrix
    Eigen::MatrixXd R_;   // Measurement noise matrix
    Eigen::MatrixXd K_;   // Kalman gain
    Eigen::VectorXd y_;   // Residual

    Eigen::MatrixXd I_;   // Identity matrix

    Eigen::VectorXd x_prior_;
    Eigen::MatrixXd P_prior_;
    Eigen::VectorXd x_post_;
    Eigen::MatrixXd P_post_;
};

ExtendedKalmanFilter::ExtendedKalmanFilter(int dim_x, int dim_z, int dim_u)
    : dim_x_(dim_x), dim_z_(dim_z), dim_u_(dim_u)
{
    x_ = Eigen::VectorXd::Zero(dim_x_);
    P_ = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
    F_ = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
    G_ = Eigen::MatrixXd::Zero(dim_x_, dim_u_);
    Q_ = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
    H_ = Eigen::MatrixXd::Zero(dim_z_, dim_x_);
    R_ = Eigen::MatrixXd::Identity(dim_z_, dim_z_);
    K_ = Eigen::MatrixXd::Zero(dim_x_, dim_z_);
    y_ = Eigen::VectorXd::Zero(dim_z_);

    I_ = Eigen::MatrixXd::Identity(dim_x_, dim_x_);

    x_prior_ = x_;
    P_prior_ = P_;
    x_post_ = x_;
    P_post_ = P_;
}

void ExtendedKalmanFilter::predict(const Eigen::MatrixXd& FJacobian, const Eigen::VectorXd& Fx, const Eigen::VectorXd& u)
{
    x_ = Fx;
    P_ = FJacobian * P_ * FJacobian.transpose() + Q_;

    x_prior_ = x_;
    P_prior_ = P_;
}

void ExtendedKalmanFilter::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& HJacobian, const Eigen::VectorXd& Hx, const Eigen::MatrixXd& R)
{
    Eigen::MatrixXd PHT = P_ * HJacobian.transpose();
    Eigen::MatrixXd S = HJacobian * PHT + R_;
    Eigen::MatrixXd SI = S.inverse();
    K_ = PHT * SI;

    y_ = z - Hx;
    x_ = x_ + K_ * y_;
    P_ = (I_ - K_ * HJacobian) * P_;

    x_post_ = x_;
    P_post_ = P_;
}

#endif // EKF_HPP
