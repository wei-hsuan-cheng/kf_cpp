#ifndef QUATERNION_ORIENTATION_UKF_HPP
#define QUATERNION_ORIENTATION_UKF_HPP

#include "Eigen/Dense"
#include "kf_cpp/ukf.hpp"  // Adjust the include path to your UKF header
#include "robot_math_utils/robot_math_utils_v1_7.hpp"
#include <cmath>

using RM = RMUtils;

/**
 * @brief StateSpaceModel implements the process and measurement models for orientation.
 * 
 * The state is 7-dimensional:
 *   - first 4 elements: quaternion (w, x, y, z)
 *   - last 3 elements: angular velocity (rad/s)
 * 
 * The process model integrates the quaternion using the angular velocity.
 * The measurement model is the identity mapping.
 * 
 * This class now also contains the parameters:
 *   - state_dim (n)
 *   - measurement_dim (p)
 *   - Gamma (for scaling, here set equal to dt)
 */
class StateSpaceModel {
public:
  double dt; // Time step
  int state_dim;
  int measurement_dim;
  double Gamma; // For example, set equal to dt (or modify as needed)

  explicit StateSpaceModel(double dt_val)
    : dt(dt_val),
      state_dim(7),
      measurement_dim(7),
      Gamma(dt_val) // Here we choose Gamma = dt; modify as needed.
  {}

  // Compute a 4x4 quaternion transition matrix based on angular velocity.
  Eigen::MatrixXd gx(const Eigen::VectorXd &x) const {
    // x: state vector (7x1), where q = x.head(4), omega = x.tail(3)
    Eigen::VectorXd omega = x.tail(3);
    double omega_norm = RM::Norm(omega);
    if (omega_norm == 0)
      return Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd I4 = Eigen::MatrixXd::Identity(4, 4);
    double half_dt = dt / 2.0;
    // RM::Sinc(x) returns sin(x)/x.
    return cos(omega_norm * half_dt) * I4 +
           (half_dt) * RM::Sinc(omega_norm * half_dt) * Omegax(x);
  }

  // Helper: compute a 4x4 matrix from the angular velocity.
  Eigen::MatrixXd Omegax(const Eigen::VectorXd &x) const {
    Eigen::VectorXd omega = x.tail(3);
    Eigen::MatrixXd Omega(4, 4);
    Omega(0, 0) = 0.0;
    Omega.block(0, 1, 1, 3) = -omega.transpose();
    Omega.block(1, 0, 3, 1) = omega;
    Omega.block(1, 1, 3, 3) = RM::R3Vec2so3Mat(omega);
    return Omega;
  }

  // Process model: integrates the quaternion and keeps angular velocity constant.
  Eigen::VectorXd f(const Eigen::VectorXd &x) const {
    Eigen::VectorXd q = x.head(4);
    Eigen::VectorXd q_new = gx(x) * q;
    q_new = RM::Normalized(q_new);
    Eigen::VectorXd x_new(7);
    x_new << q_new, x.tail(3);
    return x_new;
  }

  // Overloaded f(x, control) if needed (control is ignored here)
  Eigen::VectorXd f(const Eigen::VectorXd &x, const Eigen::VectorXd &/*control*/) const {
    return f(x);
  }

  // Measurement model: identity mapping.
  Eigen::VectorXd h(const Eigen::VectorXd &x) const {
    return x;
  }
};

// Create an alias for the UKF using the StateSpaceModel.
using UKFAlgorithm = UnscentedKalmanFilter<double, StateSpaceModel>;

/**
 * @brief NoiseModel encapsulates the noise covariance components.
 */
class NoiseModel {
public:
  Eigen::MatrixXd PNstd;                // Process noise base covariance (state dimension)
  Eigen::MatrixXd MNstd;                // Measurement noise base covariance (measurement dimension)
  Eigen::MatrixXd process_noise_cov;    // Process noise covariance Q
  Eigen::MatrixXd measurement_noise_cov;// Measurement noise covariance R

  /**
   * @brief Computes the covariance matrices based on the given state-space model
   *        and tuning factors.
   * @param ssm       The state-space model (providing dimensions and Gamma).
   * @param Q_tunfac  Scaling factor for process noise.
   * @param R_tunfac  Scaling factor for measurement noise.
   */
  void get_noise_model(const StateSpaceModel &ssm, double Q_tunfac, double R_tunfac) {
    // Process noise covariance components.
    Eigen::MatrixXd PNstd_quat = Eigen::MatrixXd::Identity(4, 4) * std::pow(10, -4.0);
    Eigen::MatrixXd PNstd_omega = Eigen::MatrixXd::Identity(3, 3) * std::pow(10, -3.0);
    PNstd = Eigen::MatrixXd(ssm.state_dim, ssm.state_dim);
    PNstd << PNstd_quat,         Eigen::MatrixXd::Zero(4, 3),
             Eigen::MatrixXd::Zero(3, 4), PNstd_omega;
    process_noise_cov = ssm.Gamma * PNstd * PNstd.transpose() * ssm.Gamma * std::pow(Q_tunfac, 2);

    // Measurement noise covariance components.
    Eigen::MatrixXd MNstd_quat = Eigen::MatrixXd::Zero(4, 4);
    MNstd_quat(0, 0) = 0.0035652629269028323 * std::pow(10, 1.25);
    MNstd_quat(1, 1) = 0.001124141775313557  * std::pow(10, 1.25);
    MNstd_quat(2, 2) = 0.0032617761132252075 * std::pow(10, 1.25);
    MNstd_quat(3, 3) = 0.0023408977903160285 * std::pow(10, 1.25);
    Eigen::MatrixXd MNstd_omega = Eigen::MatrixXd::Zero(3, 3);
    MNstd_omega(0, 0) = 0.003612832752606975 * std::pow(10, 1.0);
    MNstd_omega(1, 1) = 0.001971638070939487 * std::pow(10, 1.0);
    MNstd_omega(2, 2) = 0.002420621344508792 * std::pow(10, 1.0);
    MNstd = Eigen::MatrixXd(ssm.measurement_dim, ssm.measurement_dim);
    MNstd << MNstd_quat,         Eigen::MatrixXd::Zero(4, 3),
             Eigen::MatrixXd::Zero(3, 4), MNstd_omega;
    measurement_noise_cov = MNstd * MNstd.transpose() * std::pow(R_tunfac, 2);
  }
};

/**
 * @brief Initialization encapsulates the initial state and covariance.
 */
class Initialization {
public:
  Eigen::VectorXd mean; // initial state estimate x̂₀
  Eigen::MatrixXd cov;  // initial covariance P₀

  /**
   * @brief Computes the initial state and covariance.
   * @param ssm The state-space model (providing dimensions).
   */
  void get_initial_mean_and_cov(const StateSpaceModel &ssm) {
    // Initial state: identity rotation and zero angular velocity.
    Quaterniond q_init = RM::Rot2Quat(Eigen::Matrix3d::Identity());
    Vector3d omega_init = Vector3d::Zero();
    mean.resize(ssm.state_dim);
    // Expand omega_init components explicitly.
    mean << q_init.w(), q_init.x(), q_init.y(), q_init.z(),
            omega_init(0), omega_init(1), omega_init(2);
    cov = Eigen::MatrixXd::Identity(ssm.state_dim, ssm.state_dim) * 0.01;
  }
};

/**
 * @brief UKFStateSpaceModelAndParams encapsulates the state-space model,
 *        the noise covariance matrices, and the initial state.
 *
 * Its public attributes are:
 *  - ssm: an instance of StateSpaceModel
 *  - nm: an instance of NoiseModel (Q and R)
 *  - init: an instance of Initialization (initial mean and covariance)
 */
class UKFStateSpaceModelAndParams {
public:
  StateSpaceModel ssm;
  NoiseModel nm;
  Initialization init;

  /**
   * @brief Constructor.
   * @param Ts       Sampling period (time step)
   * @param Q_tunfac Scaling factor for process noise.
   * @param R_tunfac Scaling factor for measurement noise.
   */
  UKFStateSpaceModelAndParams(double Ts, double Q_tunfac, double R_tunfac)
    : ssm(Ts)
  {
    nm.get_noise_model(ssm, Q_tunfac, R_tunfac);
    init.get_initial_mean_and_cov(ssm);
  }
};

#endif // QUATERNION_ORIENTATION_UKF_HPP