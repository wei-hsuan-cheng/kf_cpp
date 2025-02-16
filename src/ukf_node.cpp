// File: src/ukf_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <iostream>
#include "kf_cpp/ukf.hpp"  // Make sure the include path is correct

// Include the TranslationSystem defined above (or include its header if you moved it)
class TranslationSystem {
public:
  double dt;
  TranslationSystem(double dt_val = 1.0 / 60.0) : dt(dt_val) {}

  Eigen::Matrix<double, 9, 1> f(const Eigen::Matrix<double, 9, 1>& x) const {
    Eigen::Matrix<double, 9, 1> x_new;
    x_new(0) = x(0) + x(3)*dt + 0.5 * x(6) * dt * dt;
    x_new(1) = x(1) + x(4)*dt + 0.5 * x(7) * dt * dt;
    x_new(2) = x(2) + x(5)*dt + 0.5 * x(8) * dt * dt;
    x_new(3) = x(3) + x(6)*dt;
    x_new(4) = x(4) + x(7)*dt;
    x_new(5) = x(5) + x(8)*dt;
    x_new(6) = x(6);
    x_new(7) = x(7);
    x_new(8) = x(8);
    return x_new;
  }

  Eigen::Matrix<double, 9, 1> f(const Eigen::Matrix<double, 9, 1>& x,
                                const Eigen::Matrix<double, Eigen::Dynamic, 1>& /*control*/) const {
    return f(x);
  }

  Eigen::Matrix<double, 3, 1> h(const Eigen::Matrix<double, 9, 1>& x) const {
    return x.template head<3>();
  }
};

using UKF = UnscentedKalmanFilter<double, TranslationSystem>;

class UKFNode : public rclcpp::Node {
public:
  UKFNode() : Node("ukf_translation") {
    // Publishers for ground truth, measurement, and estimated positions.
    ground_truth_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("ground_truth_position", 10);
    measurement_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("measurement_position", 10);
    ukf_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("ukf_position", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Parameters
    fs_ = 60.0;              // Sampling rate in Hz
    Ts_ = 1.0 / fs_;         // Sampling period in seconds
    R_circle_ = 1.0;         // Radius of the circular trajectory
    f_circle_ = 0.1;         // Frequency in Hz
    omega_ = 2 * M_PI * f_circle_;

    normal_ = Eigen::Vector3d(0, 0, 1);
    normal_.normalize();
    center_ = Eigen::Vector3d(0, 0, 0);

    // Create rotation matrix (for now, identity if normal is (0,0,1))
    if (normal_.isApprox(Eigen::Vector3d(0, 0, 1))) {
      rot_matrix_ = Eigen::Matrix3d::Identity();
    } else {
      Eigen::Vector3d axis = normal_.cross(Eigen::Vector3d(0, 0, 1));
      double angle = std::acos(normal_.dot(Eigen::Vector3d(0, 0, 1)));
      rot_matrix_ = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    }

    k_ = 0;  // discrete time step

    // Dimensions for our UKF
    state_dim_ = 9;
    measurement_dim_ = 3;

    // Create the system object for the UKF and set the time step
    system_ = TranslationSystem(Ts_);

    // Process noise covariance: build similar to your EKF Gamma_ matrix approach
    // For simplicity, here we create a 9x9 process noise covariance.
    process_noise_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    double dt = Ts_;
    // Assuming white noise acceleration uncertainty for each axis
    double accel_noise_std = 1.0;  // adjust as needed
    // You can compute process noise following the constant acceleration model
    // For example, using the continuous white noise acceleration model:
    // Q = [dt^4/4 dt^3/2; dt^3/2 dt^2] multiplied per axis.
    for (int i = 0; i < 3; i++) {
      process_noise_(i, i) = std::pow(dt, 4) / 4.0 * accel_noise_std * accel_noise_std;
      process_noise_(i, i+3) = std::pow(dt, 3) / 2.0 * accel_noise_std * accel_noise_std;
      process_noise_(i+3, i) = std::pow(dt, 3) / 2.0 * accel_noise_std * accel_noise_std;
      process_noise_(i+3, i+3) = std::pow(dt, 2) * accel_noise_std * accel_noise_std;
    }
    // Assume little noise on the acceleration states directly:
    for (int i = 6; i < 9; i++) {
      process_noise_(i, i) = 1e-3;
    }

    // Measurement noise covariance
    measurement_noise_ = Eigen::MatrixXd::Identity(measurement_dim_, measurement_dim_);
    double sigma_measurement = 50.0 * 1.0/1000.0;  // using the mm2m conversion from your EKF sample
    measurement_noise_ *= sigma_measurement * sigma_measurement;

    // Initial mean and covariance
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(state_dim_);
    // Initialize position using the trajectory at time t=0
    Eigen::Vector3d initial_point = compute_trajectory_point(0.0);
    mean.head(3) = initial_point;
    // velocities and accelerations can start at zero

    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 0.01;

    // Create the UKF instance.
    // Note: The UnscentedKalmanFilter constructor arguments are:
    // (system, state_dim, input_dim, measurement_dim, process_noise, measurement_noise, mean, cov)
    // Since we do not use any control input, we set input_dim to 0.
    ukf_ = std::make_shared<UKF>(system_, state_dim_, 0, measurement_dim_, process_noise_, measurement_noise_, mean, cov);
  }

  void run() {
    rclcpp::Rate rate(fs_);
    while (rclcpp::ok()) {
      ukf_iteration();
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ground_truth_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr measurement_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ukf_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<UKF> ukf_;
  TranslationSystem system_;

  double fs_, Ts_, R_circle_, f_circle_, omega_;
  Eigen::Vector3d normal_, center_;
  Eigen::Matrix3d rot_matrix_;
  int k_;
  int state_dim_, measurement_dim_;
  Eigen::MatrixXd process_noise_;
  Eigen::MatrixXd measurement_noise_;

  // Compute the ground truth trajectory point (circle) at time t.
  Eigen::Vector3d compute_trajectory_point(double t) {
    double x = R_circle_ * std::cos(omega_ * t);
    double y = R_circle_ * std::sin(omega_ * t);
    double z = 0;
    Eigen::Vector3d point(x, y, z);
    return rot_matrix_ * point + center_;
  }

  // Publish a TF transform for visualization in RViz.
  void publish_tf(const Eigen::Vector3d &position,
                  const std::string &child_frame_id,
                  const std::string &parent_frame_id) {
    geometry_msgs::msg::TransformStamped tf_stamp;
    tf_stamp.header.stamp = this->get_clock()->now();
    tf_stamp.header.frame_id = parent_frame_id;
    tf_stamp.child_frame_id = child_frame_id;
    tf_stamp.transform.translation.x = position.x();
    tf_stamp.transform.translation.y = position.y();
    tf_stamp.transform.translation.z = position.z();
    tf_stamp.transform.rotation.w = 1.0;
    tf_stamp.transform.rotation.x = 0.0;
    tf_stamp.transform.rotation.y = 0.0;
    tf_stamp.transform.rotation.z = 0.0;
    tf_broadcaster_->sendTransform(tf_stamp);
  }

  // One iteration: compute ground truth, simulate a noisy measurement, run UKF predict and correct.
  void ukf_iteration() {
    // Ground truth at time t
    double t = k_ * Ts_;
    Eigen::Vector3d current_point = compute_trajectory_point(t);

    // Simulated measurement: add random noise
    Eigen::Vector3d measurement_noise = Eigen::Vector3d::Random() * std::sqrt(measurement_noise_(0, 0));
    Eigen::Vector3d measurement = current_point + measurement_noise;

    if (!measurement.allFinite()) {
      RCLCPP_WARN(this->get_logger(), "Measurement contains NaN or Inf, skipping iteration.");
      return;
    }

    // --- UKF Prediction ---
    // In the unscented filter, the predict step calculates sigma points and propagates them.
    // (If you had control inputs, you could call predict(control). Here, we call predict() without control.)
    ukf_->predict();

    // --- UKF Correction ---
    ukf_->correct(measurement);

    // Get the estimated state (position is in the first three components)
    Eigen::Vector3d ukf_estimated_position = ukf_->mean.head(3);

    // Publish ground truth
    geometry_msgs::msg::Point gt_msg;
    gt_msg.x = current_point.x();
    gt_msg.y = current_point.y();
    gt_msg.z = current_point.z();
    ground_truth_publisher_->publish(gt_msg);

    // Publish measurement
    geometry_msgs::msg::Point meas_msg;
    meas_msg.x = measurement.x();
    meas_msg.y = measurement.y();
    meas_msg.z = measurement.z();
    measurement_publisher_->publish(meas_msg);

    // Publish UKF estimated position
    geometry_msgs::msg::Point est_msg;
    est_msg.x = ukf_estimated_position.x();
    est_msg.y = ukf_estimated_position.y();
    est_msg.z = ukf_estimated_position.z();
    ukf_publisher_->publish(est_msg);

    // Publish TF frames for visualization
    publish_tf(current_point, "gt", "map");
    publish_tf(measurement, "meas", "map");
    publish_tf(ukf_estimated_position, "est", "map");

    k_++;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UKFNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
