#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "Eigen/Dense"
#include "kf_cpp/quaternion_orientation_ukf.hpp" // Include our header
#include "robot_math_utils/robot_math_utils_v1_7.hpp"

using std::placeholders::_1;
using RM = RMUtils;

class UKFOrientationSimulation : public rclcpp::Node {
public:
  UKFOrientationSimulation() : Node("ukf_ori_sim"),
                         k_(0),
                         fs_(60.0),
                         Ts_(1.0 / fs_),
                         t_(0.0)
  {
    // Publishers for ground-truth, measurement, and estimated state.
    quat_gt_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("quat_gt", 10);
    quat_meas_pub_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("quat_meas", 10);
    quat_est_pub_= this->create_publisher<std_msgs::msg::Float64MultiArray>("quat_est", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Create an instance of the state-space model with desired tuning factors.
    double Q_tunfac = std::pow(10, 1.0);
    double R_tunfac = std::pow(10, -1.0);
    ukf_ssm_and_params_ = std::make_shared<UKFStateSpaceModelAndParams>(Ts_, Q_tunfac, R_tunfac);

    // Create the UKF instance using the attributes from ukf_ssm_and_params_.
    ukf_ = std::make_shared<UKFAlgorithm>(
              ukf_ssm_and_params_->ssm,
              ukf_ssm_and_params_->ssm.state_dim,
              0,
              ukf_ssm_and_params_->ssm.measurement_dim,
              ukf_ssm_and_params_->nm.process_noise_cov,
              ukf_ssm_and_params_->nm.measurement_noise_cov,
              ukf_ssm_and_params_->init.mean,
              ukf_ssm_and_params_->init.cov
           );

    // Initialize ground-truth state.
    // q_init: initial quaternion from identity rotation.
    Quaterniond q_init_ = RM::Rot2Quat(Matrix3d::Identity());
    // omega_init_ = Vector3d::Ones().normalized() * 45.0 * M_PI / 180.0;
    Vector3d omega_init_ = Vector3d::Ones().normalized() * 5.0 * M_PI / 180.0;
    state_gt_ << q_init_.w(), q_init_.x(), q_init_.y(), q_init_.z(), omega_init_;
  }

  /**
   * @brief Update the ground-truth orientation.
   * 
   * This function simulates the evolution of the ground-truth state.
   * It integrates the quaternion using the angular velocity.
   */
  Vector7d gt_motion(Vector7d x_gt) {
    Vector4d quat = x_gt.head(4);
    Vector3d omega = x_gt.tail(3);
    double omega_norm = RM::Norm(omega);
    if (omega_norm > 0) {
      // Compute the delta quaternion from the angular velocity.
      Matrix4d delta_q = cos(omega_norm * Ts_ / 2.0) * Matrix4d::Identity()
                         + (1.0 / omega_norm) * sin(omega_norm * Ts_ / 2.0) * 
                           orientation_Omegax(omega);
      quat = delta_q * quat;
      quat = RM::Normalized(quat);
    }
    x_gt.head(4) = quat;

    // Add process noise to state
    for (int i = 0; i < x_gt.size(); i++) {
        x_gt(i) += ukf_ssm_and_params_->ssm.Gamma * ukf_ssm_and_params_->nm.PNstd(i, i) * RM::RandNorDist(); // Adding noise (ssm.Gamma * w)
    }
    
    return x_gt;
  }

  // Helper: compute the 4x4 Omega matrix for the current state.
  Matrix4d orientation_Omegax(const Vector3d &omega) {
    Matrix4d Omegax;
    Omegax(0,0) = 0.0;
    Omegax.block(0,1,1,3) = -omega.transpose();
    Omegax.block(1,0,3,1) = omega;
    Omegax.block(1,1,3,3) = RM::R3Vec2so3Mat(omega);
    return Omegax;
  }

  void publish_tf(const VectorXd& q, const std::string& frame_id, const std::string& parent_frame_id) {
    geometry_msgs::msg::TransformStamped tf_stamp;
    tf_stamp.header.stamp = this->get_clock()->now();
    tf_stamp.header.frame_id = parent_frame_id;
    tf_stamp.child_frame_id = frame_id;
    tf_stamp.transform.translation.x = 0.0;
    tf_stamp.transform.translation.y = 0.0;
    tf_stamp.transform.translation.z = 0.0;
    tf_stamp.transform.rotation.w = q(0);
    tf_stamp.transform.rotation.x = q(1);
    tf_stamp.transform.rotation.y = q(2);
    tf_stamp.transform.rotation.z = q(3);
    tf_broadcaster_->sendTransform(tf_stamp);
  }

  void run() {
    rclcpp::Rate rate(fs_);
    while (rclcpp::ok()) {
      iteration();
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }
  }


private:
  void iteration() {
    // Simulate the ground-truth state.
    t_ = k_ * Ts_;
    state_gt_ = gt_motion(state_gt_);

    // Publish ground-truth.
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(state_gt_.data(), state_gt_.data() + state_gt_.size());
    quat_gt_pub_->publish(msg);

    // Simulate the measurement (ground truth output plus noise).
    VectorXd meas = ukf_ssm_and_params_->ssm.h(state_gt_);
    VectorXd measurement_noise = VectorXd::Zero(meas.size());
    for (int i = 0; i < meas.size(); i++) {
      meas(i) += ukf_ssm_and_params_->nm.MNstd(i, i) * RM::RandNorDist(); // Adding noise
    }
    msg.data.assign(meas.data(), meas.data() + meas.size());
    quat_meas_pub_->publish(msg);

    // UKF prediction & correction.
    ukf_->predict();
    ukf_->correct(meas);
    VectorXd state_est = ukf_->mean;
    state_est.head(4) = state_est.head(4).normalized();
    ukf_->setMean(state_est);

    // Publish estimated state.
    msg.data.assign(state_est.data(), state_est.data() + state_est.size());
    quat_est_pub_->publish(msg);

    // Optionally, publish TF frames for visualization.
    publish_tf(state_gt_.head(4), "gt", "map");
    publish_tf(meas.head(4), "meas", "map");
    publish_tf(state_est.head(4), "est", "map");

    k_++;
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr quat_gt_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr quat_meas_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr quat_est_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<UKFAlgorithm> ukf_;
  std::shared_ptr<UKFStateSpaceModelAndParams> ukf_ssm_and_params_;
  Vector7d state_gt_;

  double fs_, Ts_, t_;
  int k_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UKFOrientationSimulation>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
