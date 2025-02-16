#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <iostream>
#include "kf_cpp/ekf.hpp"

class EKFNode : public rclcpp::Node
{
public:
    EKFNode()
    : Node("ekf_translation")
    {
        // Publishers
        ground_truth_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("ground_truth_position", 10);
        measurement_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("measurement_position", 10);
        ekf_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("ekf_position", 10);

        // TF2 Broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Parameters
        fs_ = 60.0;  // Sampling rate in Hz
        Ts_ = 1.0 / fs_;  // Sampling period in seconds

        R_ = 1.0;  // Radius of the circle
        f_ = 0.1;  // Frequency in Hz
        omega_ = 2 * M_PI * f_;  // Angular velocity

        normal_ = Eigen::Vector3d(0, 0, 1);  // Set to (0, 0, 1) for testing
        normal_.normalize();
        center_ = Eigen::Vector3d(0, 0, 0);

        if (normal_.isApprox(Eigen::Vector3d(0, 0, 1))) {
            rot_matrix_ = Eigen::Matrix3d::Identity();
        } else {
            Eigen::Vector3d axis = normal_.cross(Eigen::Vector3d(0, 0, 1));
            double angle = std::acos(normal_.dot(Eigen::Vector3d(0, 0, 1)));
            rot_matrix_ = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        }

        k_ = 0;  // discrete time step

        // EKF initialization
        dim_x_ = 9;
        dim_z_ = 3;
        ekf_ = std::make_shared<ExtendedKalmanFilter>(dim_x_, dim_z_);

        // State-space model (constant acceleration)
        ekf_->setF(create_state_transition_matrix());
        ekf_->setH(create_measurement_matrix());

        // Measurement noise covariance
        sigma_ = 50.0 * mm2m_;
        ekf_->setMeasurementNoise(Eigen::MatrixXd::Identity(dim_z_, dim_z_) * sigma_ * sigma_);

        // Process noise covariance
        double q = 1.0;  // Adjust this value as needed
        Gamma_ = create_process_noise_covariance();
        ekf_->setProcessNoise(Gamma_ * q * Gamma_.transpose());
    }

    void run()
    {
        rclcpp::Rate rate(fs_);  // 60 Hz
        while (rclcpp::ok()) {
            ekf_iteration();
            rclcpp::spin_some(shared_from_this());
            rate.sleep();
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ground_truth_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr measurement_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ekf_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<ExtendedKalmanFilter> ekf_;

    double fs_, Ts_, R_, f_, omega_, sigma_;
    double m2mm_ = 1000.0, mm2m_ = 1.0 / 1000.0;
    Eigen::Vector3d normal_, center_;
    Eigen::Matrix3d rot_matrix_;
    int k_;
    int dim_x_, dim_z_;
    Eigen::MatrixXd Gamma_;

    Eigen::MatrixXd create_state_transition_matrix()
    {
        Eigen::MatrixXd F(dim_x_, dim_x_);
        F << 1, 0, 0, Ts_, 0, 0, 0.5 * Ts_ * Ts_, 0, 0,
             0, 1, 0, 0, Ts_, 0, 0, 0.5 * Ts_ * Ts_, 0,
             0, 0, 1, 0, 0, Ts_, 0, 0, 0.5 * Ts_ * Ts_,
             0, 0, 0, 1, 0, 0, Ts_, 0, 0,
             0, 0, 0, 0, 1, 0, 0, Ts_, 0,
             0, 0, 0, 0, 0, 1, 0, 0, Ts_,
             0, 0, 0, 0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 1;
        return F;
    }

    Eigen::MatrixXd create_measurement_matrix()
    {
        Eigen::MatrixXd H(dim_z_, dim_x_);
        H << 1, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0, 0, 0, 0;
        return H;
    }

    Eigen::MatrixXd create_process_noise_covariance()
    {
        Eigen::MatrixXd Gamma(9, 3);
        Gamma << Ts_ * Ts_ * Ts_ / 6.0, 0, 0,
                 0, Ts_ * Ts_ * Ts_ / 6.0, 0,
                 0, 0, Ts_ * Ts_ * Ts_ / 6.0,
                 Ts_ * Ts_ / 2.0, 0, 0,
                 0, Ts_ * Ts_ / 2.0, 0,
                 0, 0, Ts_ * Ts_ / 2.0,
                 Ts_, 0, 0,
                 0, Ts_, 0,
                 0, 0, Ts_;
        return Gamma;
    }

    Eigen::VectorXd fx(const Eigen::VectorXd &x)
    {
        return ekf_->getF() * x;
    }

    Eigen::MatrixXd jacobian_F(const Eigen::VectorXd &)
    {
        return ekf_->getF();
    }

    Eigen::MatrixXd jacobian_H(const Eigen::VectorXd &)
    {
        return ekf_->getH();
    }

    Eigen::VectorXd hx(const Eigen::VectorXd &x)
    {
        return x.head(3);
    }

    Eigen::Vector3d compute_trajectory_point(double t)
    {
        double x = R_ * std::cos(omega_ * t);
        double y = R_ * std::sin(omega_ * t);
        double z = 0;
        Eigen::Vector3d point(x, y, z);
        return rot_matrix_ * point + center_;
    }

    void publish_tf(const Eigen::Vector3d &position, const std::string &child_frame_id, const std::string &parent_frame_id)
    {
        geometry_msgs::msg::TransformStamped tf_stamp;
        tf_stamp.header.stamp = this->get_clock()->now();
        tf_stamp.header.frame_id = parent_frame_id;
        tf_stamp.child_frame_id = child_frame_id;

        tf_stamp.transform.translation.x = position.x();
        tf_stamp.transform.translation.y = position.y();
        tf_stamp.transform.translation.z = position.z();

        tf_stamp.transform.rotation.w = 1.0; // Identity rotation
        tf_stamp.transform.rotation.x = 0.0;
        tf_stamp.transform.rotation.y = 0.0;
        tf_stamp.transform.rotation.z = 0.0;

        tf_broadcaster_->sendTransform(tf_stamp);
    }

    void ekf_iteration()
    {
        // Compute the current point (ground truth)
        double t = k_ * Ts_;
        Eigen::Vector3d current_point = compute_trajectory_point(t);

        // Simulated sensor measurement with noise
        Eigen::Vector3d measurement_noise = Eigen::Vector3d::Random() * sigma_;
        Eigen::Vector3d measurement = current_point + measurement_noise;

        if (!measurement.allFinite()) {
            RCLCPP_WARN(this->get_logger(), "Measurement contains NaN or Inf, skipping this iteration.");
            return;
        }

        // EKF time update
        ekf_->predict(jacobian_F(ekf_->getState()), fx(ekf_->getState()));

        if (!ekf_->getState().allFinite()) {
            RCLCPP_WARN(this->get_logger(), "State contains NaN or Inf, skipping this iteration.");
            return;
        }

        // EKF measurement update
        ekf_->update(measurement, jacobian_H(ekf_->getState()), hx(ekf_->getState()));

        // EKF estimated position
        Eigen::Vector3d ekf_estimated_position = ekf_->getState().head(3);

        // Publish the ground truth position as a Point message
        geometry_msgs::msg::Point ground_truth_point;
        ground_truth_point.x = current_point.x();
        ground_truth_point.y = current_point.y();
        ground_truth_point.z = current_point.z();
        ground_truth_publisher_->publish(ground_truth_point);

        // Publish the measurement position as a Point message
        geometry_msgs::msg::Point measurement_point;
        measurement_point.x = measurement.x();
        measurement_point.y = measurement.y();
        measurement_point.z = measurement.z();
        measurement_publisher_->publish(measurement_point);

        // Publish the EKF estimated position as a Point message
        geometry_msgs::msg::Point ekf_point;
        ekf_point.x = ekf_estimated_position.x();
        ekf_point.y = ekf_estimated_position.y();
        ekf_point.z = ekf_estimated_position.z();
        ekf_publisher_->publish(ekf_point);

        // Publish the TF frames
        publish_tf(current_point, "gt", "map");
        publish_tf(measurement, "meas", "map");
        publish_tf(ekf_estimated_position, "est", "map");

        k_++;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKFNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
