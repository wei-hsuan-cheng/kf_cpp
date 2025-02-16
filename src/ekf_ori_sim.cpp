#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "Eigen/Dense"
#include "kf_cpp/ekf.hpp"
#include "robot_math_utils/robot_math_utils_v1_7.hpp"

using std::placeholders::_1;
using RM = RMUtils;

class EKFOrientation : public rclcpp::Node {
public:
    EKFOrientation() : Node("ekf_ori"), 
                        k_(0), 
                        t_(0.0), 
                        fs_(60.0), 
                        Ts_(1.0 / fs_), 
                        Gamma_(Ts_), 
                        Q_tunfac_(pow(10, 1.0)), 
                        R_tunfac_(pow(10, -1.0)) {
        quat_gt_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("quat_gt", 10);
        quat_m_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("quat_m", 10);
        quat_est_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("quat_est", 10);
        
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // EKF initialization
        q_init_ = RM::Rot2Quat(Matrix3d::Identity());
        // omega_init_ = Vector3d::Ones().normalized() * 45.0 * M_PI / 180.0;
        omega_init_ = Vector3d::Ones().normalized() * 10.0 * M_PI / 180.0;
        x_gt_ = VectorXd(7);
        x_gt_ << q_init_.w(), q_init_.x(), q_init_.y(), q_init_.z(), omega_init_;

        q_m_ = q_init_;
        omega_m_ = omega_init_;

        ekf_ = std::make_shared<ExtendedKalmanFilter>(7, 7);  // dim_x=7, dim_z=7
        Vector7d initial_state;
        initial_state << q_m_.w(), q_m_.x(), q_m_.y(), q_m_.z(), omega_m_;
        ekf_->setState(initial_state);

        // State-space model (functions or matrices)
        ekf_->setF(jacobian_F(ekf_->getState()));

        MatrixXd H(7, 7);
        H << MatrixXd::Identity(4, 4), MatrixXd::Zero(4, 3),
             MatrixXd::Zero(3, 4), MatrixXd::Identity(3, 3);
        ekf_->setH(H);

        // // Covariance matrices
        // Process noise covariance
        PNstd_quat_ = MatrixXd::Identity(4, 4) * pow(10, -4.0);
        PNstd_omega_ = MatrixXd::Identity(3, 3) * pow(10, -3.0);
        PNstd_ = MatrixXd(7, 7);
        PNstd_ << PNstd_quat_, MatrixXd::Zero(4, 3),
                  MatrixXd::Zero(3, 4), PNstd_omega_;

        ekf_->setProcessNoise(Gamma_ * PNstd_ * PNstd_.transpose() * Gamma_* pow(Q_tunfac_, 2));

        // Measurement noise covariance (experimental data and fine-tuning)
        double quat_multiplier = pow(10, 1.25);
        double omega_multiplier = pow(10, 1.0);
        MNstd_quat_ = MatrixXd::Zero(4, 4);
        MNstd_quat_(0, 0) = 0.0035652629269028323 * quat_multiplier;
        MNstd_quat_(1, 1) = 0.001124141775313557 * quat_multiplier;
        MNstd_quat_(2, 2) = 0.0032617761132252075 * quat_multiplier;
        MNstd_quat_(3, 3) = 0.0023408977903160285 * quat_multiplier;

        MNstd_omega_ = MatrixXd::Zero(3, 3);
        MNstd_omega_(0, 0) = 0.003612832752606975 * omega_multiplier;
        MNstd_omega_(1, 1) = 0.001971638070939487 * omega_multiplier;
        MNstd_omega_(2, 2) = 0.002420621344508792 * omega_multiplier;

        MNstd_ = MatrixXd(7, 7);
        MNstd_ << MNstd_quat_, MatrixXd::Zero(4, 3),
                  MatrixXd::Zero(3, 4), MNstd_omega_;

        ekf_->setMeasurementNoise(MNstd_ * MNstd_.transpose() * pow(R_tunfac_, 2));

    }

    void run() {
        rclcpp::Rate rate(fs_);  // 60 Hz
        while (rclcpp::ok()) {
            ekf_iteration();
            rclcpp::spin_some(shared_from_this());
            rate.sleep();
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr quat_gt_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr quat_m_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr quat_est_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<ExtendedKalmanFilter> ekf_;

    double fs_, Ts_, Gamma_, Q_tunfac_, R_tunfac_;
    int k_;
    double t_;

    Quaterniond q_init_, q_m_;
    Vector3d omega_init_, omega_m_;
    Vector7d x_gt_;

    MatrixXd PNstd_quat_, PNstd_omega_, PNstd_, MNstd_quat_, MNstd_omega_, MNstd_;

    MatrixXd jacobian_F(const VectorXd& x) {
        VectorXd Quat = x.head(4);
        VectorXd omega = x.tail(3);
        double omega_norm = RM::Norm(omega);
        VectorXd omega_normalized = RM::Normalized(omega);

        MatrixXd F(7, 7);
        if (omega_norm == 0) {
            F << gx(x), MatrixXd::Zero(4, 3),
                 MatrixXd::Zero(3, 4), MatrixXd::Identity(3, 3);
        } else {
            MatrixXd F12_1 = -pow((Ts_ / 2.0), 2) * RM::Sinc(omega_norm * Ts_ / 2.0) * (Quat * omega.transpose());
            MatrixXd F12_2_omega = ((cos(omega_norm * Ts_ / 2.0) * (Ts_ / 2.0) * omega_norm - (Ts_ / 2.0) * RM::Sinc(omega_norm * Ts_ / 2.0)) * (omega_normalized * omega_normalized.transpose())) + ((Ts_ / 2.0) * RM::Sinc(omega_norm * Ts_ / 2.0) * MatrixXd::Identity(3, 3));
            MatrixXd F12_2 = (MatrixXd(4, 3) << -Quat.tail(3).transpose(), MatrixXd::Identity(3, 3) * Quat(0) - RM::R3Vec2so3Mat(Quat.tail(3))).finished() * F12_2_omega;
            MatrixXd F12 = F12_1 + F12_2;

            F << gx(x), F12,
                 MatrixXd::Zero(3, 4), MatrixXd::Identity(3, 3);
        }

        return F;
    }

    MatrixXd gx(const VectorXd& x) {
        VectorXd omega = x.tail(3);
        double omega_norm = RM::Norm(omega);
        if (omega_norm == 0) {
            return MatrixXd::Identity(4, 4);
        }
        return cos(omega_norm * Ts_ / 2.0) * MatrixXd::Identity(4, 4) + (Ts_ / 2.0) * RM::Sinc(omega_norm * Ts_ / 2.0) * Omegax(x);
    }

    MatrixXd Omegax(const VectorXd& x) {
        VectorXd omega = x.tail(3);
        MatrixXd Omegax(4, 4);
        Omegax << 0, -omega.transpose(),
                  omega, RM::R3Vec2so3Mat(omega);
        return Omegax;
    }

    VectorXd fx(const VectorXd& x) {
        VectorXd Quat = x.head(4);
        VectorXd omega = x.tail(3);
        VectorXd result(7);
        result << gx(x) * Quat, omega;
        return result;
    }

    MatrixXd jacobian_H(const VectorXd& x) {
        return ekf_->getH();
    }

    VectorXd hx(const VectorXd& x) {
        return x.head(7);  // dim_z = 7
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

    VectorXd gt_motion(Vector7d& x_gt) {
        Vector4d quat = x_gt.head(4);
        Vector3d omega = x_gt.tail(3);
        double omega_norm = RM::Norm(omega);
        if (omega_norm > 0) {
            MatrixXd delta_q = cos(omega_norm * Ts_ / 2.0) * MatrixXd::Identity(4, 4) + (1.0 / omega_norm) * sin(omega_norm * Ts_ / 2.0) * Omegax((VectorXd(7) << quat, omega).finished());
            quat = delta_q * quat;
            quat = RM::Normalized(quat);
        }
        x_gt.head(4) = quat;

        // Add process noise to state
        for (int i = 0; i < 7; i++) {
            x_gt(i) += PNstd_(i, i) * RM::RandNorDist();
        }

        return x_gt;
    }

    void ekf_iteration() {
        // Compute the current orientation (ground-truth)
        t_ = k_ * Ts_;
        x_gt_ = gt_motion(x_gt_);
        
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.assign(x_gt_.data(), x_gt_.data() + x_gt_.size());
        quat_gt_pub_->publish(msg);

        // Simulated sensor measurement with noise
        VectorXd measurement_noise = VectorXd::Zero(7); // dim_z = 7
        for (int i = 0; i < 7; i++) {
            measurement_noise(i) = MNstd_(i, i) * RM::RandNorDist(); // Adding noise
        }
        VectorXd measurement = hx(x_gt_) + measurement_noise;

        if (measurement.hasNaN() || measurement.array().isInf().any()) {
            RCLCPP_WARN(this->get_logger(), "Measurement contains NaN or Inf, skipping this iteration.");
            return;
        }

        // EKF time update
        ekf_->predict(jacobian_F(ekf_->getState()), fx(ekf_->getState()));
        VectorXd state = ekf_->getState();
        state.head(4) = state.head(4).normalized();
        ekf_->setState(state);

        if (ekf_->getState().hasNaN() || ekf_->getState().array().isInf().any()) {
            RCLCPP_WARN(this->get_logger(), "State contains NaN or Inf, skipping this iteration.");
            return;
        }

        // EKF measurement update
        ekf_->update(measurement, jacobian_H(ekf_->getState()), hx(ekf_->getState()));
        state = ekf_->getState();
        state.head(4) = state.head(4).normalized();
        ekf_->setState(state);

        VectorXd q_est = ekf_->getState().head(4);
        VectorXd q_gt = x_gt_.head(4);
        VectorXd omega_est = ekf_->getState().tail(3);
        VectorXd omega_gt = x_gt_.tail(3);

        // Publish the estimated quaternion and angular velocity
        std::string state_str = "EKF State: ";
        for (int i = 0; i < state.size(); ++i) {
            state_str += std::to_string(state[i]) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", state_str.c_str());
        msg.data.assign(state.data(), state.data() + state.size());
        quat_est_pub_->publish(msg);

        // Publish quaternion measurement
        msg.data.assign(measurement.data(), measurement.data() + measurement.size());
        quat_m_pub_->publish(msg);

        // Publish the TF frames
        publish_tf(q_gt, "gt", "map");
        publish_tf(measurement.head(4), "meas", "map");
        publish_tf(q_est, "est", "map");

        k_++;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKFOrientation>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
