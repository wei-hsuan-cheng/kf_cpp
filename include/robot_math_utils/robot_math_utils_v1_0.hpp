#ifndef RM_UTILS_HPP
#define RM_UTILS_HPP

// Author: Wei-Hsuan Cheng, johnathancheng0125@gmail.com, https://github.com/wei-hsuan-cheng
// ver_1.0, last edit: 241002
// Some functions are adapted from the Modern Robotics book codebase: https://github.com/Le0nX/ModernRoboticsCpp/tree/eacdf8800bc591b03727512c102d2c5dffe78cec

#include <Eigen/Dense>
#include <deque>
#include <utility>
#include <fstream>
#include <iomanip>  // For setting date format
#include <chrono>   // For timestamp
#include <ctime>    // For time conversion
#include <cmath>
#include <random> // For random number generation
#include <unsupported/Eigen/MatrixFunctions>  // For matrix logarithm and exponential
#include <iostream>

class RMUtils {
public:
    /* Data logging */
    static void PrintVec(const Eigen::VectorXd& vec, const std::string& vec_name) {
        std::cout << vec_name << " = ";
        for (int i = 0; i < vec.size(); ++i)
        {
            std::cout << vec[i];
            if (i < vec.size() - 1)
            {
                std::cout << ", ";  // Add comma between elements
            }
        }
        std::cout << std::endl;
    }

    static void InitDatalog(std::ofstream& csv_writer_, const std::string& datalog_filename, const std::vector<std::string>& header_row) {
        csv_writer_.open(datalog_filename);
        if (csv_writer_.is_open()) {
            // Write the header row from the provided vector of strings
            for (size_t i = 0; i < header_row.size(); ++i) {
                csv_writer_ << header_row[i];
                if (i < header_row.size() - 1) {
                    csv_writer_ << ",";  // Add a comma separator between header entries
                }
            }
            csv_writer_ << "\n";  // End the header row
        }
    }

    static void Datalog(std::ofstream& csv_writer_, std::_Put_time<char> datetime, int k, double Ts, double time_elapsed, const std::vector<Eigen::VectorXd> &data_vectors)
    {
        if (!csv_writer_.is_open()) return;

        // Start writing the timestamp and elapsed time
        csv_writer_ << datetime  << "," << k << "," << k * Ts << "," << time_elapsed; // datetime, k [idx], kTs [s], t [s] 
        for (const auto &vec : data_vectors)
        {
            for (int i = 0; i < vec.size(); ++i)
            {
                csv_writer_ << "," << vec[i];  // Add each element separated by a comma
            }
        }
        csv_writer_ << "\n"; // End of each row
    }



    /* Numerical conditions */
    static bool NearZero(const double val, double thresh = pow(10, -7)) {
		return (std::abs(val) < thresh);
	}

    static Eigen::VectorXd MeanBuffer(const std::deque<Eigen::VectorXd> &buffer) {
        /* Mean in a sliding window */
        Eigen::VectorXd mean = Eigen::VectorXd::Zero(buffer.front().size());
        for (const auto& vec : buffer) {
            mean += vec;
        }
        mean /= buffer.size();
        return mean;
    }

    static Eigen::VectorXd StdBuffer(const std::deque<Eigen::VectorXd> &buffer) {
        /* Standard deviation in a sliding window */
        Eigen::VectorXd mean = MeanBuffer(buffer);
        Eigen::VectorXd variance = Eigen::VectorXd::Zero(buffer.front().size());
        for (const auto& vec : buffer) {
            Eigen::VectorXd diff = vec - mean;
            variance += diff.array().square().matrix();
        }
        variance /= buffer.size();
        return variance.array().sqrt();
    }

    static Eigen::VectorXd MAvg(const Eigen::VectorXd &vec, std::deque<Eigen::VectorXd> &buffer, std::size_t window_size) {
        /* Moving average */
        buffer.push_back(vec);
        if (buffer.size() > window_size) {
            buffer.pop_front();
        } // Ensure buffer size is within window_size
        if (buffer.size() <= window_size) {
            // Buffer is not yet full, handle accordingly
            // For example, return the average over the available samples
            return MeanBuffer(buffer);
        }
    }



    /* Basic math functions */
    static double ErrorPercentage(double meas, double gt) {
        return (meas - gt) / gt;
    }

    static double Sinc(double x) {
        return ( NearZero(x) ) ? cos(x) : std::sin(x) / x;
    }

    static double Norm(const Eigen::VectorXd& v) {
        return v.norm();
    }

    static double MatNorm(const Eigen::MatrixXd& M) {
        return M.norm();
    }

    static Eigen::VectorXd Normalized(const Eigen::VectorXd& v) {
        return v.normalized();
    }

    static Eigen::MatrixXd Inv(const Eigen::MatrixXd& M) {
        return M.inverse();
    }

    static Eigen::MatrixXd Transpose(const Eigen::MatrixXd& M) {
        return M.transpose();
    }

    static double Det(const Eigen::MatrixXd& M) {
        return M.determinant();
    }

    static double Tr(const Eigen::MatrixXd& M) {
        return M.trace();
    }

    static double ConstrainedAngle(double angle, bool rad = true) {
        if (!rad) {
            angle *= M_PI / 180.0;
        }
        if (angle >= M_PI && angle < 2 * M_PI) {
            angle -= 2 * M_PI;
        } else if (angle >= -2 * M_PI && angle < -M_PI) {
            angle += 2 * M_PI;
        }
        if (!rad) {
            angle *= 180.0 / M_PI;
        }
        return angle;
    }

    static double random_normal(double mean = 0.0, double stddev = 1.0) { // random normal function
        static std::mt19937 rng(std::random_device{}()); // Standard mersenne_twister_engine
        std::normal_distribution<double> dist(mean, stddev);
        return dist(rng);
    }


    /* Robot transformation functions */
    // Homogeneous coordinates
    static Eigen::Vector4d R3Vec2Homo(const Eigen::Vector3d& v, double s = 1.0) {
        if (v.size() != 3) {
            throw std::invalid_argument("The input pose must have exactly 3 elements.");
        } 
        Eigen::Vector4d v_h;
        v_h << v(0) * s, v(1) * s, v(2) * s, s;
        return v_h;
    }

    static std::vector<Eigen::Vector4d> R3Vecs2Homos(const std::vector<Eigen::Vector3d>& r3_vecs, double s = 1.0) {
        if (r3_vecs.size() == 0) {
            throw std::invalid_argument("The input list of vectors is empty.");
        }
        std::vector<Eigen::Vector4d> r3_vecs_h;
        for (const auto& r3_vec : r3_vecs) {
            if (r3_vec.size() != 3) {
                throw std::invalid_argument("Each vector must have exactly 3 elements.");
            }
            Eigen::Vector4d r3_vec_h = R3Vec2Homo(r3_vec, s);
            r3_vecs_h.push_back(r3_vec_h);
        }
        return r3_vecs_h; // {v1_h, v2_h, v3_h, ...}
    }
    
    static Eigen::Vector3d Homo2R3Vec(const Eigen::Vector4d& v_h) {
        if (v_h.size() != 4) {
            throw std::invalid_argument("The input vector must have exactly 4 elements.");
        }
        if (v_h(3) == 0) {
            throw std::invalid_argument("The homogeneous coordinate (last element) must not be zero.");
        }
        Eigen::Vector3d v;
        v << v_h(0) / v_h(3), v_h(1) / v_h(3), v_h(2) / v_h(3);
        return v;
    }

    static std::vector<Eigen::Vector3d> Homos2R3Vecs(const std::vector<Eigen::Vector4d>& r3_vec_hs) {
        if (r3_vec_hs.size() == 0) {
            throw std::invalid_argument("The input list of homogeneous vectors is empty.");
        }
        std::vector<Eigen::Vector3d> r3_vecs;
        for (const auto& r3_vec_h : r3_vec_hs) {
            if (r3_vec_h.size() != 4) {
                throw std::invalid_argument("Each homogeneous vector must have exactly 4 elements.");
            }
            Eigen::Vector3d r3_vec = Homo2R3Vec(r3_vec_h);
            r3_vecs.push_back(r3_vec);
        }
        return r3_vecs; // {v1, v2, v3, ...}
    }

    static Eigen::Vector3d ImgCoord2Homo(const Eigen::Vector2d& img_coord, double s = 1.0) {
        if (img_coord.size() != 2) {
            throw std::invalid_argument("The input image coordinate must have exactly 2 elements.");
        }
        Eigen::Vector3d img_coord_h;
        img_coord_h << img_coord(0) * s, img_coord(1) * s, s;
        return img_coord_h;
    }

    static std::vector<Eigen::Vector3d> ImgCoords2Homos(const std::vector<Eigen::Vector2d>& img_coords, double s = 1.0) {
        if (img_coords.size() == 0) {
            throw std::invalid_argument("The input list of image coordinates is empty.");
        }
        std::vector<Eigen::Vector3d> img_coords_h;
        for (const auto& img_coord : img_coords) {
            if (img_coord.size() != 2) {
                throw std::invalid_argument("Each image coordinate must have exactly 2 elements.");
            }
            Eigen::Vector3d img_coord_h = ImgCoord2Homo(img_coord, s);
            img_coords_h.push_back(img_coord_h);
        }
        return img_coords_h; // {img_coord_h_1, img_coord_h_2, img_coord_h_3, ...}
    }

    static Eigen::Vector2d Homo2ImgCoord(const Eigen::Vector3d& img_coord_h) {
        if (img_coord_h.size() != 3) {
            throw std::invalid_argument("The input image coordinate must have exactly 3 elements.");
        }
        if (img_coord_h(2) == 0) {
            throw std::invalid_argument("The homogeneous coordinate (last element) must not be zero.");
        }
        Eigen::Vector2d img_coord;
        img_coord << img_coord_h(0) / img_coord_h(2), img_coord_h(1) / img_coord_h(2);
        return img_coord;
    }

    static std::vector<Eigen::Vector2d> Homos2ImgCoords(const std::vector<Eigen::Vector3d>& img_coords_hs) {
        if (img_coords_hs.size() == 0) {
            throw std::invalid_argument("The input list of homogeneous image coordinates is empty.");
        }
        std::vector<Eigen::Vector2d> img_coords;
        for (const auto& img_coord_h : img_coords_hs) {
            if (img_coord_h.size() != 3) {
                throw std::invalid_argument("Each homogeneous image coordinate must have exactly 3 elements.");
            }
            Eigen::Vector2d img_coord = Homo2ImgCoord(img_coord_h);
            img_coords.push_back(img_coord);
        }
        return img_coords; // {img_coord_1, img_coord_2, img_coord_3, ...}
    }

    



    // SO(3) and so(3) functions
    static Eigen::Matrix3d R3Vec2Skew(const Eigen::Vector3d& v) {
        Eigen::Matrix3d skew;
        skew <<  0,      -v(2),   v(1),
                 v(2),   0,      -v(0),
                -v(1),   v(0),    0;
        return skew;
    }

    static Eigen::Vector3d Skew2Vec(const Eigen::Matrix3d& skew) {
        if ( !NearZero( MatNorm(skew + Transpose(skew)) ) ) {
            throw std::invalid_argument("The input matrix is not skew-symmetric.");
        }
        Eigen::Vector3d v;
        v << skew(2, 1), skew(0, 2), skew(1, 0);
        return v;
    }

    static Eigen::Matrix3d R3Vec2so3Mat(const Eigen::Vector3d& v) {
        Eigen::Matrix3d so3Mat;
        so3Mat <<  0,      -v(2),   v(1),
                 v(2),   0,      -v(0),
                -v(1),   v(0),    0;
        return so3Mat;
    }

    static Eigen::Vector3d so3Mat2R3Vec(const Eigen::MatrixXd& so3Mat) {
        if ( !NearZero( MatNorm(so3Mat + Transpose(so3Mat)) ) ) {
            throw std::invalid_argument("The input matrix is not skew-symmetric.");
        }
        Eigen::Vector3d v;
		v << so3Mat(2, 1), so3Mat(0, 2), so3Mat(1, 0);
		return v;
	}
    
     static Eigen::Vector4d AxisAng3(const Eigen::Vector3d& so3) {
        if (so3.size() != 3) {
            throw std::invalid_argument("The input vector must have exactly 3 elements.");
        }
		Eigen::Vector4d v;
		v << Normalized(so3), Norm(so3);
		return v; // {uhat_x, uhat_y, uhat_z, theta}
	}

    static Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d& so3Mat) {
		Eigen::Vector3d so3 = so3Mat2R3Vec(so3Mat);
		Eigen::Matrix3d m_ret = Eigen::Matrix3d::Identity();
		if ( NearZero( MatNorm(so3Mat) ) ) {
			return m_ret;
		}
		else {
			double theta = (AxisAng3(so3))(3);
			Eigen::Matrix3d uhatMat = so3Mat * (1 / theta);
			return m_ret + std::sin(theta) * uhatMat + ((1 - std::cos(theta)) * (uhatMat * uhatMat));
		}
	}
    
    static Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d& R) {
		double acosinput = (Tr(R) - 1) / 2.0;
		Eigen::MatrixXd m_ret = Eigen::MatrixXd::Zero(3, 3);
		if (acosinput >= 1)
			return m_ret;
		else if (acosinput <= -1) {
			Eigen::Vector3d uhat;
			if ( !NearZero(1 + R(2, 2)) )
				uhat = (1.0 / std::sqrt(2 * (1 + R(2, 2)))) * Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
			else if ( !NearZero(1 + R(1, 1)) )
				uhat = (1.0 / std::sqrt(2 * (1 + R(1, 1)))) * Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
			else
				uhat = (1.0 / std::sqrt(2 * (1 + R(0, 0)))) * Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
			m_ret = R3Vec2so3Mat(M_PI * uhat);
			return m_ret;
		}
		else {
			double theta = std::acos(acosinput);
			// m_ret = theta / 2.0 / sin(theta) * ( R - Transpose(R) ); // Originally from modern_robotics_cpp
            m_ret = ( R - Transpose(R) ) / (2 * Sinc(theta)); // Improved version (by Sinc function)
			return m_ret;
		}
	}
    
    static Eigen::Matrix3d Rotx(double thx, bool rad = true) {
        if (!rad) {
            thx *= M_PI / 180.0;
        }
        Eigen::Matrix3d Rx;
        Rx << 1, 0,        0,
              0, cos(thx), -sin(thx),
              0, sin(thx), cos(thx);
        return Rx;
    }

    static Eigen::Matrix3d Roty(double thy, bool rad = true) {
        if (!rad) {
            thy *= M_PI / 180.0;
        }
        Eigen::Matrix3d Ry;
        Ry << cos(thy),  0, sin(thy),
              0,         1, 0,
             -sin(thy), 0, cos(thy);
        return Ry;
    }

    static Eigen::Matrix3d Rotz(double thz, bool rad = true) {
        if (!rad) {
            thz *= M_PI / 180.0;
        }
        Eigen::Matrix3d Rz;
        Rz << cos(thz), -sin(thz), 0,
              sin(thz), cos(thz),  0,
              0,       0,         1;
        return Rz;
    }

    static Eigen::Matrix3d Rotxyz(const Eigen::Vector3d& thxyz, bool rad = true) {
        Eigen::Matrix3d Rx = Rotx(thxyz(0), rad);
        Eigen::Matrix3d Ry = Roty(thxyz(1), rad);
        Eigen::Matrix3d Rz = Rotz(thxyz(2), rad);
        return Rx * Ry * Rz;
    }

    static Eigen::Matrix3d Rotzyx(const Eigen::Vector3d& thzyx, bool rad = true) {
        Eigen::Matrix3d Rz = Rotz(thzyx(0), rad);
        Eigen::Matrix3d Ry = Roty(thzyx(1), rad);
        Eigen::Matrix3d Rx = Rotx(thzyx(2), rad);
        return Rz * Ry * Rx;
    }

    static Eigen::Vector3d Rot2zyxEuler(const Eigen::Matrix3d& Rotzyx, bool rad = true) {
        double thy = atan2(-Rotzyx(2, 0), sqrt(Rotzyx(0, 0) * Rotzyx(0, 0) + Rotzyx(1, 0) * Rotzyx(1, 0)));
        double thz = atan2(Rotzyx(1, 0) / cos(thy), Rotzyx(0, 0) / cos(thy));
        double thx = atan2(Rotzyx(2, 1) / cos(thy), Rotzyx(2, 2) / cos(thy));

        if (!rad) {
            thx *= 180.0 / M_PI;
            thy *= 180.0 / M_PI;
            thz *= 180.0 / M_PI;
        }
        return Eigen::Vector3d(thz, thy, thx);
    }

    static Eigen::Vector3d Rot2so3(const Eigen::Matrix3d& R) {
        return so3Mat2R3Vec( MatrixLog3(R) );
    }

    static Eigen::Matrix3d so32Rot(const Eigen::Vector3d& so3) {
        return MatrixExp3( R3Vec2so3Mat(so3) );
    }

    static Eigen::Vector4d Rot2Quat(const Eigen::Matrix3d& R) {
        Eigen::Quaterniond quat(R);
        return Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
    }

    static Eigen::Matrix3d Quat2Rot(const Eigen::Vector4d& quat) {
        Eigen::Quaterniond q(quat(0), quat(1), quat(2), quat(3));
        return q.toRotationMatrix();
    }

    static Eigen::Vector4d zyxEuler2Quat(const Eigen::Vector3d& zyx_euler, bool rad = true) {
        Eigen::Matrix3d R = Rotzyx(zyx_euler, rad);
        return Rot2Quat(R);
    }

    static Eigen::Vector3d Quat2zyxEuler(const Eigen::Vector4d& quat, bool rad = true) {
        Eigen::Matrix3d R = Quat2Rot(quat);
        return Rot2zyxEuler(R, rad);
    }

    static Eigen::Vector4d so32Quat(const Eigen::Vector3d& so3) {
        return Rot2Quat( so32Rot(so3) );
    }

    static Eigen::Vector3d Quat2so3(const Eigen::Vector4d& quat) {
        return Rot2so3( Quat2Rot(quat) );
    }

    static Eigen::Matrix3d ThreeAxes2Rot(const Eigen::Vector3d& x_1_2, const Eigen::Vector3d& y_1_2, const Eigen::Vector3d& z_1_2) {
        Eigen::Matrix3d R_1_2;
        R_1_2.col(0) = x_1_2;
        R_1_2.col(1) = y_1_2;
        R_1_2.col(2) = z_1_2;
        return R_1_2;
    }





    // SE(3) and se(3) functions
    static Eigen::MatrixXd R6Vec2se3Mat(const Eigen::VectorXd& V) {
		// linear and angular parts
		Eigen::Vector3d linear(V(0), V(1), V(2));
		Eigen::Vector3d ang(V(3), V(4), V(5));
		Eigen::MatrixXd M(4, 4);
		M <<           R3Vec2so3Mat(ang), linear,
			 Eigen::MatrixXd::Zero(1, 3),      0;
		return M;
	}

    static Eigen::VectorXd se3Mat2R6Vec(const Eigen::MatrixXd& se3Mat) {
        if (se3Mat.rows() != 4 || se3Mat.cols() != 4) {
            throw std::invalid_argument("The input matrix must be 4x4.");
        }
        Eigen::VectorXd V(6);
        V << se3Mat(0, 3), se3Mat(1, 3), se3Mat(2, 3), se3Mat(2, 1), se3Mat(0, 2), se3Mat(1, 0); 
        return V; // {vx, vy, vz, wx, wy, wz}
    }

    static Eigen::VectorXd AxisAng6(const Eigen::VectorXd& se3) {
		Eigen::VectorXd v_ret(7);
		double theta = Norm( Eigen::Vector3d(se3(3), se3(4), se3(5)) ); // angular part
		if (NearZero(theta))
			theta = Norm( Eigen::Vector3d(se3(0), se3(1), se3(2)) ); // linear part
		v_ret << se3 / theta, theta;
		return v_ret;
	}

    static Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd& se3Mat) {
		// Extract the angular velocity vector from the transformation matrix
		Eigen::Matrix3d se3Mat_cut = se3Mat.block<3, 3>(0, 0);
		Eigen::Vector3d so3 = so3Mat2R3Vec(se3Mat_cut);
		Eigen::MatrixXd m_ret(4, 4);
		// If negligible rotation, m_Ret = [[Identity, angular velocty ]]
		//									[	0	 ,		1		   ]]
		if ( NearZero(Norm(so3)) ) {
			se3Mat_cut = Eigen::MatrixXd::Identity(3, 3);
			so3 << se3Mat(0, 3), se3Mat(1, 3), se3Mat(2, 3);
			m_ret <<                  se3Mat_cut, so3,
				     Eigen::MatrixXd::Zero(1, 3),   1;
			return m_ret;
		}
		// If not negligible, MR page 105
		else {
			double theta = (AxisAng3(so3))(3);
			Eigen::Matrix3d uhatMat = se3Mat.block<3, 3>(0, 0) / theta;
			Eigen::Matrix3d expExpand = Eigen::MatrixXd::Identity(3, 3) * theta + (1 - std::cos(theta)) * uhatMat + ((theta - std::sin(theta)) * (uhatMat * uhatMat));
			Eigen::Vector3d linear(se3Mat(0, 3), se3Mat(1, 3), se3Mat(2, 3));
			Eigen::Vector3d GThetaV = (expExpand * linear) / theta;
			m_ret <<      MatrixExp3(se3Mat_cut), GThetaV,
				     Eigen::MatrixXd::Zero(1, 3),       1;
			return m_ret;
		}
	}

    static Eigen::MatrixXd MatrixLog6(const Eigen::MatrixXd& T) {
		Eigen::MatrixXd m_ret(4, 4);
        auto [R, p] = SE32RotPos(T);
        Eigen::Matrix3d uhatMat = MatrixLog3(R);
        if (NearZero( MatNorm(uhatMat) )) {
			m_ret << Eigen::MatrixXd::Zero(3, 3), p,
				     Eigen::MatrixXd::Zero(1, 3), 0;
		}
        else {
            double theta = std::acos((Tr(R) - 1) / 2.0);
            Eigen::Matrix3d logExpand1 = Eigen::MatrixXd::Identity(3, 3) - uhatMat / 2.0;
            Eigen::Matrix3d logExpand2 = (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2) * uhatMat * uhatMat / theta;
            Eigen::Matrix3d logExpand = logExpand1 + logExpand2;
            m_ret <<                     uhatMat, logExpand * p,
                     Eigen::MatrixXd::Zero(1, 3),             0;
        }
        return m_ret;
	}

    static Eigen::Matrix4d R6Pose2SE3(const Eigen::VectorXd& pose_1_2) {
        Eigen::Matrix4d T_1_2 = Eigen::Matrix4d::Identity();
        T_1_2.topLeftCorner<3, 3>() = Rotzyx(pose_1_2.tail<3>().reverse(), true);
        T_1_2.topRightCorner<3, 1>() = pose_1_2.head<3>();
        return T_1_2;
    }

    static Eigen::Matrix4d R6Poses2SE3(const Eigen::VectorXd& pose_b_1, const Eigen::VectorXd& pose_b_2) {
        Eigen::Matrix4d T_b_1 = R6Pose2SE3(pose_b_1);
        Eigen::Matrix4d T_b_2 = R6Pose2SE3(pose_b_2);
        return Inv(T_b_1) * T_b_2; // T_1_2
    }

    static std::pair<Eigen::Matrix3d, Eigen::Vector3d> R6Pose2RotPos(const Eigen::VectorXd& pose_1_2) {
        Eigen::Matrix4d T_1_2 = R6Pose2SE3(pose_1_2);
        Eigen::Matrix3d R_1_2 = T_1_2.topLeftCorner<3, 3>();
        Eigen::Vector3d p_1_2 = T_1_2.topRightCorner<3, 1>();
        return std::make_pair(R_1_2, p_1_2);
    }

    static std::pair<Eigen::Matrix3d, Eigen::Vector3d> R6Poses2RotPos(const Eigen::VectorXd& pose_b_1, const Eigen::VectorXd& pose_b_2) {
        Eigen::Matrix4d T_1_2 = R6Poses2SE3(pose_b_1, pose_b_2);
        Eigen::Matrix3d R_1_2 = T_1_2.topLeftCorner<3, 3>();
        Eigen::Vector3d p_1_2 = T_1_2.topRightCorner<3, 1>();
        return std::make_pair(R_1_2, p_1_2);
    }

    static Eigen::Matrix4d RotPos2SE3(const Eigen::Matrix3d& R_1_2, const Eigen::Vector3d& p_1_2) {
        Eigen::Matrix4d T_1_2 = Eigen::Matrix4d::Identity();
        T_1_2.topLeftCorner<3, 3>() = R_1_2;
        T_1_2.topRightCorner<3, 1>() = p_1_2;
        return T_1_2;
    }

    static std::pair<Eigen::Matrix3d, Eigen::Vector3d> SE32RotPos(const Eigen::Matrix4d& T_1_2) {
        Eigen::Matrix3d R_1_2 = T_1_2.topLeftCorner<3, 3>();
        Eigen::Vector3d p_1_2 = T_1_2.topRightCorner<3, 1>();
        return std::make_pair(R_1_2, p_1_2);
    }

    static Eigen::VectorXd RotPos2R6Pose(const Eigen::Matrix3d& R_1_2, const Eigen::Vector3d& p_1_2) {
        Eigen::VectorXd pose_1_2(6);
        pose_1_2.head<3>() = p_1_2;
        pose_1_2.tail<3>() = Rot2zyxEuler(R_1_2, true).reverse();
        return pose_1_2;
    }

    static Eigen::VectorXd SE32R6Pose(const Eigen::Matrix4d& T_1_2) {
        Eigen::Matrix3d R_1_2 = T_1_2.topLeftCorner<3, 3>();
        Eigen::Vector3d p_1_2 = T_1_2.topRightCorner<3, 1>();
        Eigen::VectorXd pose_1_2(6);
        pose_1_2.head<3>() = p_1_2;
        pose_1_2.tail<3>() = Rot2zyxEuler(R_1_2, true).reverse();
        return pose_1_2;
    }

    static Eigen::VectorXd SE32Posso3(const Eigen::Matrix4d& T_1_2) {
        Eigen::VectorXd pos_so3_1_2(6);
        pos_so3_1_2.head<3>() = T_1_2.topRightCorner<3, 1>(); // Position
        // pos_so3_1_2.tail<3>() = so3Mat2R3Vec( MatrixLog3( T_1_2.topLeftCorner<3, 3>() ) ); // Orientation [rad]
        pos_so3_1_2.tail<3>() = Rot2so3(T_1_2.topLeftCorner<3, 3>()); // Orientation [rad]
        return pos_so3_1_2;
    }

    static Eigen::Matrix4d Posso32SE3(const Eigen::VectorXd& pos_so3_1_2) {
        if (pos_so3_1_2.size() != 6) {
            throw std::invalid_argument("The input vector must have exactly 6 elements.");
        }
        Eigen::Matrix4d T_1_2 = Eigen::Matrix4d::Identity();
        // T_1_2.topLeftCorner<3, 3>() = MatrixExp3( R3Vec2so3Mat( pos_so3_1_2.tail<3>() ) ); // Set rotation part
        T_1_2.topLeftCorner<3, 3>() = so32Rot( pos_so3_1_2.tail<3>()); // Set rotation part
        T_1_2.topRightCorner<3, 1>() = pos_so3_1_2.head<3>(); // Set position part
        return T_1_2;
    }

    static Eigen::VectorXd R6Pose2PosQuat(const Eigen::VectorXd& pose_1_2) {
        if (pose_1_2.size() != 6) {
            throw std::invalid_argument("The input pose must have exactly 6 elements.");
        }
        auto [R_1_2, p_1_2] = R6Pose2RotPos(pose_1_2);
        Eigen::VectorXd pos_quat_1_2(7);
        pos_quat_1_2.head<3>() = p_1_2;
        pos_quat_1_2.tail<4>() = Rot2Quat(R_1_2);
        pos_quat_1_2.tail<4>() = zyxEuler2Quat(pose_1_2.tail<3>().reverse(), true);
        return pos_quat_1_2;
    }

    static Eigen::VectorXd PosQuat2R6Pose(const Eigen::VectorXd& pos_quat_1_2) {
        if (pos_quat_1_2.size() != 7) {
            throw std::invalid_argument("The input vector must have exactly 7 elements.");
        }
        Eigen::VectorXd pose_1_2(6);
        pose_1_2.head<3>() = pos_quat_1_2.head<3>();
        pose_1_2.tail<3>() = Rot2zyxEuler(Quat2Rot( pos_quat_1_2.tail<4>() ), true).reverse();
        return pose_1_2;
    }

    static Eigen::Matrix4d PosQuat2SE3(const Eigen::VectorXd& pos_quat_1_2) {
        if (pos_quat_1_2.size() != 7) {
            throw std::invalid_argument("The input vector must have exactly 7 elements.");
        }
        Eigen::Matrix4d T_1_2 = Eigen::Matrix4d::Identity();
        T_1_2.topLeftCorner<3, 3>() = Quat2Rot( pos_quat_1_2.tail<4>() );
        T_1_2.topRightCorner<3, 1>() = pos_quat_1_2.head<3>();
        return T_1_2;
    }

    static Eigen::VectorXd SE32PosQuat(const Eigen::Matrix4d& T_1_2) {
        Eigen::VectorXd pos_quat_1_2(7);
        pos_quat_1_2.head<3>() = T_1_2.topRightCorner<3, 1>();
        pos_quat_1_2.tail<4>() = Rot2Quat( T_1_2.topLeftCorner<3, 3>() );
        return pos_quat_1_2;
    }

    static Eigen::VectorXd PosQuat2Posso3(const Eigen::VectorXd& pos_quat_1_2) {
        if (pos_quat_1_2.size() != 7) {
            throw std::invalid_argument("The input vector must have exactly 7 elements.");
        }
        Eigen::Matrix4d T_1_2 = PosQuat2SE3(pos_quat_1_2);
        return SE32Posso3(T_1_2);
    }

    static Eigen::VectorXd Posso32PosQuat(const Eigen::VectorXd& pos_so3_1_2) {
        if (pos_so3_1_2.size() != 6) {
            throw std::invalid_argument("The input vector must have exactly 6 elements.");
        }
        Eigen::Matrix4d T_1_2 = Posso32SE3(pos_so3_1_2);
        return SE32PosQuat(T_1_2);
    }

    static Eigen::VectorXd InvR6Pose(const Eigen::VectorXd& pose_1_2) {
        if (pose_1_2.size() != 6) {
            throw std::invalid_argument("The input pose must have exactly 6 elements.");
        }
        return Inv( R6Pose2SE3(pose_1_2) );
    }

    static Eigen::VectorXd InvPosQuat(const Eigen::VectorXd& pos_quat_1_2) {
        if (pos_quat_1_2.size() != 7) {
            throw std::invalid_argument("The input vector must have exactly 7 elements.");
        }
        return PosQuat2R6Pose( Inv( PosQuat2SE3(pos_quat_1_2) ) );
    }

    // Transform poses and relative poses
    static Eigen::VectorXd R6Poses2RelativeR6Pose(const Eigen::VectorXd& pose_b_1, const Eigen::VectorXd& pose_b_2) {
        if (pose_b_1.size() != 6 || pose_b_2.size() != 6) {
            throw std::invalid_argument("Each pose must have exactly 6 elements.");
        }
        Eigen::Matrix4d T_1_2 = R6Poses2SE3(pose_b_1, pose_b_2);
        return SE32R6Pose(T_1_2);
    }

    static Eigen::VectorXd TransformR6Pose(const Eigen::VectorXd& pose_b_1, const Eigen::VectorXd& pose_1_2) {
        if (pose_b_1.size() != 6 || pose_1_2.size() != 6) {
            throw std::invalid_argument("Each pose must have exactly 6 elements.");
        }
        Eigen::Matrix4d T_b_2 = R6Pose2SE3(pose_b_1) * R6Pose2SE3(pose_1_2);
        return SE32R6Pose(T_b_2);
    }

    static Eigen::VectorXd TransformR6Poses(const std::initializer_list<Eigen::VectorXd>& poses) {
        if (poses.size() == 0) {
            throw std::invalid_argument("The input initializer list of poses is empty.");
        }
        Eigen::Matrix4d T_init_final = Eigen::Matrix4d::Identity();
        for (const auto& pose : poses) {
            if (pose.size() != 6) {
                throw std::invalid_argument("Each pose must have exactly 6 elements.");
            }
            T_init_final *= R6Pose2SE3(pose);
        }
        return SE32R6Pose(T_init_final); // pose_init_final
    }

    static Eigen::VectorXd TransformPosQuat(const Eigen::VectorXd& pos_quat_b_1, const Eigen::VectorXd& pos_quat_1_2) {
        if (pos_quat_b_1.size() != 7 || pos_quat_1_2.size() != 7) {
            throw std::invalid_argument("Each pose must have exactly 7 elements.");
        }
        Eigen::Matrix4d T_b_2 = PosQuat2SE3(pos_quat_b_1) * PosQuat2SE3(pos_quat_1_2);
        return SE32PosQuat(T_b_2);
    }

    static Eigen::VectorXd TransformPosQuats(const std::initializer_list<Eigen::VectorXd>& pos_quats) {
        if (pos_quats.size() == 0) {
            throw std::invalid_argument("The input initializer list of poses is empty.");
        }
        Eigen::Matrix4d T_init_final = Eigen::Matrix4d::Identity();
        for (const auto& pos_quat : pos_quats) {
            if (pos_quat.size() != 7) {
                throw std::invalid_argument("Each pose must have exactly 7 elements.");
            }
            T_init_final *= PosQuat2SE3(pos_quat);
        }
        return SE32PosQuat(T_init_final); // pos_quat_init_final
    }

    // Velocity adjoint maps
    static Eigen::MatrixXd Adjoint(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
        Eigen::MatrixXd adj = Eigen::MatrixXd::Identity(6, 6);
        Eigen::Matrix3d p_skew = R3Vec2so3Mat(p);
        adj.topLeftCorner<3, 3>() = R;
        adj.bottomRightCorner<3, 3>() = R;
        adj.topRightCorner<3, 3>() = p_skew * R;
        return adj;
    }

    static Eigen::VectorXd AdjointE2B(const Eigen::Matrix3d& R_b_e, const Eigen::Vector3d& p_offset, const Eigen::VectorXd& twist_e_e) {
        if (twist_e_e.size() != 6) {
            throw std::invalid_argument("The input twist must have exactly 6 elements.");
        }
        Eigen::MatrixXd adj_e2b = Adjoint(R_b_e, p_offset);
        return adj_e2b * twist_e_e; // 6x1
    }

    static Eigen::VectorXd AdjointB2E(const Eigen::Matrix3d& R_b_e, const Eigen::Vector3d& p_offset, const Eigen::VectorXd& twist_b_e) {
        if (twist_b_e.size() != 6) {
            throw std::invalid_argument("The input twist must have exactly 6 elements.");
        }
        Eigen::MatrixXd adj_b2e = Inv( Adjoint(R_b_e, p_offset) );
        return adj_b2e * twist_b_e; // 6x1
    }

    static Eigen::MatrixXd LeftPInv(const Eigen::MatrixXd& M) {
        return Inv( Transpose(M) * M ) * Transpose(M);
    }


    /* Pose preprocessing */
    static Eigen::VectorXd PosQuatOutlierRemoval(const Eigen::VectorXd &current_pos_quat, double std_thresh, std::deque<Eigen::VectorXd> &buffer, std::size_t window_size) {
        // Convert pos_quat to pos_so3
        Eigen::VectorXd current_pos_so3 = PosQuat2Posso3(current_pos_quat);

        // Check if buffer has enough data
        if (buffer.size() < window_size) {
            // Not enough data, accept current_pos_quat
            buffer.push_back(current_pos_so3);
            return current_pos_quat;
        } else {
            // Compute mean and std for positions and so3
            double dev_pos_norm = Norm( (current_pos_so3 - MeanBuffer(buffer)).head(3) );
            double dev_so3_norm = Norm( (current_pos_so3 - MeanBuffer(buffer)).tail(3));
            double std_pos_norm = Norm( StdBuffer(buffer).head(3) );
            double std_so3_norm = Norm( StdBuffer(buffer).tail(3) );

            // Check if deviation exceeds threshold times standard deviation
            bool is_outlier = false;
            if (std_pos_norm > 0 && dev_pos_norm > std_thresh * std_pos_norm) {
                is_outlier = true;
                std::cout << "dev_pos_norm = " << dev_pos_norm << ", std_pos_norm = " << std_pos_norm << std::endl;
            } else if (std_so3_norm > 0 && dev_so3_norm > std_thresh * std_so3_norm) {
                is_outlier = true;
                std::cout << "dev_so3_norm = " << dev_so3_norm << ", std_so3_norm = " << std_so3_norm << std::endl;
            }

            if (is_outlier) {
                // Outlier detected, return previous accepted value
                std::cout << "Outlier detected, rejecting current value." << std::endl;
                // Return previous value converted back to pos_quat
                Eigen::VectorXd previous_pos_so3 = buffer.back();
                return Posso32PosQuat(previous_pos_so3);
            } else {
                // Accept current value, update buffer
                buffer.push_back(current_pos_so3);
                // Keep buffer size within window_size
                if (buffer.size() > window_size) {
                    buffer.pop_front();
                }
                return current_pos_quat;
            }
        }
    }



    /* Robot controller functions */
    static std::pair<Eigen::VectorXd, bool> ErrorThreshold(const Eigen::VectorXd &error_norm_mavg, const Eigen::VectorXd &erro_norm_thresh, Eigen::VectorXd twist_cmd) {
        bool target_reached(false);
        if ((error_norm_mavg.array() <= erro_norm_thresh.array()).all()) {
            target_reached = true;
            twist_cmd = Eigen::VectorXd::Zero(6);  // Set twist_cmd to zero vector
        } else {
            target_reached = false;
        }

        return std::make_pair(twist_cmd, target_reached);
    }

    static Eigen::VectorXd SCurve(const Eigen::VectorXd &twist_cmd, const Eigen::VectorXd &twist_cmd_prev, double k, double t, double T)
    {   
        // twist_cmd: cmd
        // twist_cmd_prev: start value
        // k: steepness of the curve
        // t: time elapsed
        // T: total time for the curve to reach the final value
        if (twist_cmd.size() != twist_cmd_prev.size()) {
            throw std::invalid_argument("The input twist_cmd and twist_cmd_prev must have exactly 6 elements.");
        }
        if (k <= 0 || T <= 0) {
            throw std::invalid_argument("The k and T must be positive.");
        }
        return twist_cmd_prev + (twist_cmd - twist_cmd_prev) / (1 + exp( -k * (t - T / 2.0) ));
    }
    
    static Eigen::VectorXd KpPosso3(const Eigen::VectorXd& pos_so3_m_cmd, const Eigen::MatrixXd& kp_pos_so3, bool target_reached) {
        if (pos_so3_m_cmd.size() != 6) {
            throw std::invalid_argument("The input pose_error must have exactly 6 elements.");
        }        
        Eigen::VectorXd twist_cmd = kp_pos_so3 * pos_so3_m_cmd;
        if (target_reached) {
            twist_cmd.setZero();
        }
        return twist_cmd;
    }

    static Eigen::MatrixXd KpIBVSCircle2(const Eigen::Vector3d& circle_d, const Eigen::Vector3d& circle_o, double z_d, double z_est, const Eigen::Matrix3d& kp_ibvs, bool target_reached, Eigen::VectorXd& circle_error, Eigen::VectorXd& pos_error, double& error_norm) {
        pos_error = (Eigen::Vector3d() << circle_o.head<2>() - circle_d.head<2>(), z_est - z_d).finished();
        circle_error = circle_d - circle_o;
        error_norm = pos_error(2); // Assuming the third component is the error norm in this context

        if (target_reached) {
            circle_error.setZero();
        }

        Eigen::MatrixXd e_twist_cmd = Eigen::MatrixXd::Zero(6, 1);
        e_twist_cmd.topLeftCorner<3, 1>() = kp_ibvs * pos_error;
        return e_twist_cmd;
    }

};

#endif // RM_UTILS_HPP
