#ifndef RM_UTILS_HPP
#define RM_UTILS_HPP

// Author: Wei-Hsuan Cheng, johnathancheng0125@gmail.com, https://github.com/wei-hsuan-cheng
// ver_1.1, last edit: 241004
// Some functions are adapted from the Modern Robotics book codebase: https://github.com/Le0nX/ModernRoboticsCpp/tree/eacdf8800bc591b03727512c102d2c5dffe78cec

#include <Eigen/Dense>
#include <Eigen/Geometry> // For Quaternion
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
    /* Unit conversion */
    static constexpr double r2d = 180.0 / M_PI;
    static constexpr double d2r = M_PI / 180.0;

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

    static void SaveMat(const Eigen::MatrixXd& mat, const std::string& filename) {
        std::ofstream file(filename);
        if (file.is_open()) {
            // Optional: Set precision if needed
            // file << std::fixed << std::setprecision(6);
            for (int i = 0; i < mat.rows(); ++i) {
                for (int j = 0; j < mat.cols(); ++j) {
                    file << mat(i, j);
                    if (j < mat.cols() - 1) {
                        file << ","; // Add a comma unless it's the last column
                    }
                }
                file << "\n"; // Newline after each row
            }
            file.close();
        } else {
            std::cerr << "Could not open file " << filename << " for writing." << std::endl;
        }
    }



    /* Numerical conditions */
    static bool NearZero(const double val, double thresh = pow(10, -7)) {
		return (std::abs(val) < thresh);
	}

    static double ConstrainedAngle(double angle, bool rad = true) {
        // Normalize angle to (-pi, pi] or (-180, 180]
        double full_circle = rad ? 2.0 * M_PI : 360.0;
        double half_circle = rad ? M_PI : 180.0;
        // Use modulo operation to wrap angle within (-full_circle, full_circle)
        angle = std::fmod(angle, full_circle);
        // Adjust angle to be within (-half_circle, half_circle]
        if (angle <= -half_circle) {
            angle += full_circle;
        } else if (angle > half_circle) {
            angle -= full_circle;
        }
        return angle;
    }


    /* Sliding window functions */
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

    static double ArcCos(double cos_val, bool rad = true) {
        if (cos_val > 1.0 || cos_val < -1.0) {
            throw std::invalid_argument("The input cosine value must be within [-1, 1].");
        }
        double theta = rad ? std::acos(cos_val) : std::acos(cos_val) * r2d;
        return theta; // std::acos() guarantees theta lies in [0, pi] [rad] or [0, 180] [deg]
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

    static Eigen::MatrixXd Transpose(const Eigen::MatrixXd& M) {
        return M.transpose();
    }

    static double Tr(const Eigen::MatrixXd& M) {
        return M.trace();
    }

    static double Det(const Eigen::MatrixXd& M) {
        return M.determinant();
    }

    static Eigen::MatrixXd Inv(const Eigen::MatrixXd& M) {
        return M.inverse();
    }

    static Eigen::MatrixXd LeftPInv(const Eigen::MatrixXd& M) {
        return Inv( Transpose(M) * M ) * Transpose(M);
    }


    // Random variables
    static double RandNorDist(double mean = 0.0, double stddev = 1.0) {
        static std::mt19937 rng(std::random_device{}()); // Standard mersenne_twister_engine
        std::normal_distribution<double> dist(mean, stddev);
        return dist(rng);
    }

    static Eigen::VectorXd RandNorDistVec(const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov) {
        static std::mt19937 rng(std::random_device{}());
        std::normal_distribution<double> dist;
        if (mean.size() != cov.rows() || cov.rows() != cov.cols()) {
            throw std::invalid_argument("Mean vector size and covariance matrix dimensions must match.");
        }
        // Perform Cholesky decomposition (LLT) of the covariance matrix
        Eigen::LLT<Eigen::MatrixXd> lltOfCov(cov);
        if (lltOfCov.info() == Eigen::NumericalIssue) {
            throw std::runtime_error("Covariance matrix is not positive definite.");
        }
        Eigen::MatrixXd L = lltOfCov.matrixL();
        // Generate a vector of standard normal random variables
        Eigen::VectorXd z = Eigen::VectorXd::NullaryExpr(mean.size(), [&]() { return dist(rng); });
        return mean + L * z;
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
    

    // SO(3) and so(3) functions (quaterions as main representation)
    // Quaternion operations
    static Eigen::Vector4d QuatMul(const Eigen::Vector4d& q, const Eigen::Vector4d& p) {
        Eigen::Quaterniond qp = Eigen::Quaterniond(q(0), q(1), q(2), q(3)) * Eigen::Quaterniond(p(0), p(1), p(2), p(3));
        return Eigen::Vector4d(qp.w(), qp.x(), qp.y(), qp.z());
    }

    static Eigen::Vector4d ConjQuat(const Eigen::Vector4d& quat) {
        if (quat.size() != 4) {
            throw std::invalid_argument("The input quaternion must have exactly 4 elements.");
        }
        return Eigen::Vector4d(quat(0), -quat(1), -quat(2), -quat(3));
    }

    static Eigen::Vector4d InvQuat(const Eigen::Vector4d& quat) {
        if (quat.size() != 4) {
            throw std::invalid_argument("The input quaternion must have exactly 4 elements.");
        }
        return ConjQuat(quat);
    }

    static Eigen::Vector4d TransformQuats(const std::vector<Eigen::Vector4d>& quats) {
        if (quats.size() == 0) {
            throw std::invalid_argument("The input list of quaternions is empty.");
        }
        Eigen::Vector4d quat_init_final = quats[0];
        for (size_t i = 1; i < quats.size(); ++i) {
            quat_init_final = QuatMul(quat_init_final, quats[i]);
        }
        return quat_init_final;
    }

    // Exp and Log maps in SO(3)
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

    static Eigen::Vector3d Quat2so3(const Eigen::Vector4d& quat) {
        if (quat.size() != 4) {
            throw std::invalid_argument("The input quaternion must have exactly 4 elements.");
        }
        double theta = 2 * ArcCos(quat(0), true); 
        return (2 / Sinc(theta / 2)) * quat.tail(3);
    }

    static Eigen::Vector4d so32Quat(const Eigen::Vector3d& so3) {
        Eigen::Vector4d v;
        Eigen::Vector4d axis_ang = AxisAng3(so3);
        v << std::cos(axis_ang(3) / 2), std::sin(axis_ang(3) / 2) * axis_ang.head(3);
        return v;
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
    
    // Rotation matrices
    static Eigen::Matrix3d ThreeAxes2Rot(const Eigen::Vector3d& x_1_2, const Eigen::Vector3d& y_1_2, const Eigen::Vector3d& z_1_2) {
        Eigen::Matrix3d R_1_2;
        R_1_2.col(0) = x_1_2;
        R_1_2.col(1) = y_1_2;
        R_1_2.col(2) = z_1_2;
        return R_1_2;
    }

    static Eigen::Matrix3d so32Rot(const Eigen::Vector3d& so3) {
        return MatrixExp3( R3Vec2so3Mat(so3) );
    }

    static Eigen::Vector3d Rot2so3(const Eigen::Matrix3d& R) {
        return so3Mat2R3Vec( MatrixLog3(R) );
    }

    static Eigen::Matrix3d Quat2Rot(const Eigen::Vector4d& quat) {
        return so32Rot( Quat2so3(quat) );
    }

    static Eigen::Vector4d Rot2Quat(const Eigen::Matrix3d& R) {
        return so32Quat( Rot2so3(R) );
    }

    // ZYX Euler angles
    static Eigen::Matrix3d Rotx(double thx, bool rad = true) {
        if (!rad) {
            thx *= d2r;
        }
        return so32Rot(Eigen::Vector3d(thx, 0, 0));
    }

    static Eigen::Matrix3d Roty(double thy, bool rad = true) {
        if (!rad) {
            thy *= d2r;
        }
        return so32Rot(Eigen::Vector3d(0, thy, 0));
    }

    static Eigen::Matrix3d Rotz(double thz, bool rad = true) {
        if (!rad) {
            thz *= d2r;
        }
        return so32Rot(Eigen::Vector3d(0, 0, thz));
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
            thx *= r2d;
            thy *= r2d;
            thz *= r2d;
        }
        return Eigen::Vector3d(thz, thy, thx);
    }

    static Eigen::Vector4d Quatx(double thx, bool rad = true) {
        if (!rad) {
            thx *= d2r;
        }
        return so32Quat(Eigen::Vector3d(thx, 0, 0));
    }

    static Eigen::Vector4d Quaty(double thy, bool rad = true) {
        if (!rad) {
            thy *= d2r;
        }
        return so32Quat(Eigen::Vector3d(0, thy, 0));
    }

    static Eigen::Vector4d Quatz(double thz, bool rad = true) {
        if (!rad) {
            thz *= d2r;
        }
        return so32Quat(Eigen::Vector3d(0, 0, thz));
    }

    static Eigen::Vector4d xyzEuler2Quat(const Eigen::Vector3d& xyz_euler, bool rad = true) {
        Eigen::Vector4d qx = Quatx(xyz_euler(0), rad);
        Eigen::Vector4d qy = Quaty(xyz_euler(1), rad);
        Eigen::Vector4d qz = Quatz(xyz_euler(2), rad);
        return TransformQuats({qx, qy, qz});
    }
    
    static Eigen::Vector4d zyxEuler2Quat(const Eigen::Vector3d& zyx_euler, bool rad = true) {
        Eigen::Vector4d qz = Quatz(zyx_euler(0), rad);
        Eigen::Vector4d qy = Quaty(zyx_euler(1), rad);
        Eigen::Vector4d qx = Quatx(zyx_euler(2), rad);
        return TransformQuats({qz, qy, qx});
    }

    static Eigen::Vector3d Quat2zyxEuler(const Eigen::Vector4d& quat, bool rad = true) {
        Eigen::Matrix3d R = Quat2Rot(quat);
        return Rot2zyxEuler(R, rad);
    }

    // Rotate vectors
    static Eigen::Vector3d RotateR3VecFromQuat(const Eigen::Vector3d& r3_vec, const Eigen::Vector4d& quat) {
        return Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)) * r3_vec;
    }



    // SE(3) and se(3) functions
    // Exp and Log maps in SE(3)
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

    // Core conversions (quaternions to others)
    // pos_quat <-> r6_pose 
    static Eigen::VectorXd PosQuat2R6Pose(const Eigen::VectorXd& pos_quat_1_2) {
        if (pos_quat_1_2.size() != 7) {
            throw std::invalid_argument("The input vector must have exactly 7 elements.");
        }
        Eigen::VectorXd pose_1_2(6);
        pose_1_2.head<3>() = pos_quat_1_2.head<3>();
        pose_1_2.tail<3>() = Rot2zyxEuler( Quat2Rot( pos_quat_1_2.tail<4>() ), true ).reverse();
        return pose_1_2;
    }
    
    static Eigen::VectorXd R6Pose2PosQuat(const Eigen::VectorXd& pose_1_2) {
        if (pose_1_2.size() != 6) {
            throw std::invalid_argument("The input pose must have exactly 6 elements.");
        }
        Eigen::VectorXd pos_quat_1_2(7);
        pos_quat_1_2.head<3>() = pose_1_2.head<3>();
        pos_quat_1_2.tail<4>() = zyxEuler2Quat(pose_1_2.tail<3>().reverse(), true);
        return pos_quat_1_2;
    }

    // pos_quat <-> pos_so3
    static Eigen::VectorXd PosQuat2Posso3(const Eigen::VectorXd& pos_quat_1_2) {
        if (pos_quat_1_2.size() != 7) {
            throw std::invalid_argument("The input vector must have exactly 7 elements.");
        }
        Eigen::VectorXd pos_so3_1_2(6);
        pos_so3_1_2.head<3>() = pos_quat_1_2.head<3>();
        pos_so3_1_2.tail<3>() = Quat2so3( pos_quat_1_2.tail<4>() );
        return pos_so3_1_2;
    }

    static Eigen::VectorXd Posso32PosQuat(const Eigen::VectorXd& pos_so3_1_2) {
        if (pos_so3_1_2.size() != 6) {
            throw std::invalid_argument("The input vector must have exactly 6 elements.");
        }
        Eigen::VectorXd pos_quat_1_2(7);
        pos_quat_1_2.head<3>() = pos_so3_1_2.head<3>();
        pos_quat_1_2.tail<4>() = so32Quat( pos_so3_1_2.tail<3>() );
        return pos_quat_1_2;
    }

    // pos_quat <-> SE(3) matrix
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


    // Other conversions
    // rot_pos <-> SE(3) matrix
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


    // r6_pose <-> SE(3) matrix
    static Eigen::Matrix4d R6Pose2SE3(const Eigen::VectorXd& pose_1_2) {
        return PosQuat2SE3( R6Pose2PosQuat(pose_1_2) );
    }

    static Eigen::VectorXd SE32R6Pose(const Eigen::Matrix4d& T_1_2) {
        return PosQuat2R6Pose( SE32PosQuat(T_1_2) );
    }

    // r6_pose <-> rot_pos
    static std::pair<Eigen::Matrix3d, Eigen::Vector3d> R6Pose2RotPos(const Eigen::VectorXd& pose_1_2) {
        Eigen::Matrix4d T_1_2 = R6Pose2SE3(pose_1_2);
        Eigen::Matrix3d R_1_2 = T_1_2.topLeftCorner<3, 3>();
        Eigen::Vector3d p_1_2 = T_1_2.topRightCorner<3, 1>();
        return std::make_pair(R_1_2, p_1_2);
    }

    static Eigen::VectorXd RotPos2R6Pose(const Eigen::Matrix3d& R_1_2, const Eigen::Vector3d& p_1_2) {
        return PosQuat2R6Pose( SE32PosQuat( RotPos2SE3(R_1_2, p_1_2) ) );
    }


    // SE3 matrix <-> pos_so3
    static Eigen::VectorXd SE32Posso3(const Eigen::Matrix4d& T_1_2) {
        return PosQuat2Posso3( SE32PosQuat(T_1_2) );
    }

    static Eigen::Matrix4d Posso32SE3(const Eigen::VectorXd& pos_so3_1_2) {
        return PosQuat2SE3( Posso32PosQuat(pos_so3_1_2) );
    }


    // Inverse transformations
    static Eigen::VectorXd InvPosQuat(const Eigen::VectorXd& pos_quat_1_2) {
        if (pos_quat_1_2.size() != 7) {
            throw std::invalid_argument("The input vector must have exactly 7 elements.");
        }
        Eigen::VectorXd pos_quat_2_1(7);
        pos_quat_2_1 << -RotateR3VecFromQuat( pos_quat_1_2.head<3>(), InvQuat(pos_quat_1_2.tail<4>()) ), InvQuat(pos_quat_1_2.tail<4>());
        return pos_quat_2_1;
    }

    static Eigen::VectorXd InvR6Pose(const Eigen::VectorXd& pose_1_2) {
        if (pose_1_2.size() != 6) {
            throw std::invalid_argument("The input pose must have exactly 6 elements.");
        }
        return PosQuat2R6Pose( InvPosQuat( R6Pose2PosQuat(pose_1_2) ) );
    }


    // Transform poses and relative poses
    // pos_quats
    static Eigen::VectorXd TransformPosQuat(const Eigen::VectorXd& pos_quat_b_1, const Eigen::VectorXd& pos_quat_1_2) {
        if (pos_quat_b_1.size() != 7 || pos_quat_1_2.size() != 7) {
            throw std::invalid_argument("Each pose must have exactly 7 elements.");
        }
        Eigen::Vector4d quat_b_2 = QuatMul(pos_quat_b_1.tail<4>(), pos_quat_1_2.tail<4>());
        Eigen::Vector3d p_b_2 = RotateR3VecFromQuat(pos_quat_1_2.head<3>(), pos_quat_b_1.tail<4>()) + pos_quat_b_1.head<3>();
        Eigen::VectorXd pos_quat_b_2(7);
        pos_quat_b_2 << p_b_2, quat_b_2;
        return pos_quat_b_2;
    }

    static Eigen::VectorXd TransformPosQuats(const std::vector<Eigen::VectorXd>& pos_quats) {
        if (pos_quats.size() == 0) {
            throw std::invalid_argument("The input list of poses is empty.");
        }
        Eigen::VectorXd pos_quat_init_final = pos_quats[0]; // Initial pos_quat
        for (std::size_t i = 1; i < pos_quats.size(); ++i) {
            pos_quat_init_final = TransformPosQuat(pos_quat_init_final, pos_quats[i]);
        }
        return pos_quat_init_final; 
    }

    static Eigen::VectorXd PosQuats2RelativePosQuat(const Eigen::VectorXd& pos_quat_b_1, const Eigen::VectorXd& pos_quat_b_2) {
        if (pos_quat_b_1.size() != 7 || pos_quat_b_2.size() != 7) {
            throw std::invalid_argument("Each pose must have exactly 7 elements.");
        }
        return TransformPosQuat(InvPosQuat(pos_quat_b_1), pos_quat_b_2); // pos_quat_1_2
    }


    // SE3 matrices
    static Eigen::MatrixXd TransformSE3(const Eigen::MatrixXd& SE3_1_2, const Eigen::MatrixXd& SE3_2_3) {
        if (SE3_1_2.rows() != 4 || SE3_1_2.cols() != 4 || SE3_2_3.rows() != 4 || SE3_2_3.cols() != 4) {
            throw std::invalid_argument("Each SE3 matrix must be 4x4.");
        }
        return SE3_1_2 * SE3_2_3;
    }

    static Eigen::MatrixXd TransformSE3s(const std::vector<Eigen::MatrixXd>& SE3s) {
        if (SE3s.empty()) {
            throw std::invalid_argument("Input vector of SE3 matrices is empty.");
        } else if (SE3s[0].rows() != 4 || SE3s[0].cols() != 4) {
            throw std::invalid_argument("Each SE3 matrix must be 4x4.");
        }
        Eigen::MatrixXd result = SE3s[0];
        for (size_t i = 1; i < SE3s.size(); ++i) {
            result = TransformSE3(result, SE3s[i]);
        }
        return result;
    }


    // 6D poses
    static Eigen::VectorXd TransformR6Pose(const Eigen::VectorXd& pose_b_1, const Eigen::VectorXd& pose_1_2) {
        if (pose_b_1.size() != 6 || pose_1_2.size() != 6) {
            throw std::invalid_argument("Each pose must have exactly 6 elements.");
        }
        return PosQuat2R6Pose( TransformPosQuat( R6Pose2PosQuat(pose_b_1), R6Pose2PosQuat(pose_1_2) ) ); // pose_b_2
    }

    static Eigen::VectorXd TransformR6Poses(const std::vector<Eigen::VectorXd>& poses) {
        if (poses.size() == 0) {
            throw std::invalid_argument("The input list of poses is empty.");
        }
        Eigen::VectorXd pose_init_final = poses[0]; // Initial pose
        for (std::size_t i = 1; i < poses.size(); ++i) {
            pose_init_final = TransformR6Pose(pose_init_final, poses[i]);
        }
        return pose_init_final;
    }

    static Eigen::VectorXd R6Poses2RelativeR6Pose(const Eigen::VectorXd& pose_b_1, const Eigen::VectorXd& pose_b_2) {
        if (pose_b_1.size() != 6 || pose_b_2.size() != 6) {
            throw std::invalid_argument("Each pose must have exactly 6 elements.");
        }
        return TransformR6Pose(InvR6Pose(pose_b_1), pose_b_2); // pose_1_2
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



    /* Motion mapping */
    static Eigen::VectorXd AxisDecoupling(Eigen::VectorXd joy_input) {
        double max_val = 0.68;
        Eigen::VectorXd thres_percent(6);
        thres_percent << 0.45, 0.45, 0.55, 0.90, 0.90, 0.90; // To be tuned according needs

        Eigen::VectorXd out_thresh = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd joy_dec = Eigen::VectorXd::Zero(6);

        double max_temp = 0;
        int max_id = -1;
        for (int i = 0; i < joy_input.size(); i++)
        {
            double sig = joy_input(i);
            // Check threshold
            if (std::abs(sig) >= (thres_percent(i) * max_val))
            {
                out_thresh(i) = sig > 0 ? 1 : -1; // 1 if positive, -1 if negative
                if ((std::abs(sig) > max_temp) && !((i == 2 && max_id != -1) || (max_id == 2)))
                {
                    max_temp = std::abs(sig);
                    max_id = i;
                }
            }
        }

        if (max_id != -1)
        {
            joy_dec(max_id) = out_thresh(max_id);
        }
        return joy_dec;
    }

    static Eigen::VectorXd VelMapping(Eigen::VectorXd joy_input, Eigen::MatrixXd vel_scale = Eigen::MatrixXd::Identity(6, 6)) {
        return vel_scale * joy_input; // vx, vy, vz, wx, wy, wz [m/s, rad/s]
    }



    /* Motion planning */
    // S-curve velocity smoothing
    static Eigen::VectorXd SCurve(const Eigen::VectorXd &twist_cmd, const Eigen::VectorXd &twist_cmd_prev, double lambda, double t, double T)
    {   
        // twist_cmd: cmd
        // twist_cmd_prev: start value
        // lambda: steepness of the curve (determines max. acc.)
        // t: time elapsed
        // T: total time for the curve to reach the final value
        if (twist_cmd.size() != twist_cmd_prev.size()) {
            throw std::invalid_argument("The input twist_cmd and twist_cmd_prev must have exactly 6 elements.");
        }
        if (lambda <= 0 || T <= 0) {
            throw std::invalid_argument("The lambda and T must be positive.");
        }
        return twist_cmd_prev + (twist_cmd - twist_cmd_prev) / (1 + exp( -lambda * (t - T / 2.0) ));
    }


    // Screw motion path generation (only waypoints)
    static std::vector<Eigen::VectorXd> ScrewMotionPath(const Eigen::VectorXd& pos_quat_b_e, const Eigen::VectorXd& pos_quat_e_e_cmd, int N) {
        std::vector<Eigen::VectorXd> waypoints; // Output waypoints
        if (pos_quat_b_e.size() != 7 || pos_quat_e_e_cmd.size() != 7)
        {
            throw std::invalid_argument("Each pos_quat must have exactly 7 elements.");
        }
        // Convert pos_quat_e_e_cmd to pos_so3_e_e_cmd
        Eigen::VectorXd pos_so3_e_e_cmd = PosQuat2Posso3(pos_quat_e_e_cmd);
        // Generate waypoints
        for (int i = 0; i < N; ++i)
        {
            double alpha = static_cast<double>(i + 1) / N;
            // Intermediate pos_so3
            Eigen::VectorXd pos_so3_e_e_cmd_i = alpha * pos_so3_e_e_cmd;
            // Compute pos_so3 back to pos_quat
            Eigen::VectorXd pos_quat_e_e_cmd_i = Posso32PosQuat(pos_so3_e_e_cmd_i);
            // Transform to base frame
            Eigen::VectorXd pos_quat_b_e_d_i = TransformPosQuat(pos_quat_b_e, pos_quat_e_e_cmd_i);
            // Append to waypoints
            waypoints.push_back(pos_quat_b_e_d_i);
        }
        return waypoints;
    }


    // Screw motion trajectory generation (waypoints and timestamps)
    static std::pair<std::vector<Eigen::VectorXd>, std::vector<double>> ScrewMotionTraj(const Eigen::VectorXd& pos_quat_b_e, const Eigen::VectorXd& pos_quat_e_e_cmd, int N, double T) {
        std::vector<Eigen::VectorXd> waypoints; // Output waypoints
        std::vector<double> timestamps; // Output timestamps
        if (pos_quat_b_e.size() != 7 || pos_quat_e_e_cmd.size() != 7)
        {
            throw std::invalid_argument("Each pos_quat must have exactly 7 elements.");
        }
        // Convert pos_quat_e_e_cmd to pos_so3_e_e_cmd
        Eigen::VectorXd pos_so3_e_e_cmd = PosQuat2Posso3(pos_quat_e_e_cmd);
        // Generate waypoints
        for (int i = 0; i < N; ++i)
        {
            double alpha = static_cast<double>(i + 1) / N;
            // Intermediate pos_so3
            Eigen::VectorXd pos_so3_e_e_cmd_i = alpha * pos_so3_e_e_cmd;
            // Compute pos_so3 back to pos_quat
            Eigen::VectorXd pos_quat_e_e_cmd_i = Posso32PosQuat(pos_so3_e_e_cmd_i);
            // Transform to base frame
            Eigen::VectorXd pos_quat_b_e_d_i = TransformPosQuat(pos_quat_b_e, pos_quat_e_e_cmd_i);
            // Append to waypoints
            waypoints.push_back(pos_quat_b_e_d_i);
            // Append timestamps
            timestamps.push_back(alpha * T);
        }
        return std::make_pair(waypoints, timestamps);
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

    
    // Cartesian kinematic control
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

    // static Eigen::MatrixXd KpIBVSCircle2(const Eigen::Vector3d& circle_d, const Eigen::Vector3d& circle_o, double z_d, double z_est, const Eigen::Matrix3d& kp_ibvs, bool target_reached, Eigen::VectorXd& circle_error, Eigen::VectorXd& pos_error, double& error_norm) {
    //     pos_error = (Eigen::Vector3d() << circle_o.head<2>() - circle_d.head<2>(), z_est - z_d).finished();
    //     circle_error = circle_d - circle_o;
    //     error_norm = pos_error(2); // Assuming the third component is the error norm in this context

    //     if (target_reached) {
    //         circle_error.setZero();
    //     }

    //     Eigen::MatrixXd e_twist_cmd = Eigen::MatrixXd::Zero(6, 1);
    //     e_twist_cmd.topLeftCorner<3, 1>() = kp_ibvs * pos_error;
    //     return e_twist_cmd;
    // }

};

#endif // RM_UTILS_HPP
