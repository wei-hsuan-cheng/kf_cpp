# kf_cpp
ROS 2 C++ implementation of Extended Kalman filter (EKF) and Unscented Kalman filter (UKF) with demo nodes.

## Table of Contents

- [kf_cpp](#kf_cpp)
  - [Table of Contents](#table-of-contents)
  - [Installation](#installation)
    - [Prerequisites](#prerequisites)
      - [Dependencies from `CMakeLists.txt`](#dependencies-from-cmakeliststxt)
  - [Acknowledgements](#acknowledgements)
  - [Contact](#contact)


## Installation

This repository is ROS 2-based and utilises several dependencies that are common in robotics applications. Follow the instructions below to build the package in a ROS 2 environment.

### Prerequisites

Make sure you have the following installed:

- **ROS 2 humble**: Installed and properly sourced. Follow the [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) if you haven't set it up yet.
- **C++17 Compiler**: Ensure your compiler supports C++17.
- **Dependencies**: The package depends on several libraries and ROS 2 packages.

#### Dependencies from `CMakeLists.txt`

- `Eigen3`
- ROS 2 packages:
  - `rclcpp`

Clone this repo into your ROS 2 workspace, install dependencies, and build pkg:

```bash
cd ~/ros2_ws/src && git clone https://github.com/wei-hsuan-cheng/kf_cpp.git

cd ~/ros2_ws && rosdep update && rosdep install --from-paths src --ignore-src -r -y

cd ~/ros2_ws && colcon build --packages-select kf_cpp && . install/setup.bash
```

Run the ROS 2 node for test:

```bash
cd ~/ros2_ws && . install/setup.bash
ros2 run kf_cpp ukf_ori_sim
```


## KF v.s. EKF v.s. UKF

The table below compares the process of each filter in each iteration.

$$\begin{array}{l|l|l|l}
\text{Process}                      &
\text{Kalman filter (KF)}           & 
\text{Extended Kalman filter (EKF)} & 
\text{Unscented Kalman filter (UKF)} \\

\hline 
& & & \\

\text{Prediction} &
\mathbf{\bar x} = \mathbf{Fx} + \mathbf{Bu} & 
\mathbf{\bar x} = f(\mathbf x, \mathbf u),\, \mathbf F = {\frac{\partial{f(\mathbf x_t, \mathbf u_t)}}{\partial{\mathbf x}}}\biggr|_{{\mathbf x_t},{\mathbf u_t}} & 
\mathbf{\bar x} = \sum w^m\boldsymbol{\mathcal Y},\, \boldsymbol{\mathcal Y} = f(\boldsymbol\chi) \\

\text{(Time update)} &
\mathbf{\bar P} = \mathbf{FPF}^\mathsf{T}+\mathbf Q  & 
\mathbf{\bar P} = \mathbf{FPF}^\mathsf{T}+\mathbf Q  &
\mathbf{\bar P} = \sum w^c(\boldsymbol{\mathcal Y} - \mathbf{\bar x})(\boldsymbol{\mathcal Y} - \mathbf{\bar x})^\mathsf T+\mathbf Q \\

\hline
& & & \\

& & & \boldsymbol\mu_z = \sum w^m\boldsymbol{\mathcal{Z}}, \boldsymbol{\mathcal Z} =  h(\boldsymbol{\mathcal{Y}}) \\

\text{Update} &
\textbf{y} = \mathbf z - \mathbf{H \bar{x}} & 
\textbf{y} = \mathbf z - h(\bar{x}),\, \mathbf H = \frac{\partial{h(\bar{\mathbf x}_t)}}{\partial{\bar{\mathbf x}}}\biggr|_{\bar{\mathbf x}_t} &
\mathbf y = \mathbf z - \boldsymbol\mu_z \\

\text{(Measurement update)} &
\mathbf S = \mathbf{H\bar PH}^\mathsf{T} + \mathbf R & 
\mathbf S = \mathbf{H\bar PH}^\mathsf{T} + \mathbf R & 
\mathbf P_z = \sum w^c{(\boldsymbol{\mathcal Z}-\boldsymbol\mu_z)(\boldsymbol{\mathcal{Z}}-\boldsymbol\mu_z)^\mathsf{T}} + \mathbf R \\

& 
\mathbf K = \mathbf{\bar PH}^\mathsf T \mathbf S^{-1} & 
\mathbf K = \mathbf{\bar PH}^\mathsf T \mathbf S^{-1} &
\mathbf K = \left[\sum w^c(\boldsymbol{\mathcal Y}-\bar{\mathbf x})(\boldsymbol{\mathcal{Z}}-\boldsymbol\mu_z)^\mathsf{T}\right] \mathbf P_z^{-1} \\

&
\mathbf x = \mathbf{\bar x} + \mathbf{Ky} & 
\mathbf x = \mathbf{\bar x} + \mathbf{Ky} &
\mathbf x = \mathbf{\bar x} + \mathbf{Ky} \\

&
\mathbf P= (\mathbf{I}-\mathbf{KH})\mathbf{\bar{P}} & 
\mathbf P= (\mathbf{I}-\mathbf{KH})\mathbf{\bar{P}} &
\mathbf P = \bar{\mathbf P} - \mathbf{KP_z}\mathbf{K}^\mathsf{T}
\end{array}$$


## Bibliography
- EKF:
  - [1]
- UKF:
  - [1] Rudolph Van der Merwe. "Sigma-Point Kalman Filters for Probabilistic Inference in Dynamic State-Space Models" dissertation (2004).

  - [2] Simon J. Julier. "The Scaled Unscented Transformation". Proceedings of the American Control Conference 6. IEEE. (2002)

  - [3] http://www.esdradar.com/brochures/Compact%20Tracking%2037250X.pdf

  - [4] Julier, Simon J.; Uhlmann, Jeffrey "A New Extension of the Kalman  Filter to Nonlinear Systems". Proc. SPIE 3068, Signal Processing, Sensor Fusion, and Target Recognition VI, 182 (July 28, 1997)

  - [5] Cholesky decomposition. Wikipedia. http://en.wikipedia.org/wiki/Cholesky_decomposition

## Acknowledgements

- **Kalman-and-Bayesian-Filters-in-Python**: The EKF algorithm is an C++ implementation adapted from the [Kalman-and-Bayesian-Filters-in-Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python).
- **hdl-localization-ROS2**: The UKF algorithm is adapted from the [hdl-localization-ROS2 codebase](https://github.com/pyc5714/hdl-localization-ROS2/blob/35de917029371c4de93fc8107ad25a09cca7b238/hdl_localization/include/kkl/alg/unscented_kalman_filter.hpp#L241).
- **Robot Math Utils**: Some utilities from [Robot Math Utils](https://github.com/wei-hsuan-cheng/robot_math_utils) is used.
- **Eigen Library**: This library heavily relies on the Eigen library for linear algebra operations.

## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)


