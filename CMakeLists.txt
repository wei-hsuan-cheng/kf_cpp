cmake_minimum_required(VERSION 3.8)
project(kf_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(visualization_msgs REQUIRED)  # Added visualization_msgs

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Source files
set(EKF_NODE_SOURCES src/ekf_node.cpp)
set(EKF_ORI_SIM_SOURCES src/ekf_ori_sim.cpp)
set(UKF_NODE_SOURCES src/ukf_node.cpp)
set(UKF_ORI_SIM_SOURCES src/ukf_ori_sim.cpp)


# Add executables
add_executable(ekf_node ${EKF_NODE_SOURCES})
add_executable(ekf_ori_sim ${EKF_ORI_SIM_SOURCES})
add_executable(ukf_node ${UKF_NODE_SOURCES})
add_executable(ukf_ori_sim ${UKF_ORI_SIM_SOURCES})


# Link libraries
target_link_libraries(ekf_node ${PCL_LIBRARIES})
target_link_libraries(ekf_ori_sim ${PCL_LIBRARIES})
target_link_libraries(ukf_node ${PCL_LIBRARIES})
target_link_libraries(ukf_ori_sim ${PCL_LIBRARIES})


# Add dependencies
ament_target_dependencies(ekf_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  Eigen3
  visualization_msgs  # Added visualization_msgs dependency
)

ament_target_dependencies(ekf_ori_sim
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  Eigen3
  visualization_msgs  # Added visualization_msgs dependency
)

ament_target_dependencies(ukf_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  Eigen3
  visualization_msgs  # Added visualization_msgs dependency
)

ament_target_dependencies(ukf_ori_sim
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  Eigen3
  visualization_msgs  # Added visualization_msgs dependency
)


# Define installation rules
install(TARGETS
  ekf_node
  ekf_ori_sim
  ukf_node
  ukf_ori_sim


  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
