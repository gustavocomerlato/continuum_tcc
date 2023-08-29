cmake_minimum_required(VERSION 3.8)
project(trajectory_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)

add_executable(pose_trajectory_publisher src/pose_trajectory_publisher.cpp)
target_include_directories(pose_trajectory_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pose_trajectory_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pose_trajectory_publisher
  "rclcpp"
  "tf2_kdl"
  "geometry_msgs"
  "trajectory_msgs"
  "orocos_kdl"
)

install(TARGETS pose_trajectory_publisher
  DESTINATION lib/${PROJECT_NAME})

# add_executable(joint_trajectory_publisher src/joint_trajectory_publisher.cpp)
# target_include_directories(joint_trajectory_publisher PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(joint_trajectory_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   joint_trajectory_publisher
#   "rclcpp"
#   "tf2_kdl"
#   "geometry_msgs"
#   "trajectory_msgs"
#   "orocos_kdl"
# )

# install(TARGETS joint_trajectory_publisher
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
