cmake_minimum_required(VERSION 3.8)
project(continuum_manipulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(continuum_msgs REQUIRED)

# executaveis
add_executable(continuum_section src/continuum_section.cpp)
target_include_directories(continuum_section PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(continuum_section PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
ament_target_dependencies(
  continuum_section
  "rclcpp"
  "orocos_kdl"
  "kdl_parser"
  "Eigen3"
  "tf2"
  "tf2_ros"
  "tf2_eigen"
  "tf2_geometry_msgs"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "continuum_msgs"
)

add_executable(continuum_robot src/continuum_robot.cpp)
target_include_directories(continuum_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(continuum_robot PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
ament_target_dependencies(
  continuum_robot
  "rclcpp"
  "orocos_kdl"
  "kdl_parser"
  "Eigen3"
  "tf2"
  "tf2_ros"
  "tf2_eigen"
  "tf2_geometry_msgs"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "visualization_msgs"
)

add_executable(cable_untangler src/cable_untangler.cpp)
target_include_directories(cable_untangler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cable_untangler PUBLIC c_std_99 cxx_std_20)
ament_target_dependencies(
  cable_untangler
  "rclcpp"
  "orocos_kdl"
  "Eigen3"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "continuum_msgs"
  "visualization_msgs"
)

install(TARGETS continuum_section continuum_robot muxer cable_untangler
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz scripts
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
