cmake_minimum_required(VERSION 3.8)
project(tug_mr2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tug_msgs REQUIRED)

# add executables
add_executable(tug_kinematic "src/tug_kinematic.cpp" "src/KinematicNode.cpp")
target_include_directories(tug_kinematic PUBLIC "include/")
target_compile_features(tug_kinematic PUBLIC cxx_std_17)
ament_target_dependencies(
  tug_kinematic
    rclcpp
    geometry_msgs
    tug_msgs
)

add_executable(tug_transform "src/tug_transform.cpp" "src/TransformNode.cpp")
target_include_directories(tug_transform PUBLIC "include/")
target_compile_features(tug_transform PUBLIC cxx_std_17)
ament_target_dependencies(
  tug_transform
    rclcpp
    nav_msgs
    sensor_msgs
)

# install executables
install(
  TARGETS
    tug_kinematic
    tug_transform
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY "launch/" DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
