cmake_minimum_required(VERSION 3.8)
project(tug_turtlebot4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# find message dependencies
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tug_msgs REQUIRED)

#find ignition dependencies
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

# add Gazebo library
add_library(tug_wheel_encoder-system SHARED "src/ign_plugin/WheelEncoder.cpp")
target_compile_features(tug_wheel_encoder-system PUBLIC cxx_std_17)
target_include_directories(tug_wheel_encoder-system PUBLIC "include/")
target_link_libraries(
  tug_wheel_encoder-system
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::ignition-plugin1
)
ament_target_dependencies(tug_wheel_encoder-system rclcpp tug_msgs)

# add nodes
add_executable(simple_turtle "src/simple_turtle.cpp" "src/SimpleTurtleNode.cpp")
target_compile_features(simple_turtle PUBLIC cxx_std_17)
target_include_directories(simple_turtle PUBLIC "include/")
ament_target_dependencies(
  simple_turtle
    rclcpp
    tf2
    tf2_ros
    geometry_msgs
    tf2_geometry_msgs
    tug_msgs
)

add_executable(slam "src/slam.cpp" "src/SlamNode.cpp" "src/OccupancyGridMap.cpp")
target_compile_features(slam PUBLIC cxx_std_17)
target_include_directories(slam PUBLIC "include/")
ament_target_dependencies(
  slam
    rclcpp
    tf2
    tf2_ros
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2_geometry_msgs
)

add_executable(static_transform_publisher "src/static_transform_publisher.cpp")
target_compile_features(static_transform_publisher PUBLIC cxx_std_17)
target_include_directories(static_transform_publisher PUBLIC "include/")
ament_target_dependencies(
  static_transform_publisher
    rclcpp
    tf2_ros
    geometry_msgs
)

# install targets
install(
  TARGETS
    tug_wheel_encoder-system
    simple_turtle
    slam
    static_transform_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# add config directory
install(DIRECTORY "config/" DESTINATION share/${PROJECT_NAME}/config)

# add launch directory
install(DIRECTORY "launch/" DESTINATION share/${PROJECT_NAME}/launch)

# add rviz directory
install(DIRECTORY "rviz/" DESTINATION share/${PROJECT_NAME}/rviz)

# add urdf directory
install(DIRECTORY "urdf/" DESTINATION share/${PROJECT_NAME}/urdf)

# add world directory
install(DIRECTORY "worlds/" DESTINATION share/${PROJECT_NAME}/worlds)

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
