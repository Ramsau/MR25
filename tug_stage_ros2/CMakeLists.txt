cmake_minimum_required(VERSION 3.8)
project(tug_stage_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(stage REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tug_msgs REQUIRED)

# add executables
add_executable(tug_stage_ros2 "src/tug_stage_ros2.cpp" "src/StageNode.cpp")
target_include_directories(
  tug_stage_ros2
    PUBLIC "include/"
    PUBLIC ${STAGE_INCLUDE_DIRS}
)
target_link_libraries(tug_stage_ros2 ${STAGE_LIBRARIES})
target_compile_features(tug_stage_ros2 PUBLIC cxx_std_17)
ament_target_dependencies(
  tug_stage_ros2 
    rclcpp
    stage
    tf2
    tf2_ros
    geometry_msgs
    nav_msgs
    rosgraph_msgs
    sensor_msgs
    tf2_geometry_msgs
    tug_msgs
)

# install executables
install(TARGETS tug_stage_ros2 DESTINATION lib/${PROJECT_NAME})

# instal launch files
install(DIRECTORY "launch/" DESTINATION share/${PROJECT_NAME}/launch)

# install rviz files
install(DIRECTORY "rviz/" DESTINATION share/${PROJECT_NAME}/rviz)

# install worlds
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