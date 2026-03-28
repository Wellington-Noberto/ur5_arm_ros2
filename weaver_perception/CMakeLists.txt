cmake_minimum_required(VERSION 3.8)
project(weaver_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(apriltag REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

add_executable(marker_detector
    src/marker_detector.cpp)
  add_executable(pose_evaluator
    src/pose_evaluator.cpp)

ament_target_dependencies(marker_detector
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  apriltag
  tf2_ros
  tf2_eigen
)

ament_target_dependencies(pose_evaluator
  rclcpp
  tf2_ros
  tf2_eigen
)

target_link_libraries(marker_detector
  apriltag
)

install(DIRECTORY  src
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  marker_detector
  pose_evaluator
  DESTINATION lib/${PROJECT_NAME})


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
