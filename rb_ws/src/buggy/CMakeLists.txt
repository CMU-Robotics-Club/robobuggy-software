cmake_minimum_required(VERSION 3.8)
project(buggy)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Install Launch Files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/hello_world.py
  scripts/controller/controller_node.py
  scripts/path_planner/path_planner.py
  scripts/simulator/engine.py
  scripts/simulator/radio_sim.py
  scripts/simulator/velocity_updater.py
  scripts/buggy_state_converter.py
  scripts/serial/ros_to_bnyahaj.py
  scripts/debug/debug_steer.py
  scripts/vision/detector_node.py
  scripts/debug/set_steer.py
  scripts/debug/sim_metrics.py
  scripts/debug/pass_metrics.py
  scripts/path_planner/raceline_optimizer.py
  scripts/path_planner/frenet_planner.py
  scripts/perception/lidar_opponent_node.py
  scripts/perception/opponent_tracker.py
  scripts/lidar/buggy_lidar.py
  scripts/util/bag_to_course.py
  scripts/util/record_course.sh
  scripts/estimation/localization_monitor.py
  scripts/simulator/speed_model_node.py
  scripts/simulator/perception_sim.py
  scripts/estimation/nand_estimator.py
  scripts/estimation/steer_offset_estimator.py
  DESTINATION lib/${PROJECT_NAME}
)


find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryMsg.msg"
  "msg/StampedFloat64Msg.msg"
  "msg/SCDebugInfoMsg.msg"
  "msg/SCSensorMsg.msg"
  "msg/NANDRawGPSMsg.msg"
  "msg/NANDDebugInfoMsg.msg"
  "msg/SCRadioNANDMsg.msg"
  "msg/DetectionsMsg.msg"
  "msg/TrackedObjectsMsg.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
