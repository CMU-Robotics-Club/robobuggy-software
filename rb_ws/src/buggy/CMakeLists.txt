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
  scripts/watchdog/watchdog_node.py
  scripts/buggy_state_converter.py
  scripts/serial/ros_to_bnyahaj.py
  scripts/telemetry/telematics.py
  scripts/debug/debug_steer.py
  DESTINATION lib/${PROJECT_NAME}
)


find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryMsg.msg"
  "msg/SCDebugInfoMsg.msg"
  "msg/SCSensorMsg.msg"
  "msg/NANDRawGPSMsg.msg"
  "msg/NANDDebugInfoMsg.msg"
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()