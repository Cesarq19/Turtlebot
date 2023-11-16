################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_mobility)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/diff_drive_controller.cpp"
  "src/odometry.cpp"
  "src/velocity_cmd_node.cpp"
)

set(DEPENDENCIES
  "dynamixel_sdk"
  "geometry_msgs"
  "message_filters"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
)

set(EXEC_NAME "turtlebot3_mobility")

add_executable(${EXEC_NAME} src/velocity_cmd_node.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXEC_NAME} ${DEPENDENCIES})

################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)
################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(dynamixel_sdk)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_package()