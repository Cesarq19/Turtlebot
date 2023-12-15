cmake_minimum_required(VERSION 3.8)
project(node_turtlebot)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_lib
  "src/main.cpp"
  "src/motores.cpp" 
  "src/sensores.cpp"
)

set(DEPENDENCIES
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "std_msgs"
  "nav_msgs"
  "dynamixel_sdk"
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

set(EXEC_NAME "tb3_node")

add_executable(${EXEC_NAME} src/main.cpp)
target_link_libraries(${EXEC_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXEC_NAME} ${DEPENDENCIES})

install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(dynamixel_sdk)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(nav_msgs)
ament_package()

# add_executable(motores src/motores.cpp)
# ament_target_dependencies(motores rclcpp std_msgs)

# add_executable(sensores src/sensores.cpp)
# ament_target_dependencies(sensores rclcpp std_msgs sensor_msgs geometry_msgs)

# add_executable(main src/main.cpp)
# ament_target_dependencies(main rclcpp std_msgs sensor_msgs geometry_msgs)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# install(TARGETS
#     motores
#     sensores
#     main
#     DESTINATION lib/${PROJECT_NAME}
# )

# ament_package()