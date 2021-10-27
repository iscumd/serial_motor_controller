cmake_minimum_required(VERSION 3.5)
project(serial_motor_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(serial_motor_controller src/serial_motor_controller.cpp)
ament_target_dependencies(
  serial_motor_controller
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

target_include_directories(serial_motor_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(roboteq_mdc2460 src/roboteq_mdc2460.cpp)
target_link_libraries(roboteq_mdc2460 serial_motor_controller)
add_executable(roboteq_mdc2460_node src/nodes/roboteq_mdc2460_node.cpp)
target_link_libraries(roboteq_mdc2460_node roboteq_mdc2460)

add_library(roboteq_hdc2460 src/roboteq_hdc2460.cpp)
target_link_libraries(roboteq_hdc2460 serial_motor_controller)
add_executable(roboteq_hdc2460_node src/nodes/roboteq_hdc2460_node.cpp)
target_link_libraries(roboteq_hdc2460_node roboteq_hdc2460)

add_library(sabertooth_2x12 src/sabertooth_2x12.cpp)
target_link_libraries(sabertooth_2x12 serial_motor_controller)
add_executable(sabertooth_2x12_node src/nodes/sabertooth_2x12_node.cpp)
target_link_libraries(sabertooth_2x12_node sabertooth_2x12)

install(
  TARGETS
  serial_motor_controller
  roboteq_mdc2460
  roboteq_hdc2460
  sabertooth_2x12
  roboteq_mdc2460_node
  roboteq_hdc2460_node
  sabertooth_2x12_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()