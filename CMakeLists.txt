cmake_minimum_required(VERSION 3.8)
project(kk_servo_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kk_driver_msg REQUIRED)

add_executable(kk_servo_ctrl src/kk_servo_ctrl.cpp)
ament_target_dependencies(kk_servo_ctrl rclcpp kk_driver_msg)

install(TARGETS
  kk_servo_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
