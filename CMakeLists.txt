cmake_minimum_required(VERSION 3.8)
project(パッケージ名)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kk_driver_msg REQUIRED)

## 
add_executable(ノード名 src/main.cpp)
ament_target_dependencies(ノード名 std_msgs sensor_msgs rclcpp kk_driver_msg)

install(TARGETS
  ノード名
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
