cmake_minimum_required(VERSION 3.5)
project(ros_gz_example_application)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ign_pubsub src/ign_pubsub.cpp)
ament_target_dependencies(ign_pubsub rclcpp std_msgs sensor_msgs)
install(TARGETS
  ign_pubsub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
