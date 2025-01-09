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
find_package(ros_gz_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)  # Add Eigen3



#IGNPUBSUB
add_executable(ign_pubsub src/ign_pubsub.cpp src/ButterworthFilter.cpp)
ament_target_dependencies(ign_pubsub
 rclcpp 
 std_msgs 
 sensor_msgs 
 ros_gz_interfaces 
 geometry_msgs 
 tf2 
 tf2_ros 
 visualization_msgs)

install(TARGETS
  ign_pubsub
  DESTINATION lib/${PROJECT_NAME})


 target_include_directories(ign_pubsub PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory 
)

target_link_libraries(ign_pubsub ButterworthFilter)


#SEDAS_RVIZ
add_executable(sedas_rviz src/sedas_rviz.cpp)
ament_target_dependencies(sedas_rviz
 rclcpp 
 std_msgs 
 sensor_msgs 
 ros_gz_interfaces 
 geometry_msgs 
 tf2 
 tf2_ros 
 visualization_msgs)

install(TARGETS
sedas_rviz
  DESTINATION lib/${PROJECT_NAME})


 target_include_directories(sedas_rviz PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory 
)

target_link_libraries(sedas_rviz)


#PINOCCHIOTRY
add_executable(pinocchio_try src/pinocchio_try.cpp)
ament_target_dependencies(pinocchio_try
 rclcpp 
 std_msgs 
 sensor_msgs 
 geometry_msgs 
 tf2 
 tf2_ros 
)
 
target_link_libraries(pinocchio_try pinocchio::pinocchio)

target_include_directories(pinocchio_try PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory 
)

install(TARGETS
  pinocchio_try
  DESTINATION lib/${PROJECT_NAME})


#SEDAS_IGNGZ
add_executable(sedas_igngz src/sedas_igngz.cpp)
ament_target_dependencies(sedas_igngz
 rclcpp 
 std_msgs 
 sensor_msgs 
 ros_gz_interfaces 
 geometry_msgs 
 tf2 
 tf2_ros 
 visualization_msgs)

target_include_directories(sedas_igngz PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory 
)


install(TARGETS
sedas_igngz
  DESTINATION lib/${PROJECT_NAME})



#SEDAS_IGNGZ
add_executable(sedas_control src/sedas_control.cpp)
ament_target_dependencies(sedas_control
 rclcpp 
 std_msgs 
 sensor_msgs 
 ros_gz_interfaces 
 geometry_msgs 
 tf2 
 tf2_ros 
 visualization_msgs)

 target_include_directories(sedas_control PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory 
)


install(TARGETS
sedas_control
  DESTINATION lib/${PROJECT_NAME})

#butterworth filter
add_library(ButterworthFilter src/ButterworthFilter.cpp)
target_include_directories(ButterworthFilter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}  # Eigen의 include 경로 추가
)


ament_package()
