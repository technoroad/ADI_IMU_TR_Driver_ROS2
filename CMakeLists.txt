cmake_minimum_required(VERSION 3.5)

project(adi_driver2)

# Default to C++14
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
find_package(tf2_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)

include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

### demos
add_executable(adis_rcv_csv_node
  src/adis_rcv_csv_node.cpp
  src/adis_rcv_csv.cpp
  )

ament_target_dependencies(adis_rcv_csv_node 
  "rclcpp"
  "sensor_msgs"
  "tf2_ros"
  "diagnostic_msgs"
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY
  launch params
  DESTINATION share/${PROJECT_NAME}/
)


install(TARGETS
  adis_rcv_csv_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
