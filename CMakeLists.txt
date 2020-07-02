cmake_minimum_required(VERSION 3.5)

project(adi_imu_tr_driver_ros2)


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
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SimpleCmd.srv"
 )

include_directories(
  lib/include
  )

### demos
add_subdirectory(lib)
add_executable(adis_rcv_csv_node
  src/adis_rcv_csv_node.cpp
  )

# wait creating custom header
rosidl_target_interfaces(adis_rcv_csv_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )

ament_target_dependencies(adis_rcv_csv_node 
  "rclcpp"
  "sensor_msgs"
  "tf2_ros"
  "diagnostic_msgs"
  )
target_link_libraries(adis_rcv_csv_node adis_rcv_csv)


install(DIRECTORY
  launch rviz urdf
  DESTINATION share/${PROJECT_NAME}/
  )


install(TARGETS
  adis_rcv_csv_node
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()

