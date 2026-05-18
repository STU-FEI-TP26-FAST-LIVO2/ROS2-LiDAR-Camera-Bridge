cmake_minimum_required(VERSION 3.8)
project(lidar_stamp_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(lidar_stamp_bridge_node
  src/lidar_stamp_bridge_node.cpp
)

target_include_directories(lidar_stamp_bridge_node PRIVATE include)
target_compile_features(lidar_stamp_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(lidar_stamp_bridge_node rclcpp sensor_msgs)

install(TARGETS lidar_stamp_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
