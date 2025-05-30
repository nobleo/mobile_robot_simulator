cmake_minimum_required(VERSION 3.5)
project(mobile_robot_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

###########
## Build ##
###########

include_directories(include)

add_library(mobile_robot_simulator
  src/mobile_robot_simulator.cpp
)
target_link_libraries(mobile_robot_simulator PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
)

add_library(laser_simulator
  src/laser_simulator.cpp
)
target_link_libraries(laser_simulator PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
)

add_executable(mobile_robot_simulator_node src/mobile_robot_simulator_node.cpp)
target_link_libraries(mobile_robot_simulator_node mobile_robot_simulator)

add_executable(laser_scanner_simulator_node src/laser_simulator_node.cpp)
target_link_libraries(laser_scanner_simulator_node laser_simulator)

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs nav_msgs rclcpp tf2_ros sensor_msgs)
ament_package()

#############
## Install ##
#############

install(
  TARGETS mobile_robot_simulator laser_simulator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mobile_robot_simulator_node laser_scanner_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)
