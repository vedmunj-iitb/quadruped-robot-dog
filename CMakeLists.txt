cmake_minimum_required(VERSION 3.8)
project(robot_dog_sim)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(gait_node src/gait_node.cpp)
target_include_directories(gait_node PRIVATE include)
ament_target_dependencies(gait_node rclcpp std_msgs geometry_msgs sensor_msgs)

add_executable(path_node src/path_node.cpp)
ament_target_dependencies(path_node rclcpp geometry_msgs nav_msgs)

install(TARGETS gait_node path_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch urdf worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_leg test/test_leg.cpp)
  target_include_directories(test_leg PRIVATE include)
endif()

ament_package()
