cmake_minimum_required(VERSION 3.8)
project(drive_tb3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)



add_executable(robot_square src/robot_square.cpp)
ament_target_dependencies(robot_square rclcpp  geometry_msgs)


install(TARGETS robot_square
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
