cmake_minimum_required(VERSION 3.8)
project(drive_tb3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



add_executable(p2_robot_square src/p2_robot_square.cpp)
ament_target_dependencies(p2_robot_square rclcpp  geometry_msgs)

add_executable(p3_go_to_goal src/p3_go_to_goal.cpp)
ament_target_dependencies(p3_go_to_goal rclcpp  geometry_msgs nav_msgs tf2_geometry_msgs)



install(TARGETS p2_robot_square p3_go_to_goal
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
