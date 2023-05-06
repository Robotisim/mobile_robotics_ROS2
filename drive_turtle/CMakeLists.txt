cmake_minimum_required(VERSION 3.8)
project(drive_turtle)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(multi_turtle_controller src/p1_3_multi_turtle_controller.cpp)
ament_target_dependencies(multi_turtle_controller rclcpp turtlesim geometry_msgs)


add_executable(single_straight_drive src/p1_1_single_straight_drive.cpp)
ament_target_dependencies(single_straight_drive rclcpp turtlesim geometry_msgs)


add_executable(multi_straight_drive src/p1_2_multi_straight_drive.cpp)
ament_target_dependencies(multi_straight_drive rclcpp turtlesim geometry_msgs)

install(TARGETS
  multi_turtle_controller single_straight_drive multi_straight_drive
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
