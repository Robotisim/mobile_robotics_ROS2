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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(p2_robot_square src/p2_robot_square.cpp)
ament_target_dependencies(p2_robot_square rclcpp  geometry_msgs)

add_executable(p3_go_to_goal src/p3_go_to_goal.cpp)
ament_target_dependencies(p3_go_to_goal rclcpp  geometry_msgs nav_msgs tf2_geometry_msgs)

add_executable(p5_lidar_a_processing src/p5_lidar_a_processing.cpp)
ament_target_dependencies(p5_lidar_a_processing rclcpp  geometry_msgs  sensor_msgs)

add_executable(p5_lidar_b_processing src/p5_lidar_b_processing.cpp)
ament_target_dependencies(p5_lidar_b_processing rclcpp  geometry_msgs  sensor_msgs)


add_executable(p5_lidar_c_processing src/p5_lidar_c_processing.cpp)
ament_target_dependencies(p5_lidar_c_processing rclcpp  geometry_msgs  sensor_msgs)

add_executable(p5_lidar_d_processing src/p5_lidar_d_processing.cpp)
ament_target_dependencies(p5_lidar_d_processing rclcpp  geometry_msgs  sensor_msgs)

add_executable(p5_camera_a_feed_process src/p5_camera_a_feed_process.cpp)
ament_target_dependencies(p5_camera_a_feed_process rclcpp sensor_msgs cv_bridge)
target_link_libraries(p5_camera_a_feed_process ${OpenCV_LIBS})


add_executable(p5_camera_b_feed_process src/p5_camera_b_feed_process.cpp)
ament_target_dependencies(p5_camera_b_feed_process rclcpp sensor_msgs cv_bridge)
target_link_libraries(p5_camera_b_feed_process ${OpenCV_LIBS})


add_executable(p5_camera_c_feed_process src/p5_camera_c_feed_process.cpp)
ament_target_dependencies(p5_camera_c_feed_process rclcpp sensor_msgs cv_bridge)
target_link_libraries(p5_camera_c_feed_process ${OpenCV_LIBS})




install(TARGETS p2_robot_square p3_go_to_goal
p5_lidar_a_processing p5_lidar_b_processing p5_lidar_c_processing p5_lidar_d_processing
p5_camera_a_feed_process p5_camera_b_feed_process p5_camera_c_feed_process
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch world
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
