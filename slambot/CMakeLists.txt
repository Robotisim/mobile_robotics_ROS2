cmake_minimum_required(VERSION 3.8)
project(slambot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_library(PIGPIOD_IF2_LIBRARY pigpiod_if2)



add_executable(test_gpio src/test_gpio.cpp)
target_link_libraries(test_gpio ${PIGPIOD_IF2_LIBRARY} rt)
ament_target_dependencies(test_gpio rclcpp)

add_executable(p7_a_motor_driving src/p7_a_motor_driving.cpp)
target_link_libraries(p7_a_motor_driving ${PIGPIOD_IF2_LIBRARY} rt)
ament_target_dependencies(p7_a_motor_driving rclcpp)

add_executable(p7_a_encoders_tick_rev src/p7_a_encoders_tick_rev.cpp)
target_link_libraries(p7_a_encoders_tick_rev ${PIGPIOD_IF2_LIBRARY} rt)
ament_target_dependencies(p7_a_encoders_tick_rev rclcpp)

add_executable(p7_a_encoders_difference src/p7_a_encoders_difference.cpp)
target_link_libraries(p7_a_encoders_difference ${PIGPIOD_IF2_LIBRARY} rt)
ament_target_dependencies(p7_a_encoders_difference rclcpp)

add_executable(p7_a_encoders_direction_manual src/p7_a_encoders_direction_manual.cpp)
target_link_libraries(p7_a_encoders_direction_manual ${PIGPIOD_IF2_LIBRARY} rt)
ament_target_dependencies(p7_a_encoders_direction_manual rclcpp)

add_executable(p7_a_encoders_manual_interrupt src/p7_a_encoders_manual_interrupt.cpp)
target_link_libraries(p7_a_encoders_manual_interrupt ${PIGPIOD_IF2_LIBRARY} rt)
ament_target_dependencies(p7_a_encoders_manual_interrupt rclcpp)

# add_executable(p7_c_odom src/p7_c_odom.cpp)
# ament_target_dependencies(p7_c_odom rclcpp std_msgs geometry_msgs)

# add_executable(p7_a_imus src/p7_a_imus.cpp)
# target_link_libraries(p7_a_imus ${PIGPIOD_IF2_LIBRARY} rt)
# ament_target_dependencies(p7_a_imus rclcpp)



add_executable(p7_b_cmd_vel_drive src/p7_b_cmd_vel_drive.cpp)
target_link_libraries(p7_b_cmd_vel_drive ${PIGPIOD_IF2_LIBRARY} rt)
ament_target_dependencies(p7_b_cmd_vel_drive rclcpp geometry_msgs)

add_executable(p7_b_sensor_publish src/p7_b_sensor_publish.cpp)
target_link_libraries(p7_b_sensor_publish ${PIGPIOD_IF2_LIBRARY} rt)
ament_target_dependencies(p7_b_sensor_publish rclcpp geometry_msgs sensor_msgs)


install(TARGETS test_gpio p7_a_motor_driving p7_a_encoders_difference  p7_a_encoders_tick_rev  p7_a_encoders_direction_manual
                p7_b_cmd_vel_drive p7_b_sensor_publish p7_a_encoders_manual_interrupt

  DESTINATION lib/${PROJECT_NAME})


install(PROGRAMS
      # src/p7_b_cmd_vel_drive.py
      # src/p7_a_enc_read.py
      src/p7_a_drive_control.py
      src/p7_c_odom.py

DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY launch config map
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
