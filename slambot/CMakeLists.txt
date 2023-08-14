cmake_minimum_required(VERSION 3.8)
project(slambot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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

add_executable(p7_a_encoders src/p7_a_encoders.cpp)
target_link_libraries(p7_a_encoders ${PIGPIOD_IF2_LIBRARY} rt)
ament_target_dependencies(p7_a_encoders rclcpp)

add_executable(p7_a_imus src/p7_a_imus.cpp)
target_link_libraries(p7_a_imus ${PIGPIOD_IF2_LIBRARY} rt)
ament_target_dependencies(p7_a_imus rclcpp)


# add_executable(p7_c_cmd_vel src/p7_c_cmd_vel.cpp)
# ament_target_dependencies(p7_c_cmd_vel rclcpp geometry_msgs)


install(TARGETS test_gpio p7_a_motor_driving p7_a_encoders p7_a_imus

  DESTINATION lib/${PROJECT_NAME})


# install(PROGRAMS
#       src/p6_c_commander_api_go_to_pose.py

# DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY launch
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
