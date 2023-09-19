cmake_minimum_required(VERSION 3.8)
project(point_cloud_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(p9_a_pcd_import src/p9_a_table_scene.cpp)
target_link_libraries(p9_a_pcd_import ${PCL_LIBRARIES})

add_executable(p9_b_plane_cloud src/p9_b_planner_cloud.cpp)
target_link_libraries(p9_b_plane_cloud ${PCL_LIBRARIES})

add_executable(p9_c_circular_cloud src/p9_c_circular_cloud.cpp)
target_link_libraries(p9_c_circular_cloud ${PCL_LIBRARIES})


add_executable(p9_d_filtering_segmentation src/p9_d_filtering_segmentation.cpp)
target_link_libraries(p9_d_filtering_segmentation ${PCL_LIBRARIES})

add_executable(p9_e_path_creation src/p9_e_point_path_drawing.cpp)
target_link_libraries(p9_e_path_creation ${PCL_LIBRARIES})




install(TARGETS p9_a_pcd_import p9_b_plane_cloud p9_c_circular_cloud p9_d_filtering_segmentation  p9_e_path_creation
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
