cmake_minimum_required(VERSION 3.5)
project(lidar_to_pcd)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED) 
find_package(PCL REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
# Add executable
add_executable(lidar_to_pcd src/lidar_to_pcd.cpp)
#add_executable(obsatcle_detect src/obstacle_detect.cpp)

ament_target_dependencies (${PROJECT_NAME}
   rclcpp
   sensor_msgs
   pcl_conversions
)
# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}  # Add this line to include PCL headers
)

# Link against PCL libraries
target_link_libraries(lidar_to_pcd
  ${PCL_LIBRARIES}
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Install targets
install(TARGETS lidar_to_pcd
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/obstacle_detection.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME obstacle_detection  # Rename the installed file
)
# Install additional files (e.g., launch files, configuration files)
install(DIRECTORY launch
 DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
 
ament_package()

