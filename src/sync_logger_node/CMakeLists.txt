cmake_minimum_required(VERSION 3.10)
project(sync_logger_node)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # If needed
find_package(Eigen3 REQUIRED NO_MODULE)

# Add message_filters (needed for synchronizer)
find_package(message_filters REQUIRED)

# Create executable
add_executable(sync_logger_node src/sync_logger_node.cpp)

# Link against ROS 2 and Eigen
ament_target_dependencies(sync_logger_node
  rclcpp
  std_srvs
  geometry_msgs
  sensor_msgs
  message_filters
)

target_link_libraries(sync_logger_node
  Eigen3::Eigen
)

# Install target
install(TARGETS sync_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export package
ament_package()
