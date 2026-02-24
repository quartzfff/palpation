cmake_minimum_required(VERSION 3.5)
project(netft_reader)

# ----------------------------------------------------
# C++ standard
# ----------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ----------------------------------------------------
# Dependencies
# ----------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)

# ----------------------------------------------------
# Include headers
# ----------------------------------------------------
include_directories(include)

# ----------------------------------------------------
# Build the original netft_node executable
# ----------------------------------------------------
add_executable(netft_node
  src/netft_node.cpp
  src/netft_rdt_driver.cpp
)

ament_target_dependencies(netft_node
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
)

target_link_libraries(netft_node
  Boost::thread
  Boost::system
)

# ----------------------------------------------------
# Build the netft_raw_node executable
# ----------------------------------------------------
add_executable(netft_raw_node
  src/netft_raw_node.cpp
  src/netft_rdt_driver.cpp
)

ament_target_dependencies(netft_raw_node
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
)

target_link_libraries(netft_raw_node
  Boost::thread
  Boost::system
)

# ----------------------------------------------------
# Build the netft_baseraw_node executable (NEW)
# ----------------------------------------------------
add_executable(netft_baseraw_node
  src/netft_baseraw_node.cpp
  src/netft_rdt_driver.cpp
)

ament_target_dependencies(netft_baseraw_node
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
)

target_link_libraries(netft_baseraw_node
  Boost::thread
  Boost::system
)

# ----------------------------------------------------
# Install targets and headers
# ----------------------------------------------------
install(TARGETS
  netft_node
  netft_raw_node
  netft_baseraw_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ----------------------------------------------------
# Export the package
# ----------------------------------------------------
ament_package()
