cmake_minimum_required(VERSION 3.8)
project(ndi_aurora_node)

# ======================================================
# C++ standard
# ======================================================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ======================================================
# Compiler warnings
# ======================================================
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======================================================
# ROS 2 / dependencies
# ======================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# ======================================================
# Build NDI API static library
# ======================================================
add_library(ndi_api STATIC
  ndi_lib/src/BufferedReader.cpp
  ndi_lib/src/CombinedApi.cpp
  ndi_lib/src/ComConnection.cpp
  ndi_lib/src/Connection.cpp
  ndi_lib/src/CryptoConnection.cpp
  ndi_lib/src/DtlsConnection.cpp
  ndi_lib/src/FileConnection.cpp
  ndi_lib/src/GbfButton1D.cpp
  ndi_lib/src/GbfComponent.cpp
  ndi_lib/src/GbfContainer.cpp
  ndi_lib/src/GbfData3D.cpp
  ndi_lib/src/GbfData6D.cpp
  ndi_lib/src/GbfFrame.cpp
  ndi_lib/src/GbfLineSeparation.cpp
  ndi_lib/src/GbfMarker3DError.cpp
  ndi_lib/src/GbfSystemAlert.cpp
  ndi_lib/src/MarkerData.cpp
  ndi_lib/src/PortHandleInfo.cpp
  ndi_lib/src/SystemAlert.cpp
  ndi_lib/src/SystemCRC.cpp
  ndi_lib/src/TcpConnection.cpp
  ndi_lib/src/TlsConnection.cpp
  ndi_lib/src/ToolData.cpp
  ndi_lib/src/Transform.cpp
  ndi_lib/src/UdpSocket.cpp
)

target_include_directories(ndi_api PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ndi_lib/include>
  $<INSTALL_INTERFACE:include/ndi_lib>
)

target_link_libraries(ndi_api PUBLIC Eigen3::Eigen)

# ======================================================
# ndi_aurora_publisher node
# ======================================================
add_executable(ndi_aurora_publisher
  src/ndi_aurora_publisher.cpp
)

ament_target_dependencies(ndi_aurora_publisher
  rclcpp
  geometry_msgs
  std_srvs
)

target_link_libraries(ndi_aurora_publisher
  ndi_api
  Eigen3::Eigen
)

# ======================================================
# ndi_aurora_raw_node
# ======================================================
add_executable(ndi_aurora_raw_node
  src/ndi_aurora_raw_node.cpp
)

ament_target_dependencies(ndi_aurora_raw_node
  rclcpp
  geometry_msgs
  std_srvs
)

target_link_libraries(ndi_aurora_raw_node
  ndi_api
  Eigen3::Eigen
)

# ======================================================
# Install
# ======================================================
install(TARGETS ndi_api
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ndi_aurora_publisher
  ndi_aurora_raw_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install NDI headers (the only real headers)
install(DIRECTORY ndi_lib/include/
  DESTINATION include/ndi_lib
)

# ======================================================
# Export
# ======================================================
ament_export_include_directories(include/ndi_lib)
ament_export_libraries(ndi_api)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  std_srvs
  Eigen3
)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
