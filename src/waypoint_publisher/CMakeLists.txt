cmake_minimum_required(VERSION 3.8)
project(waypoint_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Create executable
add_executable(waypoint_publisher src/waypoint_publisher.cpp)


# Link dependencies
ament_target_dependencies(waypoint_publisher rclcpp std_msgs nav_msgs)


# Include data/ folder
install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME})

# Include header files
target_include_directories(waypoint_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Require C99 and C++17
target_compile_features(waypoint_publisher PUBLIC c_std_99 cxx_std_17)

# Install executable to lib/ for ros2 run
install(TARGETS
  waypoint_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
