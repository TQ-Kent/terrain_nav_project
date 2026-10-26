cmake_minimum_required(VERSION 3.8)
project(terrain_speed_governor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(governor_math src/governor_math.cpp)
target_include_directories(governor_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(speed_governor src/speed_governor_node.cpp)
target_link_libraries(speed_governor governor_math)
ament_target_dependencies(speed_governor rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS speed_governor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_governor_math test/test_governor_math.cpp)
  target_link_libraries(test_governor_math governor_math)
endif()

ament_package()
