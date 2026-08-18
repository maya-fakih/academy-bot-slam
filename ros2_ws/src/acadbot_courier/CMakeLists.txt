cmake_minimum_required(VERSION 3.8)
project(acadbot_courier)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(acadbot_courier_msgs REQUIRED)

# Executables get added here once nodes exist, e.g.:
# add_executable(dispatcher src/dispatcher.cpp)
# ament_target_dependencies(dispatcher
#   rclcpp rclcpp_action geometry_msgs nav2_msgs acadbot_courier_msgs)
#
# add_executable(executor src/executor.cpp)
# ament_target_dependencies(executor
#   rclcpp rclcpp_action geometry_msgs nav2_msgs acadbot_courier_msgs)
#
# install(TARGETS
#   dispatcher
#   executor
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
