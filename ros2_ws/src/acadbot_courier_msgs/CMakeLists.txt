cmake_minimum_required(VERSION 3.8)
project(acadbot_courier_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# .srv / .action / .msg files get listed here once drafted, e.g.:
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/RequestDelivery.srv"
#   "action/ExecuteDelivery.action"
#   "msg/NamedLocation.msg"
#   DEPENDENCIES geometry_msgs
# )

ament_package()
