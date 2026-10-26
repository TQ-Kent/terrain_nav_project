cmake_minimum_required(VERSION 3.8)
project(terrain_multirobot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CbbaMsg.msg"
  "msg/TaskPoint.msg"
  "msg/TaskList.msg"
  "msg/TaskDone.msg"
)

ament_package()
