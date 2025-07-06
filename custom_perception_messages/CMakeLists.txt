cmake_minimum_required(VERSION 3.8)
project(custom_perception_messages)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedSurfaces.msg"
  "msg/DetectedObjects.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()