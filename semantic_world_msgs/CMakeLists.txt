cmake_minimum_required(VERSION 3.16)  # 3.16+ is safe for Jazzy; 3.10 also works if you must
project(semantic_world_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
