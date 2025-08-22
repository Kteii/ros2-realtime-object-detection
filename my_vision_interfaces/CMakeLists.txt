cmake_minimum_required(VERSION 3.8)
project(my_vision_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  DEPENDENCIES std_msgs
)

# Export the package dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)

# Declare build dependencies
ament_package(
  # List your package dependencies here if any
  # CONFIG_EXTRAS "my_vision_interfaces-extras.cmake"
)
