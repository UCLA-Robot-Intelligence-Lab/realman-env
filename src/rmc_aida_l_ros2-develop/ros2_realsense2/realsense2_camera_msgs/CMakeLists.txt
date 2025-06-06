cmake_minimum_required(VERSION 3.5)
#使用python旧规则编译，以去除警告
if(POLICY CMP0148) 
  cmake_policy(SET CMP0148 OLD) 
endif()
project(realsense2_camera_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/IMUInfo.msg"
  "msg/Extrinsics.msg"
  "msg/Metadata.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  "srv/DeviceInfo.srv"
  DEPENDENCIES builtin_interfaces std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
