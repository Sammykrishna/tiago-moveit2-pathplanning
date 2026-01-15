cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces (msg/srv/action)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriveToWallT.msg"
  "msg/ReachedWallT.msg"
  "srv/DriveToWallS.srv"
  "action/MoveHead.action"
  "action/LookAtMarker.action"
)

# Needed so other pkgs can depend on generated interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()
