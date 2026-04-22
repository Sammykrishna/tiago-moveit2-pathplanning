cmake_minimum_required(VERSION 3.8)
project(moveit_wrapper_interfaces)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

set(msg_files
    "msg/Waypoint.msg"
    "msg/WaypointSequence.msg"
)

set(service_files
    "srv/AddColObj.srv"
    "srv/AttachObj.srv"
    "srv/DettachObj.srv"
    "srv/ExecutePlans.srv"
    "srv/GetAttachedObjs.srv"
    "srv/GetColObj.srv"
    "srv/GetNamedTarget.srv"
    "srv/GetNamedTargets.srv"
    "srv/GotoNamedTarget.srv"
    "srv/Plan.srv"
    "srv/PlanJoints.srv"
    "srv/RemoveColObj.srv"
    "srv/MoveTorso.srv"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${service_files}
  DEPENDENCIES geometry_msgs visualization_msgs sensor_msgs moveit_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
