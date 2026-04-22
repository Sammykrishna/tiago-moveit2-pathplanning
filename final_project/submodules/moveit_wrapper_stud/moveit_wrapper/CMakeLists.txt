cmake_minimum_required(VERSION 3.16)
project(moveit_wrapper)

# Set C++ standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)

# For Jazzy, moveit_cpp functionality is included in moveit_ros_planning
# Try to find moveit_cpp, fall back to moveit_ros_planning if not found
find_package(moveit_cpp QUIET)
if(NOT moveit_cpp_FOUND)
  message(STATUS "moveit_cpp not found (normal for Jazzy), using moveit_ros_planning instead")
  # In Jazzy, moveit_cpp functionality is part of moveit_ros_planning
  set(MOVEIT_CPP_DEPENDENCY moveit_ros_planning)
else()
  message(STATUS "Found moveit_cpp package")
  set(MOVEIT_CPP_DEPENDENCY moveit_cpp)
endif()

find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_wrapper_interfaces REQUIRED)

# Includes
include_directories(
  include
)

file(GLOB MOVEIT_WRAPPER_SRCS
  "src/*.cpp"
  "src/services/*.cpp"
)

add_library(${PROJECT_NAME} SHARED ${MOVEIT_WRAPPER_SRCS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_core
  ${MOVEIT_CPP_DEPENDENCY}
  geometry_msgs
  shape_msgs
  visualization_msgs
  sensor_msgs
  moveit_msgs
  moveit_wrapper_interfaces
)

rclcpp_components_register_nodes(${PROJECT_NAME} "moveit_wrapper::MoveitWrapper")

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY params/ DESTINATION share/${PROJECT_NAME}/params)

ament_package()
