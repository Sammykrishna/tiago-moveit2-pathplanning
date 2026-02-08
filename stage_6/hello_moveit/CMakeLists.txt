cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
add_executable(hello_moveit src/hello_moveit.cpp)

ament_target_dependencies(hello_moveit
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

add_executable(collision_object_test src/collision_object_test.cpp)
ament_target_dependencies(collision_object_test
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS
  collision_object_test
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(hello_moveit_collision_cycle src/hello_moveit_collision_cycle.cpp)
ament_target_dependencies(hello_moveit_collision_cycle
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

install(TARGETS
  hello_moveit_collision_cycle
  DESTINATION lib/${PROJECT_NAME}
)