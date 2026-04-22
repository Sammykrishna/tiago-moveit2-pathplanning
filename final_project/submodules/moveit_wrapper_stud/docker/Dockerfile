ARG ROS_DISTRO=humble
ARG BASE_IMAGE=fbe-dockerreg.rwu.de/prj-iki-ros2/orga/ros2-docker-base:$ROS_DISTRO
FROM ${BASE_IMAGE} AS base

ENV ROS_WS=/root/ros2_ws

 # Install dependencies
COPY moveit_wrapper_interfaces/package.xml src/moveit_wrapper_interfaces/package.xml
COPY moveit_wrapper/package.xml src/moveit_wrapper/package.xml
RUN apt update \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -r -y \
 && rm -rf /var/lib/apt/lists/*

# Build
COPY moveit_wrapper_interfaces src/moveit_wrapper_interfaces
COPY moveit_wrapper src/moveit_wrapper
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && colcon build --symlink-install --parallel-workers 4

 COPY scripts /scripts
