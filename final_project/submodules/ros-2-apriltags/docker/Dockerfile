ARG BASE_IMAGE=fbe-dockerreg.rwu.de/prj-iki-ros2/orga/ros2-docker-base:humble
FROM $BASE_IMAGE as base
RUN apt update \ 
&& apt install -y vim \
   ros-$ROS_DISTRO-image-transport \
   ros-$ROS_DISTRO-cv-bridge \
&& rm -rf /var/lib/apt/lists/*

ENV WS /root/ros2_ws/src/prj-iki-ros2/perception

# copy necessary packages for apriltag_detection
WORKDIR ${WS}

COPY submodules/apriltag ${WS}/apriltag
COPY submodules/apriltag_ros ${WS}/apriltag_ros
COPY submodules/apriltag_msgs ${WS}/apriltag_msgs

# COPY necessary files
COPY configs/* ${WS}/apriltag_ros/cfg

FROM base as build
#build packages
WORKDIR /root/ros2_ws/
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --packages-select apriltag apriltag_msgs apriltag_ros ros2_apriltag_viz

