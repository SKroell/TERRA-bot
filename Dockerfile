FROM ros:iron
ARG ROS_DISTRO=iron

# Install necessary packages including SSH server
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y python3-pip curl

RUN apt-get update \
 && apt-get install -y \
    build-essential \
    cmake \
    git-all \
    software-properties-common 

RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* 

RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-twist-stamper

# Clone the repository for ODrive Nodes (only odrive_node, odrive_base, odrive_ros2_control)
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws
RUN git clone -n --depth=1 --filter=tree:0 https://github.com/odriverobotics/ros_odrive src \
    && cd src \
    && git sparse-checkout init --cone \
    && git sparse-checkout set odrive_node odrive_base odrive_ros2_control \
    && git checkout
ADD ros_ws/src/odrive_terra_bot /ros_ws/src/odrive_terra_bot

# Install necessary ROS dependencies and build the workspace
RUN rosdep install --from-paths src --ignore-src -r -y \
    && . /opt/ros/iron/setup.sh \
    && colcon build

# Set the environment to source ROS2 setup files and the entrypoint
CMD ["bash", "-c", "cd ros_ws \
     && source /opt/ros/iron/setup.bash \
     && source install/local_setup.bash \
     && ros2 launch terra_bot terra_bot.launch.py \
    "]

