FROM ros:noetic

# Install Gazebo
RUN apt-get update && apt-get install -y gazebo11 ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-xacro

# Install other dependencies
RUN apt-get install -y python3-rosdep python3-rosinstall python3-vcstools build-essential

# Install plotjuggler
RUN sudo apt install ros-noetic-plotjuggler-ros

# Set up environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

WORKDIR /root/catkin_ws
RUN mkdir -p src