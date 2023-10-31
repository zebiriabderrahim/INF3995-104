FROM ubuntu:latest

# Set the DEBIAN_FRONTEND environment variable to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list and install necessary packages
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-get update && \
    apt-get install -y curl lsb-release && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update && \
    apt-get install -y ros-noetic-desktop-full && \
    apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
    apt-get install -y python3-rosdep && \
    rosdep init && \
    rosdep update
# Add the ROS keyring
RUN apt-get update && apt-get install -y gnupg2
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Set the shell back to interactive
ENV DEBIAN_FRONTEND=dialog

WORKDIR /home/nvidia/INF3995-104/embedded/robot_104_ws

# Install ROS (replace 'ros-version' with your specific ROS version)
#RUN apt-get update && apt-get install -y ros-noetic-desktop

# Install Python
#RUN apt-get update && apt-get install -y sudo python3

COPY . /home/nvidia/INF3995-104/embedded/robot_104_ws
#COPY entrypoint.sh /entrypoint.sh

# Make the entrypoint script executable
#RUN chmod +x /entrypoint.sh

# Set the entrypoint for the container
#ENTRYPOINT ["/entrypoint.sh"]

