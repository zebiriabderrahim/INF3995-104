FROM ros:noetic-ros-base-focal

RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-multirobot-map-merge \
    ros-noetic-explore-lite \
    ros-noetic-rosbridge-server \
    ros-noetic-roslint \
    ros-noetic-slam-gmapping \
    ros-noetic-rtabmap-ros \
    ros-noetic-move-base \
    ros-noetic-robot-state-publisher \
    ros-noetic-rviz \
    ros-noetic-xacro \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    libx11-xcb1 \
    libgl1-mesa-glx \
    libfontconfig1 \
    libdbus-1-3 \
    libxrender1 \
    libxi6 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    xauth \
    xterm \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

COPY ./gz_sim_ws/src /root/gz_sim_ws/src

ENV ROS_WS=/root/gz_sim_ws
WORKDIR $ROS_WS



# # Set up X11 forwarding
# ARG HOST_IP

ENV DISPLAY=$HOST_IP:0
# Set up X11 forwarding
RUN echo "export DISPLAY=$HOST_IP:0" \
    >> ~/.bashrc && \
    echo "source /opt/ros/noetic/setup.bash" \
    >> ~/.bashrc && \
    echo "source $ROS_WS/devel/setup.bash" \
    >> ~/.bashrc

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; \
    rosdep install --from-paths src --ignore-src -r -y; \
    catkin_make'

EXPOSE 9090

CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source $ROS_WS/devel/setup.bash && roslaunch limo_gazebo_sim sim_rosbrige_ser.launch open_rviz:=false"]