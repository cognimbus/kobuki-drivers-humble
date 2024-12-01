FROM ros:humble-perception-jammy

RUN apt-get update && apt-get install -y \
    usbutils \
    ros-humble-cv-bridge \
    ros-humble-librealsense2 \
    ros-humble-message-filters \
    ros-humble-image-transport \
    ros-humble-realsense2-camera \
    libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions \
    ros-humble-rviz2 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3 \ 
    ros-humble-slam-toolbox \
    ros-humble-twist-mux \
    ros-humble-cartographer \
    ros-humble-ecl-sigslots \
    libusb-1.0-0-dev libftdi1-dev libuvc-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /kobuki_ws/src
RUN git clone --branch=humble https://github.com/IntelligentRoboticsLabs/kobuki.git
COPY src/kobuki/thirdparty.repos /kobuki/thirdparty.repos
RUN . /opt/ros/humble/setup.sh && vcs import < /kobuki/thirdparty.repos



WORKDIR /kobuki_ws
RUN 	sh /opt/ros/humble/setup.sh && sudo rosdep init && rosdep update \
    apt update && \
    rosdep install -i --from-path src --ignore-src --rosdistro humble -y




WORKDIR /kobuki_ws/src
RUN git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git
RUN rosdep update && \
    rosdep install -i --from-paths urg_node2

RUN git clone https://github.com/Slamtec/sllidar_ros2.git


WORKDIR /kobuki_ws
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --parallel-workers 1 --packages-skip gazebo_plugins aws_robomaker_bookstore_world aws_robomaker_racetrack_world aws_robomaker_small_house_world aws_robomaker_small_warehouse_world





RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /kobuki_ws/src/install/setup.bash" >> ~/.bashrc && \

    COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]



