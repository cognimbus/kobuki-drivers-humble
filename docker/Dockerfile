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
    ros-humble-laser-proc \
    libusb-1.0-0-dev libftdi1-dev libuvc-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/humble/setup.sh
RUN git clone https://github.com/cognimbus/kobuki-drivers-humble.git

WORKDIR /kobuki-drivers-humble/src
RUN vcs import < kobuki/thirdparty.repos



WORKDIR /kobuki-drivers-humble
RUN sh /opt/ros/humble/setup.sh && \
    rosdep update && \
    apt update && \
    rosdep install -i --from-path src --ignore-src --rosdistro humble -y \
    --skip-keys="gazebo gazebo11 \
    gazebo_dev gazebo_msgs gazebo_model gazebo_physics gazebo_plugins gazebo_ros gazebo_world \
    gazebo_ros_control gazebo_ros_pkgs \
    ros-humble-gazebo ros-humble-gazebo-dev ros-humble-gazebo-msgs \
    ros-humble-gazebo-plugins ros-humble-gazebo-ros ros-humble-gazebo-ros-control \
    ros-humble-gazebo-ros-pkgs \
    aws_robomaker_bookstore_world aws_robomaker_racetrack_world \
    aws_robomaker_small_house_world aws_robomaker_small_warehouse_world"

# First build specific packages
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --parallel-workers 1 \
    --packages-select astra_camera astra_camera_msgs urg_node2

# Then build the rest, skipping the already built packages and problematic ones
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --parallel-workers 1 \
    --packages-skip gazebo_plugins gazebo_ros gazebo_dev \
    aws_robomaker_bookstore_world aws_robomaker_racetrack_world \
    aws_robomaker_small_house_world aws_robomaker_small_warehouse_world \
    --packages-skip-up-to urg_node2 astra_camera astra_camera_msgs


RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /kobuki-drivers-humble/src/install/setup.bash" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]



