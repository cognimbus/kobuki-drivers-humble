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

FROM cognimbus/kobuki_all_drivers:humble as base

# Start new stage for tf_poses_stage
FROM base as tf_poses_stage

# Copy tf_to_poses from local machine to container
WORKDIR /kobuki-drivers-humble/src

# Clone repository and extract tf_to_poses with cleanup
RUN rm -rf tf_to_poses temp_repo && \
    git clone https://github.com/cognimbus/kobuki-drivers-humble.git temp_repo && \
    mv temp_repo/src/tf_to_poses . && \
    rm -rf temp_repo

# Clone again for robot_launchers update with cleanup
RUN rm -rf temp_repo2 && \
    git clone https://github.com/cognimbus/kobuki-drivers-humble.git temp_repo2 && \
    rm -rf /kobuki-drivers-humble/src/robot_launchers/launch && \
    mv temp_repo2/src/robot_launchers/launch /kobuki-drivers-humble/src/robot_launchers/ && \
    rm -rf temp_repo2

# Build only tf_to_poses package
WORKDIR /kobuki-drivers-humble
RUN . /opt/ros/humble/setup.sh && \
    . ./install/setup.sh && \
    colcon build --symlink-install --packages-select tf_to_poses robot_launchers

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /kobuki-drivers-humble/install/setup.bash" >> ~/.bashrc 

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

