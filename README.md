commands to launch the robots: from ~/kobuki_ws/src

# Robot Launch Commands

## RealSense Camera

ros2 launch realsense2_camera rs_launch.py

## RPLidar

ros2 launch sllidar_ros2 view_sllidar_a2m8_launch.py scan_mode:=Express

## Kobuki Robot

ros2 launch kobuki kobuki.launch.py

## Hokuyo LiDAR

ros2 launch urg_node2 urg_node2.launch.py

## AMR8 Robot

ros2 launch robot_launchers amr8_robot.launch.py

## Qualcomm Robot

ros2 launch robot_launchers qualcomm_robot.launch.py

# Copy UDEV Rules for Devices

sudo cp /kobuki_ws/src/ThirdParty/ros_astra_camera/astra_camera/scripts/56-orbbec-usb.rules /etc/udev/rules.d/
sudo cp /kobuki_ws/src/ThirdParty/rplidar_ros/scripts/rplidar.rules /etc/udev/rules.d/
sudo cp /kobuki_ws/src/ThirdParty/kobuki_ros/60-kobuki.rules /etc/udev/rules.d/

# Set Permissions for Devices

chmod 777 /dev/ttyUSB0
chmod 777 /dev/video0
sudo chmod 777 /dev/ttyACM0

# Kobuki Robot Configuration

Update the configuration file at `src/kobuki/config/kobuki_node_params.yaml` with the following:

GNU nano 6.2 kobuki_node_params.yaml  
kobuki_ros_node:
ros\_\_parameters:
acceleration_limiter: true
battery_capacity: 16.5
battery_low: 14.0
battery_dangerous: 13.2
device_port: /dev/ttyUSB0 or /dev/kobuki
cmd_vel_timeout_sec: 0.6
odom_frame: odom
base_frame: base_link
publish_tf: true
use_imu_heading: true
wheel_left_joint_name: wheel_left_joint
wheel_right_joint_name: wheel_right_joint

# Hokuyo LiDAR Configuration

Ensure the launch file uses local configuration instead of the internet. For `sllidar_ros2`, set:

scan_mode=Express port=/dev/rplidar

Define UDEV rules if not already done :
sudo nano /etc/udev/rules.d/99-ros-usb.rules

SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"

SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="kobuki", MODE="0666"

Save the file and apply the changes:

sudo udevadm control --reload-rules
sudo udevadm trigger

ln -sf /dev/ttyUSB1 /dev/rplidar
ln -sf /dev/ttyUSB0 /dev/kobuki

Run the container:

devices=$(ls /dev/video* | xargs -I{} echo "--device={}:{},")  # for realsense camera
docker run -it \
    --name kobuki_container \
    --network host \
    --privileged \                
    --env DISPLAY=$DISPLAY \
 --volume /tmp/.X11-unix:/tmp/.X11-unix \
 --volume /root/kobuki_ws:/kobuki_ws \
 --device=/dev/ttyUSB0:/dev/ttyUSB0 \
 --device=/dev/ttyUSB1:/dev/ttyUSB1 \
 --device=/dev/ttyACM0:/dev/ttyACM0 \
 --device=/dev/kobuki:/dev/kobuki \
 --device=/dev/rplidar:/dev/rplidar \
 $(echo $devices | sed 's/,$//') \
 IMAGE_NAME

# Additional Packages (NAV2 and SLAM)

Install necessary packages:
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3 ros-humble-slam-toolbox ros-humble-twist-mux ros-humble-cartographer

## Mapping

Launch SLAM and record a map:
ros2 launch slam_toolbox online_async_launch.py scan_topic:=/scan

### Configure RViz

- Set `Fixed Frame` to `map`.
- Add a map display and set it to listen to the `map` topic.

## Navigation

ros2 launch nav2_bringup navigation_launch.py
