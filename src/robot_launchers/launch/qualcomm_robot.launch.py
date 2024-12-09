from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import time
import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.qos import QoSProfile
import sensor_msgs.msg

def check_scan_topic(context, *args, **kwargs):
    rclpy.init(args=None)
    node = RclpyNode('scan_checker')
    qos = QoSProfile(depth=10)
    scan_received = False

    def scan_callback(msg):
        nonlocal scan_received
        scan_received = True

    subscription = node.create_subscription(
        sensor_msgs.msg.LaserScan,
        '/scan',
        scan_callback,
        qos
    )

    # Wait for a message on the /scan topic
    start_time = time.time()
    while not scan_received and (time.time() - start_time) < 2:  # 2 seconds timeout
        rclpy.spin_once(node, timeout_sec=1)

    node.destroy_node()
    rclpy.shutdown()

    if not scan_received:
        node.get_logger().warn('No messages received on /scan topic. Attempting to launch URG node.')
        # Launch the URG node
        urg_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('urg_node2'), 'launch', 'urg_node2.launch.py')
            )
        )
        urg_launch.execute(context)
    else:
        node.get_logger().info('Successfully receiving messages on /scan topic')

def generate_launch_description():
    # Get package directories
    realsense_dir = get_package_share_directory('realsense2_camera')
    kobuki_dir = get_package_share_directory('kobuki')
    urg_dir = get_package_share_directory('urg_node2')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tf_to_poses_dir = get_package_share_directory('tf_to_poses')

    # Create launch description objects
    kobuki_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_dir, 'launch', 'kobuki.launch.py')
        )
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        )
    )

    tf_to_poses_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tf_to_poses_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'base_frame': 'base_link',
            'camera_frame': 'camera_link',
            'laser_frame': 'laser', 
            'global_frame': 'map',
            'rate': '10.0'  
        }.items()
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        )
    )

    urg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(urg_dir, 'launch', 'urg_node2.launch.py')
        )
    )

    # Add transformations
    tf_footprint2base_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'base_footprint']
    )

    fake_bumper_cmd = Node(
        package='kobuki',
        executable='fake_bumer_node',
        output='screen'
    )

    from_base_to_lidar_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.4', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    from_base_to_camera_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.15', '0.0', '0.17', '0.0', '0.0', '0.0', 'base_link', 'camera_link']
    )

    return LaunchDescription([
        kobuki_launch,
        TimerAction(period=2.0, actions=[realsense_launch]),
        TimerAction(period=4.0, actions=[slam_toolbox_launch]),
        TimerAction(period=6.0, actions=[tf_to_poses_launch]),
        TimerAction(period=8.0, actions=[nav2_launch]),
        TimerAction(period=10.0, actions=[OpaqueFunction(function=check_scan_topic)]),
        urg_launch,
        tf_footprint2base_cmd,
        fake_bumper_cmd,
        from_base_to_lidar_cmd,
        from_base_to_camera_cmd,
    ])