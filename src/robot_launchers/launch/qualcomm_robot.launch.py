from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    realsense_dir = get_package_share_directory('realsense2_camera')
    kobuki_dir = get_package_share_directory('kobuki')
    urg_dir = get_package_share_directory('urg_node2')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tf_to_poses_dir = get_package_share_directory('tf_to_poses')

    # Create launch description objects
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        )
    )

    kobuki_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_dir, 'launch', 'kobuki.launch.py')
        )
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'max_laser_range': '8.0',
            'resolution': '0.05',
            'transform_timeout': '0.2',
            'update_rate': '2.0',
            'enable_interactive_mode': 'false',
            'use_pose_extrapolator': 'true',
            'scan_topic': 'scan',
            'stack_size_to_use': '40000000',
            'minimum_time_interval': '1.0',
            'max_queue_size': '50',
            'throttle_scans': '2',
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'controller_frequency': '10.0',
            'planner_server_rate': '10.0',
            'controller_server_rate': '10.0',
            'global_costmap_publish_rate': '2.0',
            'local_costmap_publish_rate': '10.0',
            'bt_navigator_rate': '10.0',
            'bt_loop_duration': '100',
            'default_server_timeout': '60.0',
            'recovery_enabled': 'true',
            'min_recovery_wait_time': '10.0'
        }.items()
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

    # Common transformations and nodes
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

    cmd_vel_mux = Node(
        package='topic_tools',
        executable='mux',
        name='cmd_vel_mux',
        parameters=[{
            'input_topics': ['/cmd_vel_nav', '/cmd_vel'],
            'output_topic': '/cmd_vel_robot',
            'default_topic': '/cmd_vel'
        }],
        remappings=[
            ('/nav2/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_robot', '/cmd_vel')
        ]
    )

    # URG launch configuration
    urg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(urg_dir, 'launch', 'urg_node2.launch.py')
        )
    )

    return LaunchDescription([
        # Launch Kobuki first
        kobuki_launch,
        # Launch URG after 2 seconds + 5 seconds internal delay
        TimerAction(period=2.0, actions=[
            TimerAction(
                period=5.0,
                actions=[urg_launch]
            )
        ]),
        # Launch RealSense after 4 seconds
        TimerAction(period=4.0, actions=[realsense_launch]),
        # Launch all remaining nodes in parallel after 6 seconds
        TimerAction(period=6.0, actions=[
            tf_footprint2base_cmd,
            fake_bumper_cmd,
            from_base_to_lidar_cmd,
            from_base_to_camera_cmd,
            tf_to_poses_launch,
            slam_toolbox_launch,
            nav2_launch,
            cmd_vel_mux,
        ]),
    ])
