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

    # URG Lidar (Qualcomm specific)
    urg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(urg_dir, 'launch', 'urg_node2.launch.py')
        )
    )

    # Common configurations for both robots
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'depth_width': '640',
            'depth_height': '480',
            'color_width': '640',
            'color_height': '480'
        }.items()
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
            'max_laser_range': '10.0',  # Increased range for better mapping
            'transform_timeout': '0.1',  # Reduced timeout for faster processing
            'update_rate': '10.0',  
            'enable_interactive_mode': 'false',  # Disable interactive mode for better performance
            'use_pose_extrapolator': 'true',  # Enable pose extrapolation for smoother mapping
            'scan_topic': 'scan',  # Explicitly set scan topic
            'stack_size_to_use': '40000000'  # Increased stack size for better performance
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'controller_frequency': '10.0',
            'planner_server_rate': '5.0',
            'controller_server_rate': '10.0',
            'global_costmap_publish_rate': '2.0',
            'local_costmap_publish_rate': '5.0'
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
            'default_topic': '/cmd_vel_nav'
        }],
        remappings=[
            ('/nav2/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_robot', '/cmd_vel')
        ]
    )

    return LaunchDescription([
        # Launch URG first
        urg_launch,
        # Launch RealSense after 2 seconds
        TimerAction(period=2.0, actions=[realsense_launch]),
        # Launch Kobuki after 4 seconds
        TimerAction(period=4.0, actions=[kobuki_launch]),
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