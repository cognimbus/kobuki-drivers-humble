from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Get package directories
    realsense_dir = get_package_share_directory('realsense2_camera')
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    kobuki_dir = get_package_share_directory('kobuki')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tf_to_poses_dir = get_package_share_directory('tf_to_poses')

    # Create launch description objects
    realsense_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(realsense_dir, 'launch', 'rs_launch.py')
    ),
        launch_arguments={
            'depth_width': '640',
            'depth_height': '480',
            'color_width': '640',
            'color_height': '480',
            'depth_fps': '24',
            'color_fps': '24'
        }.it
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_dir, 'launch', 'sllidar_a2m8_launch.py')
        ),
        launch_arguments={
            'scan_mode': 'Express',
            'serial_port': '/dev/rplidar'
        }.items()
    )

    kobuki_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_dir, 'launch', 'kobuki.launch.py')
        )
    )

    # Include SLAM Toolbox and Navigation
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

    # Add tf_to_poses launch
    tf_to_poses_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tf_to_poses_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'base_frame': 'base_link',
            'camera_frame': 'camera_link',
            'laser_frame': 'laser',
            'global_frame': 'map',
            'rate': '5.0'
        }.items()
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
        arguments=['0.0', '0.0', '0.3', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    from_base_to_camera_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.15', '0.0', '0.17', '0.0', '0.0', '0.0', 'base_link', 'camera_link']
    )

    return LaunchDescription([
        # Launch RPLIDAR first
        sllidar_launch,
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
        ]),
    ]) 
