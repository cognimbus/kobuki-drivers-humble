from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import os

def generate_launch_description():
    # Get package directories
    realsense_dir = get_package_share_directory('realsense2_camera')
    kobuki_dir = get_package_share_directory('kobuki')
    urg_dir = get_package_share_directory('urg_node2')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tf_to_poses_dir = get_package_share_directory('tf_to_poses')

    # Create launch description objects (same as your code)
    urg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(urg_dir, 'launch', 'urg_node2.launch.py')
        )
    )

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
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
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

    # Retry script for URG
    retry_urg_script = ExecuteProcess(
        cmd=['bash', '-c', '''
            while true; do
                if ! ros2 topic echo /scan -n 1 > /dev/null 2>&1; then
                    echo "No /scan topic found. Starting URG node directly..."
                    ros2 run urg_node2 urg_node2_node &
                    sleep 5
                else
                    echo "URG lidar is working!"
                    sleep 1
                fi
            done
        '''],
        output='screen'
    )

    return LaunchDescription([
        # Launch URG with retry mechanism
        urg_launch,
        retry_urg_script,
        
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