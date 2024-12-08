from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    return LaunchDescription([
        
        DeclareLaunchArgument('base_frame', default_value='base_link',description='Base frame of the robot'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link',description='Camera frame'),
        DeclareLaunchArgument('laser_frame', default_value='laser',description='Laser frame'),  
        DeclareLaunchArgument('global_frame', default_value='map',description='Global reference frame'),  
        DeclareLaunchArgument('rate', default_value='10.0',description='Rate of pose publishing'),

     
        
        Node(
            package='tf_to_poses',
            executable='tf_to_poses_node',            
            name='tf_to_poses_node',
            parameters=[{'base_frame': LaunchConfiguration('base_frame')},
                {'camera_frame': LaunchConfiguration('camera_frame')},
                {'laser_frame': LaunchConfiguration('laser_frame')},
                {'global_frame': LaunchConfiguration('global_frame')},
                {'rate': LaunchConfiguration('rate')}]

        )
       
        
    ])