# Copyright (c) 2024 Intelligent Robotics Lab (URJC)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Modified by Juan Carlos Manzanares Serrano

import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
import yaml


def get_model_paths(packages_names):
    model_paths = ''
    for package_name in packages_names:
        if model_paths != '':
            model_paths += os.pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, 'share')

        model_paths += model_path

    return model_paths


def generate_launch_description():
    package_dir = get_package_share_directory('kobuki')

    params_file = os.path.join(package_dir, 'config', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        kobuki_params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='')

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    kobuki_cmd = Node(
        package='kobuki_node',
        executable='kobuki_ros_node',
        namespace=namespace,
        output='screen',
        parameters=[kobuki_params],
        remappings=[
            ('/commands/velocity', '/cmd_vel'), ('/tf', 'tf'), ('/tf_static', 'tf_static')
        ]
    )

    ld.add_action(kobuki_cmd)

    return ld
