# Copyright 2020 Intelligent Robotics Lab
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

params_file = '/config/darknet_3d.yaml'


def generate_launch_description():

    # Load params
    pkg_dir = get_package_share_directory('darknet_ros_3d')
    config_file_path = pkg_dir + params_file

    stdout_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_USE_STDOUT', '1')
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Create Node:
    darknet3d_node = Node(
        package='darknet_ros_3d',
        executable='darknet3d_node',
        name='darknet3d_node',
        output='screen',
        parameters=[config_file_path]
    )

    ld = LaunchDescription()
    ld.add_action(stdout_envvar)
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(darknet3d_node)

    return ld
