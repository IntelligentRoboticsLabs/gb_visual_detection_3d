from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

params_file = '/config/darknet_3d.yaml'

def generate_launch_description():

    # Load params
    pkg_dir = get_package_share_directory('darknet_ros_3d')
    config_file_path = pkg_dir + params_file

    # Create Node:
    darknet3d_node = Node(
    package='darknet_ros_3d',
    node_executable='darknet3d_node',
    node_name='darknet3d_node',
    output='screen',
    parameters=[config_file_path]
    )

    ld = LaunchDescription()
    ld.add_action(darknet3d_node)

    return ld
