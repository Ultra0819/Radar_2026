import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

node_params = os.path.join(
    get_package_share_directory('rm_auto_record'), 'config', 'node_params.yaml')

def generate_launch_description():
    test_mode_arg = DeclareLaunchArgument(
        'test',
        default_value='false',
        description='Set to true to start recording immediately.'
    )
    
    rm_auto_record_node = Node(
        package='rm_auto_record',
        executable='rm_auto_record_node',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'rm_auto_record:=DEBUG'],
    )

    return LaunchDescription([test_mode_arg,rm_auto_record_node])
