import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 声明 Launch 参数
    # port: 串口设备路径
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='裁判系统 VTM 板对应的物理串口路径'
    )
    
    # force_color: 阵营配置 (-1: 实战模式/依赖裁判系统下发; 0: 测试模式强制蓝方; 2: 测试模式强制红方)
    force_color_arg = DeclareLaunchArgument(
        'force_color',
        default_value='-1',
        description='强制指定阵营。-1: 实战模式, 0: 测试蓝方, 2: 测试红方'
    )

    # 2. 配置节点
    serial_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='rm_serial_driver',
        output='screen',
        # 将 Launch 参数传递给 C++ 节点的 Parameter 系统
        parameters=[{
            'serial_port': LaunchConfiguration('port'),
            'force_self_color': LaunchConfiguration('force_color')
        }]
    )

    decision_node = Node(
        package='rm_serial_driver',
        executable='vulnerability_decision_node',
        name='vulnerability_decision_node',
        output='screen'
    )

    # 3. 返回 Launch 描述
    return LaunchDescription([
        port_arg,
        force_color_arg,
        serial_node,
        decision_node
    ])