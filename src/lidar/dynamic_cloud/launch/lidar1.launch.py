import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, Shutdown, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition, UnlessCondition

# def dump_params(param_file_path, node_name):
#     with open(param_file_path, 'r') as file:
#         return [yaml.safe_load(file)[node_name]['ros__parameters']]

def generate_launch_description():

    # --- 确认模式 ---
    # 声明启动参数 'use_lidar'，默认值为 'true'
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='false',
        description='Whether to start the Livox LiDAR driver'
    )
    use_lidar = LaunchConfiguration('use_lidar')

    # --- 驱动 ---
    livox_driver_launch_dir = os.path.join(
        get_package_share_directory('livox_ros2_driver'), 'launch')
    
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_driver_launch_dir, 'livox_lidar_launch.py')
        ),
        condition=IfCondition(use_lidar)
    )

    # --- 识别节点定义 ---
    def get_localization_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='localization_node',
            extra_arguments=[{'use_intra_process_comms': True},
                             {'use_multi_threaded_executor': True}],
        )
        
    def get_dynamic_cloud_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='dynamic_cloud_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_cluster_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='cluster_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_kalman_filter_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='kalman_filter_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    # 3. 定义容器
    lidar_detector_container = ComposableNodeContainer(
        name='lidar_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            get_localization_node('localization', 'tdt_radar::Localization'),
            get_dynamic_cloud_node('dynamic_cloud', 'tdt_radar::DynamicCloud'),
            get_cluster_node('cluster', 'tdt_radar::Cluster'),
            get_kalman_filter_node('kalman_filter', 'tdt_radar::KalmanFilter')
        ],
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    # 4. 使用 TimerAction 延迟启动识别节点
    delayed_lidar_detector = TimerAction(
        period=2.0,
        actions=[lidar_detector_container],
        condition=IfCondition(use_lidar)
    )
    # 当不启动驱动时，直接启动识别容器
    direct_lidar_detector = TimerAction(
        period=0.0,
        actions=[lidar_detector_container],
        condition=UnlessCondition(use_lidar)
    )
    
    return LaunchDescription([
            use_lidar_arg,
            livox_driver,
            delayed_lidar_detector,
            direct_lidar_detector
            ])