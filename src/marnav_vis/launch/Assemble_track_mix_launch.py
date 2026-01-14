from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess, RegisterEventHandler, TimerAction,
    DeclareLaunchArgument
)
from ament_index_python.packages import get_package_share_directory
import os

# 只用yaml文件配置参数的数据集启动方法

def generate_launch_description():
    # =========== 动态生成配置文件路径 ===========
    pkg_share_marnav_vis = get_package_share_directory('marnav_vis')
    pkg_share_image_stitching = get_package_share_directory('image_stitching_pkg')
    # 声明配置文件路径参数
    declare_track_offline_config_file_arg = DeclareLaunchArgument(
        'track_offline_config_file',
        default_value=os.path.join(pkg_share_marnav_vis, 'config', 'track_offline_config.yaml'),
        # default_value='/home/tl/RV/src/marnav_vis/config/track_offline_config.yaml',
        description='Path to the YAML configuration file'
    )
    declare_track_realtime_config_file_arg = DeclareLaunchArgument(
        'track_realtime_config_file',
        default_value=os.path.join(pkg_share_marnav_vis, 'config', 'track_realtime_config.yaml'),
        # default_value='/home/tl/RV/src/marnav_vis/config/track_realtime_config.yaml',
        description='Path to the YAML configuration file'
    )
    declare_rtsp_config_file_arg = DeclareLaunchArgument(
        'rtsp_config_file',
        default_value=os.path.join(pkg_share_image_stitching, 'config', 'JH_rtsp_config.yaml'),
        # default_value='/home/tl/RV/src/image_stitching_pkg/config/JH_rtsp_config.yaml',
        description='Path to the RTSP configuration file'
    )
    # 相机发布节点(来自 image_stitching_pkg 包）
    camera_pub_node = Node(
        package='image_stitching_pkg',
        executable='JH_rtsp',
        name='camera_publisher_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('rtsp_config_file')
        }]
    )

    # GNSS发布节点
    gnss_pub_node = Node(
        package='marnav_vis',
        executable='gnss_pub_node',
        name='gnss_publisher_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('track_offline_config_file')
        }]
    )

    # AIS CSV发布节点
    ais_csv_pub_node = Node(
        package='marnav_vis',
        executable='ais_csv_pub_node',
        name='ais_csv_publisher_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('track_offline_config_file')
        }]
    )

    # AIS批量发布节点
    ais_sorted_pub_node = Node(
        package='marnav_vis',
        executable='ais_sorted_pub_node',
        name='ais_batch_publisher_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('track_offline_config_file')
        }]
    )

    # DeepSORVF_ros_v节点
    deep_sorvf_node = Node(
        package='marnav_vis',
        executable='DeepSORVF_JH',
        name='ais_vis_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('track_realtime_config_file')
        }]
    )

    # 组装启动描述
    return LaunchDescription([
        declare_track_offline_config_file_arg,
        declare_track_realtime_config_file_arg,
        declare_rtsp_config_file_arg,
        camera_pub_node,
        gnss_pub_node,
        ais_csv_pub_node,
        ais_sorted_pub_node,
        deep_sorvf_node
    ])

