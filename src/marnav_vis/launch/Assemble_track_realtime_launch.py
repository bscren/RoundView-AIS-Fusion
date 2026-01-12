from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess, RegisterEventHandler, TimerAction,
    DeclareLaunchArgument
)
from ament_index_python.packages import get_package_share_directory
import os
# 只用yaml文件配置参数的 实时启动方法

def generate_launch_description():
    # =========== 动态生成配置文件路径 ===========
    pkg_share_marnav_vis = get_package_share_directory('marnav_vis')
    pkg_share_image_stitching = get_package_share_directory('image_stitching_pkg')
    # 声明轨迹跟踪实时配置文件路径参数
    declare_track_realtime_config_file_arg = DeclareLaunchArgument(
        'track_realtime_config_file',
        default_value=os.path.join(pkg_share_marnav_vis, 'config', 'track_realtime_config.yaml'),
        description='Path to the track realtime configuration file'
    )
    
    # 声明相机发布配置文件路径参数
    declare_rtsp_config_file_arg = DeclareLaunchArgument(
        'rtsp_config_file',
        default_value=os.path.join(pkg_share_image_stitching, 'config', 'JH_rtsp_config.yaml'),
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

    # GNSS解析发布节点（来自 marnav_ais 包）
    gnss_parser_pub_node = Node(
        package='marnav_ais',  # 使用 marnav_ais 包
        executable='gnss_parser_pub_node',  # C++ 可执行文件名
        name='gnss_parser_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('track_realtime_config_file')
        }]
    )

    # AIS批量解析发布节点（来自 marnav_ais 包）
    ais_parser_batch_pub_node = Node(
        package='marnav_ais',  # 使用 marnav_ais 包
        executable='ais_parser_batch_pub_node',  # C++ 可执行文件名
        name='ais_batch_parser',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('track_realtime_config_file')
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
        declare_track_realtime_config_file_arg,
        declare_rtsp_config_file_arg,
        # 启动相机发布节点（来自 image_stitching_pkg 包）
        camera_pub_node,

        # 启动GNSS和AIS解析节点（来自 marnav_ais 包）
        gnss_parser_pub_node,
        ais_parser_batch_pub_node,
        
        # 启动DeepSORVF跟踪节点（来自 marnav_vis 包）
        deep_sorvf_node
    ])

