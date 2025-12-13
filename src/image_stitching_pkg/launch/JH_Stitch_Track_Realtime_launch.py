from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess, RegisterEventHandler, TimerAction,
    DeclareLaunchArgument
)
# 只用yaml文件配置参数的数据集启动方法

def generate_launch_description():
    # 声明轨迹跟踪实时配置文件路径参数
    declare_track_realtime_config_file_arg = DeclareLaunchArgument(
        'track_realtime_config_file',
        default_value='/home/tl/RV/src/marnav_vis/config/track_realtime_config.yaml',
        description='Path to the track realtime configuration file'
    )

    # 声明相机实时发布配置文件路径参数
    declare_rtsp_config_file_arg = DeclareLaunchArgument(
        'rtsp_config_file',
        default_value='/home/tl/RV/src/image_stitching_pkg/config/JH_rtsp_config.yaml',
        description='Path to the RTSP configuration file'
    )

    # 声明图像拼接配置文件路径参数
    declare_stitch_config_file_arg = DeclareLaunchArgument(
        'stitch_config_file',
        default_value='/home/tl/RV/src/image_stitching_pkg/config/JH_stitch_config.yaml',
        description='Path to the Stitch configuration file'
    )
    
    # 实时相机发布节点 (来自 image_stitching_pkg 包）
    camera_pub_node = Node(
        package='image_stitching_pkg',
        executable='JH_rtsp',
        name='JH_rtsp',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('rtsp_config_file')
        }]
    )

    # 启动图像拼接节点
    stitch_node = Node(
        package='image_stitching_pkg',
        executable='JH_ROS_stitch',
        name='JH_ROS_stitch',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('stitch_config_file')
        }]
    )

    # GNSS发布节点
    gnss_pub_node = Node(
        package='marnav_ais',
        executable='gnss_parser_pub_node',
        name='gnss_parser_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('track_realtime_config_file')
        }]
    )

    # AIS批量发布节点
    ais_sorted_pub_node = Node(
        package='marnav_ais',
        executable='ais_parser_batch_pub_node',
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

    return LaunchDescription([
        declare_track_realtime_config_file_arg,
        declare_rtsp_config_file_arg,
        declare_stitch_config_file_arg,

        camera_pub_node, # 先启动相机发布节点
        stitch_node, # 再启动图像拼接节点
        gnss_pub_node, # 再启动GNSS发布节点
        ais_sorted_pub_node, # 再启动AIS批量发布节点
        deep_sorvf_node # 最后启动DeepSORVF_ros_v节点
    ])