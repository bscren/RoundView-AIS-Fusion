from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess, RegisterEventHandler, TimerAction,
    DeclareLaunchArgument
)
# 只用yaml文件配置参数的数据集启动方法

def generate_launch_description():
    # 声明轨迹跟踪离线配置文件路径参数
    declare_track_offline_config_file_arg = DeclareLaunchArgument(
        'track_offline_config_file',
        default_value='/home/tl/RV/src/marnav_vis/config/track_offline_config.yaml',
        description='Path to the track offline configuration file'
    )

    # 声明图像拼接配置文件路径参数
    declare_stitch_config_file_arg = DeclareLaunchArgument(
        'stitch_config_file',
        default_value='/home/tl/RV/src/image_stitching_pkg/config/JH_stitch_config.yaml',
        description='Path to the Stitch configuration file'
    )
    
    # 离线相机发布节点 (来自 marnav_vis 包）
    camera_pub_node = Node(
        package='marnav_vis',
        executable='camera_pub_temporary_Test_node',
        name='camera_publisher_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('track_offline_config_file')
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
            'config_file': LaunchConfiguration('track_offline_config_file')
        }]
    )

    return LaunchDescription([
        declare_track_offline_config_file_arg,
        declare_stitch_config_file_arg,
        camera_pub_node,
        gnss_pub_node,
        ais_csv_pub_node,
        ais_sorted_pub_node,
        stitch_node,
        deep_sorvf_node # 最后启动
    ])