from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess, RegisterEventHandler, TimerAction,
    DeclareLaunchArgument
)
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os

# 只用yaml文件配置参数的数据集启动方法

def generate_launch_description():
    # =========== 动态生成配置文件路径 ===========
    pkg_share_image_stitching = get_package_share_directory('image_stitching_pkg')
    rtsp_config_file_path = os.path.join(pkg_share_image_stitching, 'config', 'JH_rtsp_config.yaml')
    stitch_config_file_path = os.path.join(pkg_share_image_stitching, 'config', 'JH_stitch_config.yaml')

    # 声明配置文件路径参数
    declare_rtsp_config_file_arg = DeclareLaunchArgument(
        'rtsp_config_file',
        default_value=rtsp_config_file_path,
        description='Path to the RTSP configuration file'
    )
    declare_stitch_config_file_arg = DeclareLaunchArgument(
        'stitch_config_file',
        default_value=stitch_config_file_path,
        description='Path to the Stitch configuration file'
    )
    
    # 启动rtsp节点
    rtsp_node = Node(
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

    return LaunchDescription([
        declare_rtsp_config_file_arg,
        declare_stitch_config_file_arg,
        rtsp_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=rtsp_node,  # 监听具体的rtsp_node节点
                on_start=[TimerAction(period=5.0, actions=[stitch_node])]
            )
        )
    ])