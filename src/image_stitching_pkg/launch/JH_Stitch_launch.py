from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess, RegisterEventHandler, TimerAction,
    DeclareLaunchArgument
)
from launch.event_handlers import OnProcessStart

# 只用yaml文件配置参数的数据集启动方法

def generate_launch_description():
    # 声明配置文件路径参数
    declare_rtsp_config_file_arg = DeclareLaunchArgument(
        'rtsp_config_file',
        default_value='/home/tl/RV/src/image_stitching_pkg/config/JH_rtsp_config.yaml',
        description='Path to the RTSP configuration file'
    )
    declare_stitch_config_file_arg = DeclareLaunchArgument(
        'stitch_config_file',
        default_value='/home/tl/RV/src/image_stitching_pkg/config/JH_stitch_config.yaml',
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