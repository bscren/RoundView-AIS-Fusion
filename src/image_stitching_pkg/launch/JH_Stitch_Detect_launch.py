from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess, RegisterEventHandler, TimerAction,
    DeclareLaunchArgument
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. 声明参数

    
    # 1.2 yolo工作空间参数
    default_yolo_path = os.path.expanduser('~/RV')
    declare_yolo_workspace = DeclareLaunchArgument(
        'yolo_workspace_path',
        default_value=default_yolo_path,
        description='YOLO工作空间路径'
    )
    
    # 2. 启动相机
    # 2.1 ros2 run image_stitching_pkg JH_rtsp 1 2 3
    play_rtsp = ExecuteProcess(
        cmd=['ros2', 'run', 'image_stitching_pkg', 'JH_rtsp'],
        output='screen',
    )
    
    # 2.2 图像拼接节点
    stitch_node = Node(
        package='image_stitching_pkg',
        executable='JH_ROS_stitch',
        name='topic_stitch',
        output='screen',
    )
    
    # 2.4 启动rqt
    start_rqt = ExecuteProcess(
        cmd=['rqt', '--force-discover'],
        output='screen'
    )
    
    # 3. 定义启动顺序（事件监听）
    return LaunchDescription([
        # 声明参数
        declare_yolo_workspace,
        
        # 启动rtsp播放
        play_rtsp,
            
        # 等待play_rtsp启动后1秒启动拼接节点
        RegisterEventHandler(
            OnProcessStart(
                target_action=play_rtsp,  # 监听具体的play_rtsp节点
                on_start=[TimerAction(period=1.0, actions=[stitch_node])]
            )
        ),
        
        # 等待拼接节点启动后2秒启动rqt
        RegisterEventHandler(
            OnProcessStart(
                target_action=stitch_node,  # 监听具体的stitch_node节点
                on_start=[TimerAction(period=2.0, actions=[start_rqt])]
            )
        )
    ])

