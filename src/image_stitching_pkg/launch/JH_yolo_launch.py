from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # 声明工作空间路径参数
    default_yolo_path = os.path.expanduser('~/RV')
    declare_workspace = DeclareLaunchArgument(
        'yolo_workspace_path',
        default_value=default_yolo_path,  # 关键：设置默认路径
        description='拼接工作空间路径'
    )
    
    # 启动yolo检测节点
    yolo_detect = Node(
        package='yolo_detect',
        executable='yolo_inference',
        output='screen',
        cwd=LaunchConfiguration('stitch_workspace_path')
    )
    
    return LaunchDescription([
        declare_workspace,
        yolo_detect
    ])