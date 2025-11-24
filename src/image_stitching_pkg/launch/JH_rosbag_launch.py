from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # 声明bag文件路径参数（设置默认值和描述）
    default_bag_path = os.path.expanduser('~/RV/ROS2BAG/multi_camera_record_2')
    declare_bag_path = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag_path,  # 关键：设置默认路径
        description='要播放的bag文件路径（默认：~/RV/ROS2BAG/multi_camera_record_2）'
    )
    
    # 循环播放bag文件
    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-l', LaunchConfiguration('bag_path')],
        output='screen',
        cwd=os.path.expanduser('~/RV/ROS2BAG/')  # 工作目录
    )
    
    return LaunchDescription([
        declare_bag_path,
        play_bag
    ])
