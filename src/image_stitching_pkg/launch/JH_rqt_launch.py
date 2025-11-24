from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 启动rqt
    start_rqt = ExecuteProcess(
        cmd=['rqt', '--force-discover'],
        output='screen'
    )
    
    return LaunchDescription([
        start_rqt
    ])