from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 新增
from launch.substitutions import PathJoinSubstitution  # 新增
from launch_ros.substitutions import FindPackageShare  # 新增
import os

def generate_launch_description():
    # 定义路径
    bag_path = os.path.expanduser('~/RV/ROS2BAG/multi_camera_record_2/')
    # 修改拼接节点的工作目录为你的实际运行路径~/RV
    stitch_workspace_path = os.path.expanduser('~/RV')
    
    # 1. 播放bag文件 (循环播放)
    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-l', bag_path],
        output='screen',
        cwd=os.path.expanduser('~/RV/ROS2BAG/')  # 在bag文件所在目录执行
    )
    
    # 2. 运行检测节点
    yolo_detect = Node(
        package='yolo_detect',
        executable='yolo_inference',
        output='screen',
        cwd=stitch_workspace_path
    )

    # # 3. 运行检测节点
    # yolo_detect_saver = Node(
    #     package='yolo_detect',
    #     executable='detection_saver',
    #     output='screen',
    #     cwd=stitch_workspace_path
    # )

    
    # 4. 运行拼接节点（使用修改后的工作目录）
    run_stitch = Node(
        package='image_stitching_pkg',
        executable='JH_stitch',#JH_stitch,stitch
        output='screen',
        cwd=stitch_workspace_path  # 改为在~/RV目录执行，与你的手动操作一致
    )
    
    # 5. 启动rqt显示图像
    start_rqt = ExecuteProcess(
        cmd=['rqt', '--force-discover'],
        output='screen'
    )
    
    # 定义执行顺序
    return LaunchDescription([
        play_bag,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=play_bag,
                on_start=[TimerAction(period=1.0, actions=[yolo_detect])]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=yolo_detect,
                on_start=[TimerAction(period=1.0, actions=[run_stitch])]
            )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessStart(
        #         target_action=yolo_detect_saver,
        #         on_start=[TimerAction(period=1.0, actions=[run_stitch])]
        #     )
        # ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=run_stitch,
                on_start=[TimerAction(period=2.0, actions=[start_rqt])]
            )
        )
    ])
