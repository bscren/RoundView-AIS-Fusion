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
    
    # 2. 定义具体动作（进程/节点）
    # 2.1 ros2 run image_stitching_pkg JH_rtsp 1 2 3
    play_rtsp = ExecuteProcess(
        cmd=['ros2', 'run', 'image_stitching_pkg', 'JH_rtsp', '1', '2', '3'],
        output='screen',
    )
    
    # 2.2 YOLO检测节点
    yolo_detect = Node(
        package='yolo_detect',
        executable='yolo_inference',
        output='screen',
        cwd=LaunchConfiguration('yolo_workspace_path')
    )
    
    # 2.3 图像拼接节点
    stitch_node = Node(
        package='image_stitching_pkg',
        executable='JH_ROS_stitch',
        name='topic_stitch',
        output='screen',
        parameters=[{
            'refresh_time': 5, # 刷新时间间隔，单位秒
            'min_keypoints': 100, # 最小关键点数
            'min_confidence': 0.5, # 匹配对最小置信度
            'min_inliers': 10, # 最小内点数
            'max_focal_variance': 50000.0, # 最大焦距方差
            'y_tolerance': 200.0, # 拼接图像间的y轴容差
            'roi_threshold': 0.95, # ROI阈值
            'detect_confidence': 0.05, # 检测置信度
            'iou_threshold': 0.1, # IOU阈值
            'scale': 0.5, # 图像缩放比例
            'cropornot': True, # 是否裁剪全景图
            'drawboxornot': True, # 是否绘制检测框
            'save_CameraParams': False, # 是否保存相机K R T参数
            'save_CameraParams_path': "/home/tl/RV/src/image_stitching_pkg/config/CameraParams_SiteTest.yaml",
            'use_saved_CameraParams': True, # 是否使用已保存的相机K R T参数
            'FOV_hor': 105.0, # 水平视场角
            'FOV_ver': 57.0 # 垂直视场角
        }],
        remappings=[
            ('/rtsp_image_0', '/rtsp_image_0'),# 左->右
            ('/rtsp_image_1', '/rtsp_image_1'),
            ('/rtsp_image_2', '/rtsp_image_2'),
            ('/yolo/detection_results', '/yolo/detection_results'),
            ('image_topic_all', '/image_topic_all')
        ]
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
        
        # 等待bag启动后1秒启动YOLO
        RegisterEventHandler(
            OnProcessStart(
                target_action=play_rtsp,  # 监听具体的play_bag进程
                on_start=[TimerAction(period=1.0, actions=[yolo_detect])]
            )
        ),
        
        # 等待YOLO启动后1秒启动拼接节点
        RegisterEventHandler(
            OnProcessStart(
                target_action=yolo_detect,  # 监听具体的yolo_detect节点
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

