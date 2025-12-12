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
    # 1.1 bag路径参数
    default_bag_path = os.path.expanduser('~/ROS2BAG/ais_rtsp_rosbag_20250912_095732.bag')
    declare_bag_path = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag_path,
        description='要播放的bag文件路径'
    )
    
    # 1.2 yolo工作空间参数
    default_yolo_path = os.path.expanduser('~/RV')
    declare_yolo_workspace = DeclareLaunchArgument(
        'yolo_workspace_path',
        default_value=default_yolo_path,
        description='YOLO工作空间路径'
    )
    
    # 2. 定义具体动作（进程/节点）
    # 2.1 播放bag
    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-l', LaunchConfiguration('bag_path')],
        output='screen',
        cwd=os.path.expanduser('~/RV/ROS2BAG/')
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
            'min_inliers': 28, # 最小内点数
            'max_focal_variance': 50000.0, # 最大焦距方差
            'y_tolerance': 200.0, # 拼接图像间的y轴容差
            'roi_threshold': 0.95, # ROI阈值
            'detect_confidence': 0.3, # 检测置信度
            'iou_threshold': 0.1, # IOU阈值
            'scale': 0.4, # 图像缩放比例
            'cropornot': True, # 是否裁剪全景图
            'drawboxornot': True, # 是否绘制检测框
            'save_CameraParams': False,
            'save_CameraParams_path': "/home/tl/RV/src/image_stitching_pkg/config/CameraParams_LongDistance.yaml",
            'use_saved_CameraParams': True,
            'FOV_hor': 105.0,
            'FOV_ver': 57.0
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
        declare_bag_path,
        declare_yolo_workspace,
        
        # 启动bag播放
        play_bag,
        
        # 等待bag启动后1秒启动YOLO
        RegisterEventHandler(
            OnProcessStart(
                target_action=play_bag,  # 监听具体的play_bag进程
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



# —————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
# 另一種嵌套的launch寫法

# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
# from launch.event_handlers import OnProcessStart

# import os

# def generate_launch_description():
#     # 1. 图像拼接节点的 launch 配置
#     stitcher_launch_path = os.path.join(
#         get_package_share_directory('image_stitching_pkg'),
#         'launch/JH_ros_stitcher.py'
#     )

#     stitcher_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(stitcher_launch_path),
#         launch_arguments={
#             # 配置参数
#             'refresh_time': '2',  # 更新时间为2秒
#             'min_keypoints': '100',  # 提高关键点阈值
#             'min_confidence': '0.5',  # 提高置信度阈值
#             'min_inliers':'50', #最小内点数量阈值
#             'max_focal_variance':'50000.0', #最大焦距方差阈值
#             'y_tolerance':'200.0', #y坐标允许浮动范围
#             'roi_threshold':'0.95', #ROI区域白色比例阈值
#             'iou_threshold':'0.5', #检测框IOU筛选阈值
#             'scale':'0.75', #图像缩放比例
#             'cropornot': 'true',  # 不裁剪全景图
#             'filtornot': 'false',  # 过滤检测框
#             # 话题重映射参数
#             'image_topic_0': '/rtsp_image_0',  # 自定义输入话题0
#             'image_topic_1': '/rtsp_image_1',  # 自定义输入话题1
#             'image_topic_2': '/rtsp_image_2',  # 自定义输入话题2
#             'detection_topic': '/yolo/detection_results',  # 自定义检测结果话题
#             'output_topic': '/image_topic_all'  # 自定义输出话题
#         }.items()
#     )
    
#     # 2. 播放 bag 的 launch 配置
#     play_bag_launch_path = os.path.join(
#         get_package_share_directory('image_stitching_pkg'),  # 替换为实际包名
#         'launch', 'JH_rosbag_launch.py'
#     )
#     play_bag_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             play_bag_launch_path
#         ),
#         launch_arguments={'bag_path': os.path.expanduser('~/RV/ROS2BAG/multi_camera_record_2')}.items() # 替换为实际ROS2包名
#     )
    
#     # 3. YOLO 检测节点的 launch 配置
#     yolo_detect_launch_path = os.path.join(
#         get_package_share_directory('image_stitching_pkg'),  # 替换为实际包名
#         'launch', 'JH_yolo_launch.py'
#     )
#     yolo_detect_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             yolo_detect_launch_path
#         ),
#         launch_arguments={'yolo_workspace_path': os.path.expanduser('~/RV')}.items()
#     )
    
#     # 4. 启动 rqt 的 launch 配置
#     start_rqt_launch_path = os.path.join(
#         get_package_share_directory('image_stitching_pkg'),  # 替换为实际包名
#         'launch', 'JH_rqt_launch.py'
#     )
#     start_rqt_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             start_rqt_launch_path
#         )
#     )


#     # return LaunchDescription([
#     #     play_bag_launch,
#     #     yolo_detect_launch,
#     #     stitcher_launch,
#     #     start_rqt_launch
#     # ])

#     return LaunchDescription([
#             # 先启动bag播放
#             play_bag_launch,
            
#             # 等待bag启动后1秒再启动YOLO检测
#             RegisterEventHandler(
#                 event_handler=OnProcessStart(
#                     target_action=play_bag_launch,
#                     on_start=[TimerAction(period=1.0, actions=[yolo_detect_launch])]
#                 )
#             ),
            
#             # 等待YOLO启动后1秒再启动拼接节点（可选，根据依赖关系）
#             RegisterEventHandler(
#                 event_handler=OnProcessStart(
#                     target_action=yolo_detect_launch,
#                     on_start=[TimerAction(period=1.0, actions=[stitcher_launch])]
#                 )
#             ),
            
#             # 等待拼接节点启动后2秒再启动rqt（可选）
#             RegisterEventHandler(
#                 event_handler=OnProcessStart(
#                     target_action=stitcher_launch,
#                     on_start=[TimerAction(period=2.0, actions=[start_rqt_launch])]
#                 )
#             )
#         ])