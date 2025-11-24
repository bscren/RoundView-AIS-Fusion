from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明所有可配置参数
    declare_refresh_time = DeclareLaunchArgument(
        'refresh_time',
        default_value='2',
        description='拼缝线更新时间间隔(秒)'
    )
    
    declare_min_keypoints = DeclareLaunchArgument(
        'min_keypoints',
        default_value='50',
        description='最小关键点数量阈值'
    )
    
    declare_min_confidence = DeclareLaunchArgument(
        'min_confidence',
        default_value='0.4',
        description='特征匹配最小置信度'
    )
    
    declare_min_inliers = DeclareLaunchArgument(
        'min_inliers',
        default_value='50',
        description='最小内点数量阈值'
    )
    
    declare_max_focal_variance = DeclareLaunchArgument(
        'max_focal_variance',
        default_value='50000.0',
        description='最大焦距方差阈值'
    )
    
    declare_y_tolerance = DeclareLaunchArgument(
        'y_tolerance',
        default_value='200.0',
        description='y坐标允许浮动范围'
    )
    
    declare_roi_threshold = DeclareLaunchArgument(
        'roi_threshold',
        default_value='0.95',
        description='ROI区域白色比例阈值'
    )
    
    declare_iou_threshold = DeclareLaunchArgument(
        'iou_threshold',
        default_value='0.5',
        description='检测框IOU筛选阈值'
    )
    
    declare_scale = DeclareLaunchArgument(
        'scale',
        default_value='0.75',
        description='图像缩放比例'
    )
    
    declare_cropornot = DeclareLaunchArgument(
        'cropornot',
        default_value='true',
        description='是否裁剪全景图'
    )
    
    declare_drawDetectionornot = DeclareLaunchArgument(
        'drawDetectionornot',
        default_value='false',
        description='是否绘制检测框'
    )

    # 定义节点
    stitch_node = Node(
        package='image_stitching_pkg',  # 实际包名
        executable='JH_stitch',  # 实际可执行文件名
        name='topic_stitch',
        output='screen',
        parameters=[{
            'refresh_time': LaunchConfiguration('refresh_time'),
            'min_keypoints': LaunchConfiguration('min_keypoints'),
            'min_confidence': LaunchConfiguration('min_confidence'),
            'min_inliers': LaunchConfiguration('min_inliers'),
            'max_focal_variance': LaunchConfiguration('max_focal_variance'),
            'y_tolerance': LaunchConfiguration('y_tolerance'),
            'roi_threshold': LaunchConfiguration('roi_threshold'),
            'iou_threshold': LaunchConfiguration('iou_threshold'),
            'scale': LaunchConfiguration('scale'),
            'cropornot': LaunchConfiguration('cropornot'),
            'drawDetectionornot': LaunchConfiguration('drawDetectionornot')
        }],
        remappings=[
            ('/rtsp_image_0', LaunchConfiguration('image_topic_0', default='/rtsp_image_0')),
            ('/rtsp_image_1', LaunchConfiguration('image_topic_1', default='/rtsp_image_1')),
            ('/rtsp_image_2', LaunchConfiguration('image_topic_2', default='/rtsp_image_2')),
            ('/yolo/detection_results', LaunchConfiguration('detection_topic', default='/yolo/detection_results')),
            ('image_topic_all', LaunchConfiguration('output_topic', default='image_topic_all'))
        ]
    )

    return LaunchDescription([
        # 声明的参数
        declare_refresh_time,
        declare_min_keypoints,
        declare_min_confidence,
        declare_min_inliers,
        declare_max_focal_variance,
        declare_y_tolerance,
        declare_roi_threshold,
        declare_iou_threshold,
        declare_scale,
        declare_cropornot,
        declare_drawDetectionornot,
        
        # 节点
        stitch_node
    ])