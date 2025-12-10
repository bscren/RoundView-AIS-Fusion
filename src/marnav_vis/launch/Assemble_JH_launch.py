from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明所有可配置的启动参数
    declare_video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value='/home/tl/RV/src/marnav_vis/clip-01/2022_06_04_12_05_12_12_07_02_b.mp4',
        description='Path to the video file for camera publisher'
    )
    
    declare_camera_fps_arg = DeclareLaunchArgument(
        'camera_publish_fps',
        default_value='25',
        description='Publishing frequency (Hz) for camera images'
    )
    declare_camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='1280',
        description='Width of the published camera images'
    )
    declare_camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='720',
        description='Height of the published camera images'
    )
    declare_width_height_arg = DeclareLaunchArgument(
        'width_height',
        default_value='[1280, 720]',
        description='Width and height of the published camera images'
    )
    
    declare_gnss_rate_arg = DeclareLaunchArgument(
        'gnss_publish_rate',
        default_value='5.0',
        description='Publishing frequency (Hz) for GNSS data'
    )
    
    declare_ais_folder_arg = DeclareLaunchArgument(
        'ais_csv_folder',
        default_value='/home/tl/RV/src/marnav_vis/clip-01/ais/',
        description='Folder path containing AIS CSV files'
    )

    declare_ais_start_timestamp_arg = DeclareLaunchArgument(
        'ais_start_timestamp',
        default_value='1654315512000',
        description='Start timestamp for AIS data publication'
    )

    declare_gnss_start_timestamp_arg = DeclareLaunchArgument(
        'gnss_start_timestamp',
        default_value='1654315512000',
        description='Start timestamp for GNSS data publication'
    )

    declare_camera_start_timestamp_arg = DeclareLaunchArgument(
        'camera_start_timestamp',
        default_value='1654315512000',
        description='Start timestamp for camera data publication'
    )

    declare_ais_csv_topic_arg = DeclareLaunchArgument(
        'ais_csv_topic',
        default_value='/ais_csv_topic',
        description='Topic name for single AIS data publication'
    )

    declare_ais_batch_pub_topic_arg = DeclareLaunchArgument(
        'ais_batch_pub_topic',
        default_value='/ais_batch_topic_offline',
        description='Topic name for batch AIS data publication'
    )

    declare_gnss_pub_topic_arg = DeclareLaunchArgument(
        'gnss_pub_topic',
        default_value='/gnss_pub_topic',
        description='Topic name for GNSS data publication'
    )

    declare_fus_trajectory_topic_arg = DeclareLaunchArgument(
        'fus_trajectory_topic',
        default_value='/fus_trajectory_topic',
        description='Topic name for fused trajectory data publication'
    )

    # camera_topics 不作为 LaunchArgument，直接在节点中配置
    # declare_camera_topics_arg = DeclareLaunchArgument(
    #     'camera_topics',
    #     default_value=['/camera_image_topic_0', '/camera_image_topic_1', '/camera_image_topic_2'],
    #     description='Topic names for camera data publication'
    # )

    # 相机发布节点
    camera_pub_node = Node(
        package='marnav_vis',  # 替换为实际包名
        executable='camera_pub_temporary_Test_node',  # 替换为setup.py中配置的可执行文件名
        name='camera_publisher_node',
        output='screen',
        parameters=[{
            'video_path': LaunchConfiguration('video_path'),
            'publish_fps': LaunchConfiguration('camera_publish_fps'),
            'width_height': LaunchConfiguration('width_height'),
            'camera_start_timestamp': LaunchConfiguration('camera_start_timestamp'),
            'camera_topics': ['/camera_image_topic_0', '/camera_image_topic_1', '/camera_image_topic_2']  # 直接定义数组
        }]
    )

    # GNSS发布节点
    gnss_pub_node = Node(
        package='marnav_vis',  # 替换为实际包名
        executable='gnss_pub_node',  # 替换为setup.py中配置的可执行文件名
        name='gnss_publisher_node',
        output='screen',
        parameters=[{
            'gnss_start_timestamp': LaunchConfiguration('gnss_start_timestamp'),
            'publish_rate': LaunchConfiguration('gnss_publish_rate'),
            'gnss_pub_topic': LaunchConfiguration('gnss_pub_topic')
        }]
    )

    # AIS CSV发布节点
    ais_csv_pub_node = Node(
        package='marnav_vis',  # 替换为实际包名
        executable='ais_csv_pub_node',  # 替换为setup.py中配置的可执行文件名
        name='ais_csv_publisher_node',
        output='screen',
        parameters=[{
            'ais_csv_folder': LaunchConfiguration('ais_csv_folder'),
            'ais_csv_topic': LaunchConfiguration('ais_csv_topic')
        }]
    )

    # AIS批量发布节点
    ais_sorted_pub_node = Node(
        package='marnav_vis',  # 替换为实际包名
        executable='ais_sorted_pub_node',  # 替换为setup.py中配置的可执行文件名
        name='ais_batch_publisher_node',
        output='screen',
        parameters=[{
            'ais_start_timestamp': LaunchConfiguration('ais_start_timestamp'),
            'ais_csv_topic': LaunchConfiguration('ais_csv_topic'),
            'ais_batch_pub_topic': LaunchConfiguration('ais_batch_pub_topic')
        }]
    )

    # DeepSORVF_ros_v节点
    deep_sorvf_node = Node(
        package='marnav_vis',  # 替换为实际包名
        executable='DeepSORVF_JH',  # 替换为setup.py中配置的可执行文件名
        name='ais_vis_node',    
        output='screen',
        parameters=[{
        #     # 'width_height': LaunchConfiguration('width_height'),
        #     'camera_topics': ['/camera_image_topic_0', '/camera_image_topic_1', '/camera_image_topic_2'],
            'ais_batch_pub_topic': LaunchConfiguration('ais_batch_pub_topic'),
        #     'gnss_pub_topic': LaunchConfiguration('gnss_pub_topic'),
        #     'fus_trajectory_topic': LaunchConfiguration('fus_trajectory_topic'),
        #     'input_fps': 15,
        #     'output_fps': 10,
        }]
    )

    # 组装启动描述
    return LaunchDescription([
        declare_video_path_arg,
        declare_camera_fps_arg,
        # declare_camera_width_arg,
        # declare_camera_height_arg,  
        declare_width_height_arg,
        declare_camera_start_timestamp_arg,
        # declare_camera_topics_arg,  # 已注释，直接在node中配置
        declare_gnss_start_timestamp_arg,
        declare_gnss_rate_arg,
        declare_ais_folder_arg,
        declare_ais_start_timestamp_arg,
        declare_ais_batch_pub_topic_arg,
        declare_ais_csv_topic_arg,
        declare_gnss_pub_topic_arg,
        declare_fus_trajectory_topic_arg,
        
        camera_pub_node,
        gnss_pub_node,
        ais_csv_pub_node,
        ais_sorted_pub_node,
        deep_sorvf_node
    ])

