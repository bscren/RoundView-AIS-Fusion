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
        default_value='15',
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
    
    declare_gnss_rate_arg = DeclareLaunchArgument(
        'gnss_publish_rate',
        default_value='10.0',
        description='Publishing frequency (Hz) for GNSS data'
    )
    
    declare_ais_folder_arg = DeclareLaunchArgument(
        'ais_csv_folder',
        default_value='/home/tl/RV/src/marnav_vis/clip-01/ais/',
        description='Folder path containing AIS CSV files'
    )

    # 相机发布节点
    camera_pub_node = Node(
        package='marnav_vis',  # 替换为实际包名
        executable='camera_pub_temporary_Test_node',  # 替换为setup.py中配置的可执行文件名
        name='camera_publisher_node',
        output='screen',
        parameters=[{
            'video_path': LaunchConfiguration('video_path'),
            'publish_fps': LaunchConfiguration('camera_publish_fps'),
            'width': LaunchConfiguration('camera_width'),
            'height': LaunchConfiguration('camera_height')
        }]
    )

    # GNSS发布节点
    gnss_pub_node = Node(
        package='marnav_vis',  # 替换为实际包名
        executable='gnss_pub_node',  # 替换为setup.py中配置的可执行文件名
        name='gnss_publisher_node',
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('gnss_publish_rate')
        }]
    )

    # AIS CSV发布节点
    ais_csv_pub_node = Node(
        package='marnav_vis',  # 替换为实际包名
        executable='ais_csv_pub_node',  # 替换为setup.py中配置的可执行文件名
        name='ais_csv_publisher_node',
        output='screen',
        parameters=[{
            'ais_csv_folder': LaunchConfiguration('ais_csv_folder')
        }]
    )

    # AIS批量发布节点
    ais_sorted_pub_node = Node(
        package='marnav_vis',  # 替换为实际包名
        executable='ais_sorted_pub_node',  # 替换为setup.py中配置的可执行文件名
        name='ais_batch_publisher_node',
        output='screen'
        # 该节点无额外参数，无需配置parameters
    )

    # DeepSORVF_ros_v节点
    deep_sorvf_node = Node(
        package='marnav_vis',  # 替换为实际包名
        executable='DeepSORVF_JH',  # 替换为setup.py中配置的可执行文件名
        name='ais_vis_node',
        output='screen'
        # 该节点无额外参数，无需配置parameters
    )

    # 组装启动描述
    return LaunchDescription([
        declare_video_path_arg,
        declare_camera_fps_arg,
        declare_camera_width_arg,
        declare_camera_height_arg,  
        declare_gnss_rate_arg,
        declare_ais_folder_arg,
        camera_pub_node,
        gnss_pub_node,
        ais_csv_pub_node,
        ais_sorted_pub_node,
        deep_sorvf_node
    ])

