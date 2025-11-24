from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='webcamcalib',  # 节点所在的包名（根据代码中的get_package_share_directory确定）
            executable='webcam_publisher',  # 可执行文件名（通常与脚本名一致，需在setup.py中配置）
            name='webcam_publisher',  # 节点名称
            output='screen',  # 输出到控制台
            parameters=[
                # 设置参数默认值，可在启动时通过命令行覆盖
                {'image_raw_topic': '/image_raw'},
                {'image_calib_topic': '/image_calib'},
                {'JHjpg_raw_topic': '/JHjpg_raw'},
                {'JHjpg_calib_topic': '/JHjpg_calib'},
                {'relative_calib_file_path': 'CamsCalibrationData/DAHUA192.168.0.118/ost.yaml'},
                {'camera_url': 'rtsp://admin:sjtu1234@192.168.0.118:554/cam/realmonitor?channel=1&subtype=0'},
                {'frame_rate': 20},
                {'PubImageorJHjpgorRaworCalib': 'JHjpgCalib'}, # 4选1: 'ImageRaw' 或 'ImageRawCalib' 或 'JHjpgRaw' 或 'JHjpgCalib'
                {'SCALE': 0.5}  # 图像缩放比例，默认不缩放
            ]
        )
    ])