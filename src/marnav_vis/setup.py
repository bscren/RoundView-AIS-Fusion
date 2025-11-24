from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'marnav_vis'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
     # 明确列出所有需要安装的包（包括同级目录的模块）
    packages=[
        package_name,  # 主包 marnav_vis
        'detection_yolox',
        'detection_yolox.nets',
        'detection_yolox.utils',
        'deep_sort',
        'deep_sort.deep_sort',
        'deep_sort.deep_sort.deep',
        'deep_sort.deep_sort.sort',
        'deep_sort.utils', 
        'utils',
        'utils_backup'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # 添加这一行
        (f'share/{package_name}/config', ['config/camera_0_param_location.txt',
                                         'config/camera_1_param_location.txt',
                                         'config/camera_2_param_location.txt']),# f 的意思是格式化字符串
        (f'share/{package_name}/detection_yolox/model_data', 
                                ['detection_yolox/model_data/ship_classes.txt', 'detection_yolox/model_data/yolo_anchors.txt'])
        # (f'share/{package_name}/weights', 
        #                         ['weights/yolov3_deep_sort.pth']),

    ],
    install_requires=['setuptools', 'easydict==1.11', 'fastdtw'],
    zip_safe=True,
    maintainer='tl',
    maintainer_email='tl@todo.todo',
    description='AIS coordinate visualization package for ROS2',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'DeepSORVF = marnav_vis.DeepSORVF_ros_v1:main',
            'DeepSORVF_ros = marnav_vis.DeepSORVF_ros_v6:main',
            'ais_csv_pub_node = marnav_vis.ais_csv_pub:main',
            'ais_sorted_pub_node = marnav_vis.ais_sorted_pub:main',
            'camera_pub_node = marnav_vis.camera_pub:main',
            'camera_sub_node = marnav_vis.camera_sub:main',
            'camera_pub_temporary_Test_node = marnav_vis.camera_pub_temporary_Test:main',
            'gnss_pub_node = marnav_vis.gnss_pub:main',
        ],
    },
)
