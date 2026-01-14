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
        'utils_backup',
        'utils_JH'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
        # 把config目录下的配置文件安装到包的share目录下
        (f'share/{package_name}/config', [
            'config/camera_0_param_location.txt',
            'config/camera_1_param_location.txt',
            'config/camera_2_param_location.txt',
            'config/track_offline_config.yaml',  # 添加YAML配置文件
            'config/track_realtime_config.yaml'  # 添加YAML配置文件
        ]),
        # 把deep_sort/configs目录:track function 安装到包的share目录下
        (f'share/{package_name}/deep_sort/configs', glob('deep_sort/configs/*.yaml')),
        # 把deep_sort权重文件安装到包的share目录下，运行时按相对路径查找
        (f'share/{package_name}/deep_sort/deep_sort/deep/checkpoint', glob('deep_sort/deep_sort/deep/checkpoint/*')),
        # 把detection_yolox/model_data目录:detection function 安装到包的share目录下
        (f'share/{package_name}/detection_yolox/model_data', glob('detection_yolox/model_data/*'))
    ],
    install_requires=['setuptools', 'easydict==1.11', 'fastdtw', 'pyyaml'],
    zip_safe=True,
    maintainer='tl',
    maintainer_email='tl@todo.todo',
    description='AIS coordinate visualization package for ROS2',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'DeepSORVF = marnav_vis.DeepSORVF_ros_v1:main',
            'DeepSORVF_ros = marnav_vis.DeepSORVF_ros_v6:main',
            'DeepSORVF_JH = marnav_vis.DeepSORVF_ros_v7:main',
            'ais_csv_pub_node = marnav_vis.ais_csv_pub:main',
            'ais_sorted_pub_node = marnav_vis.ais_sorted_pub:main',
            'camera_pub_node = marnav_vis.camera_pub:main',
            'gnss_pub_node = marnav_vis.gnss_pub:main',
        ],
    },
)
