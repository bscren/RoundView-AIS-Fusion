from setuptools import setup
import os
from glob import glob

package_name = 'yolo_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', glob('msg/*.msg')),
        # 暂时注释：消息生成后再启用
        # (os.path.join('lib', package_name, package_name, 'msg'), 
        #  glob('build/' + package_name + '/rosidl_generator_py/' + package_name + '/msg/*.py')),
    ],
    install_requires=['setuptools', 'ultralytics'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'yolo_inference = yolo_detect.yolo_inference_node_2:main',
            'detection_saver = yolo_detect.detection_saver:main',
            'yolo_video_inference = yolo_detect.yolo_inference_node_3:main',
        ],
    },
)