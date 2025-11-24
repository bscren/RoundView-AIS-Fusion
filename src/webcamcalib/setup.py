from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'webcamcalib'

def get_calibration_files():
    """递归获取CamsCalibrationData下的所有文件，保留目录结构"""
    src_root = 'CamsCalibrationData'
    install_files = []
    
    # 遍历目录下所有文件和子目录
    for root, _, files in os.walk(src_root):
        if files:  # 只处理包含文件的目录
            # 源文件列表（相对当前目录的路径）
            src_files = [os.path.join(root, f) for f in files]
            # 目标安装路径（share/包名/原始目录结构）
            dest_path = os.path.join('share', package_name, root)
            install_files.append((dest_path, src_files))
    
    return install_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), [
            'launch/webcam_publisher_launch.py'
        ])
    ]+ get_calibration_files(),  # 追加所有标定文件（含完整目录结构）
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tl',
    maintainer_email='tl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_publisher = webcamcalib.webcam_publisher:main',
        ],
    },
)
