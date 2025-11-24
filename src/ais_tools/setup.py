from setuptools import find_packages, setup
import os
package_name = 'ais_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [
            'launch/ASM_series.launch.py'  # 匹配你的 launch 文件名
        ]),
        (os.path.join('share', package_name, 'launch'), [
            'launch/ASM_log.launch.py'  # 匹配你的 launch 文件名
        ]),
        # 若有 logs 目录，也建议添加（用于存放默认日志文件）
        (os.path.join('share', package_name, 'logs'), [
            'logs/2019-10-15T12.57.27.338020_ais.log'
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tl',
    maintainer_email='tl@todo.todo',
    description='ROS 2 tools for AIS data processing',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'nmea_relay = ais_tools.nmea_relay:main',
            'nmea_replay = ais_tools.nmea_replay:main',
            'ais_parser = ais_tools.ais_parser:main',
            'ais_contact_tracker = ais_tools.ais_contact_tracker:main',
        ],
    },
)
