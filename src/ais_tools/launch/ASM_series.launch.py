#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数（允许从命令行传入）


            # 声明ROS 2参数（带默认值）
        # self.declare_parameter('input_type', 'serial')
        # self.declare_parameter('input_address', '/dev/ttyUSB0')
        # self.declare_parameter('input_speed', 38400)
        # self.declare_parameter('input_port', 'dev/')
        # self.declare_parameter('output', 0)
        # self.declare_parameter('output_address', '<broadcast>')
        # self.declare_parameter('frame_id', 'nmea')
        # self.declare_parameter('log_directory', '')


    declare_input_type_arg = DeclareLaunchArgument(
        'input_type',
        default_value='serial',
        description='NMEA输入类型 (serial or tcp or udp)'
    )

    declare_input_address_arg = DeclareLaunchArgument(
        'input_address',
        default_value='/dev/ttyUSB0',
        description='NMEA输入设备地址'
    )
    
    declare_input_speed_arg = DeclareLaunchArgument(
        'input_speed',
        default_value='38400',
        description='NMEA输入设备波特率'
    )
    
    declare_log_directory_arg = DeclareLaunchArgument(
        'log_directory',
        default_value='ais_logs',
        description='日志文件保存目录 (空则不保存)'
    )

    # 1. 启动 nmea_relay 节点（第一个启动）
    nmea_relay_node = Node(
        package='ais_tools',
        executable='nmea_relay',
        name='nmea_relay',
        parameters=[{
            'input_type': LaunchConfiguration('input_type'),
            'input_address': LaunchConfiguration('input_address'),
            'input_speed': LaunchConfiguration('input_speed'),
            'log_directory': LaunchConfiguration('log_directory')
        }],
        output='screen'
    )

    # 2. 延迟1秒启动 ais_parser 节点（第二个启动）
    ais_parser_node = TimerAction(
        period=1.0,  # 延迟1秒，确保nmea_replay先启动
        actions=[
            Node(
                package='ais_tools',
                executable='ais_parser',
                name='ais_parser',
                output='screen'
            )
        ]
    )

    # 3. 延迟2秒启动 ais_contact_tracker 节点（第三个启动）
    ais_contact_tracker_node = TimerAction(
        period=2.0,  # 延迟2秒，确保ais_parser先启动
        actions=[
            Node(
                package='ais_tools',
                executable='ais_contact_tracker',
                name='ais_contact_tracker',
                output='screen'
            )
        ]
    )

    # 组装launch描述
    return LaunchDescription([
        declare_input_type_arg,
        declare_input_address_arg,
        declare_input_speed_arg,
        declare_log_directory_arg,
        nmea_relay_node,
        ais_parser_node,
        ais_contact_tracker_node
    ])
    