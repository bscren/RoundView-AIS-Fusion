#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数（允许从命令行传入）
    declare_filename_arg = DeclareLaunchArgument(
        'filename',
        default_value=PathJoinSubstitution([
            FindPackageShare('ais_tools'),
            'logs',
            'ais_2025-08-28T08.25.11.690444+00.00.log'  # 默认日志文件路径
        ]),
        description='NMEA日志文件路径'
    )
    
    declare_rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='日志回放速率'
    )
    
    declare_clock_arg = DeclareLaunchArgument(
        'clock',
        default_value='true',
        description='是否发布时钟话题'
    )

    # 1. 启动 nmea_replay 节点（第一个启动）
    nmea_replay_node = Node(
        package='ais_tools',
        executable='nmea_replay',
        name='nmea_replay',
        parameters=[{
            'filename': LaunchConfiguration('filename'),
            'rate': LaunchConfiguration('rate'),
            'clock': LaunchConfiguration('clock')
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
        declare_filename_arg,
        declare_rate_arg,
        declare_clock_arg,
        nmea_replay_node,
        ais_parser_node,
        ais_contact_tracker_node
    ])
    