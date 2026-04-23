from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def _load_yaml_config(config_path):
    with open(config_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def _normalize_rosbag_path(raw_path):
    rosbag_path = os.path.expanduser(raw_path)
    rosbag_path = os.path.abspath(rosbag_path)

    if not os.path.exists(rosbag_path):
        raise RuntimeError(f"指定的 rosbag_path 不存在: {rosbag_path}")

    if os.path.isdir(rosbag_path):
        metadata_path = os.path.join(rosbag_path, "metadata.yaml")
        if not os.path.exists(metadata_path):
            raise RuntimeError(
                f"指定目录不是有效的 rosbag2 目录，缺少 metadata.yaml: {rosbag_path}"
            )
        return rosbag_path

    base_name = os.path.basename(rosbag_path)
    if base_name == "metadata.yaml" or rosbag_path.endswith(".db3"):
        bag_dir = os.path.dirname(rosbag_path)
        metadata_path = os.path.join(bag_dir, "metadata.yaml")
        if not os.path.exists(metadata_path):
            raise RuntimeError(
                f"从文件路径推导 rosbag2 目录失败，缺少 metadata.yaml: {bag_dir}"
            )
        return bag_dir

    raise RuntimeError(
        "rosbag_path 必须是 rosbag2 目录、metadata.yaml 文件路径或 .db3 文件路径："
        f"{rosbag_path}"
    )


def _extract_camera_topics(config_data, top_key, nested_key=None):
    if nested_key is None:
        camera_items = config_data.get(top_key, {}).get("camera_parameters", [])
    else:
        camera_items = (
            config_data.get(top_key, {})
            .get(nested_key, {})
            .get("cameras", [])
        )
    return [camera["topic_name"] for camera in camera_items]


def _create_rosbag_player(context):
    rosbag_stitch_config_file = LaunchConfiguration(
        "rosbag_stitch_config_file"
    ).perform(context)
    track_rosbag_config_file = LaunchConfiguration(
        "track_rosbag_config_file"
    ).perform(context)
    rosbag_path_override = LaunchConfiguration("rosbag_path").perform(context)
    bag_start_delay = float(LaunchConfiguration("bag_start_delay").perform(context))

    rosbag_stitch_config = _load_yaml_config(rosbag_stitch_config_file)
    track_rosbag_config = _load_yaml_config(track_rosbag_config_file)

    rosbag_parameters = rosbag_stitch_config.get("Rosbag_parameters", {})
    rosbag_path = rosbag_path_override or rosbag_parameters.get("rosbag_path", "")
    publish_topics = rosbag_parameters.get("publish_topics", [])

    if not rosbag_path:
        raise RuntimeError(
            "JH_Stitch_Track_RosBag_launch.py 未获取到 rosbag_path，"
            "请在 JH_stitch_rosbag_config.yaml 中配置 Rosbag_parameters.rosbag_path，"
            "或通过 launch 参数 rosbag_path:=... 覆盖。"
        )
    rosbag_path = _normalize_rosbag_path(rosbag_path)

    if not publish_topics:
        raise RuntimeError(
            "JH_Stitch_Track_RosBag_launch.py 未获取到 Rosbag_parameters.publish_topics。"
        )

    stitch_camera_topics = _extract_camera_topics(
        rosbag_stitch_config, "parameters", "Main_parameters"
    )
    track_camera_topics = _extract_camera_topics(track_rosbag_config, "camera")
    if stitch_camera_topics != track_camera_topics:
        raise RuntimeError(
            "JH_stitch_rosbag_config.yaml 与 track_rosbag_config.yaml 的相机 topic 不一致，"
            "当前 RosBag launch 需要两边的图像订阅 topic 完全对齐。"
        )

    stitch_gnss_topic = (
        rosbag_stitch_config.get("parameters", {})
        .get("Main_parameters", {})
        .get("gnss_topic", "/gnss_topic")
    )
    track_gnss_topic = (
        track_rosbag_config.get("gnss", {}).get("gnss_pub_topic", "/gnss_topic")
    )
    if stitch_gnss_topic != track_gnss_topic:
        raise RuntimeError(
            "JH_stitch_rosbag_config.yaml 与 track_rosbag_config.yaml 的 GNSS topic 不一致，"
            "当前 RosBag launch 需要两边的 GNSS topic 完全对齐。"
        )

    track_ais_topic = (
        track_rosbag_config.get("ais", {}).get("ais_batch_pub_topic", "")
    )
    rosbag_ais_topic = next(
        (topic for topic in publish_topics if "ais_batch_topic" in topic),
        "",
    )

    rosbag_cmd = ["ros2", "bag", "play", rosbag_path, "--topics", *publish_topics]
    if rosbag_ais_topic and track_ais_topic and rosbag_ais_topic != track_ais_topic:
        rosbag_cmd.extend(["--remap", f"{rosbag_ais_topic}:={track_ais_topic}"])

    rosbag_play_process = ExecuteProcess(
        cmd=rosbag_cmd,
        output="screen",
    )

    return [
        TimerAction(
            period=bag_start_delay,
            actions=[rosbag_play_process],
        )
    ]


def generate_launch_description():
    pkg_share_marnav_vis = get_package_share_directory("marnav_vis")
    pkg_share_image_stitching = get_package_share_directory("image_stitching_pkg")

    declare_track_rosbag_config_file_arg = DeclareLaunchArgument(
        "track_rosbag_config_file",
        default_value=os.path.join(
            pkg_share_marnav_vis, "config", "track_rosbag_config.yaml"
        ),
        description=(
            "Path to the rosbag tracking configuration file used by DeepSORVF_JH. "
            "RosBag 回放话题需要与该配置中的相机 / AIS / GNSS 订阅项兼容。"
        ),
    )

    declare_rosbag_stitch_config_file_arg = DeclareLaunchArgument(
        "rosbag_stitch_config_file",
        default_value=os.path.join(
            pkg_share_image_stitching, "config", "JH_stitch_rosbag_config.yaml"
        ),
        description=(
            "Path to the rosbag stitch configuration file. "
            "该文件同时提供 JH_ROS_stitch 参数和 Rosbag_parameters。"
        ),
    )

    declare_rosbag_path_arg = DeclareLaunchArgument(
        "rosbag_path",
        default_value="",
        description=(
            "Override rosbag path from Rosbag_parameters.rosbag_path. "
            "为空时使用 rosbag_stitch_config_file 中的配置。"
        ),
    )

    declare_bag_start_delay_arg = DeclareLaunchArgument(
        "bag_start_delay",
        default_value="3.0",
        description=(
            "Delay in seconds before starting ros2 bag play, "
            "用于确保拼接与融合节点先完成订阅。"
        ),
    )

    stitch_node = Node(
        package="image_stitching_pkg",
        executable="JH_ROS_stitch",
        name="JH_ROS_stitch",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("rosbag_stitch_config_file")}],
    )

    deep_sorvf_node = Node(
        package="marnav_vis",
        executable="DeepSORVF_JH",
        name="ais_vis_node",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("track_rosbag_config_file")}],
    )

    rosbag_player = OpaqueFunction(function=_create_rosbag_player)

    return LaunchDescription(
        [
            declare_track_rosbag_config_file_arg,
            declare_rosbag_stitch_config_file_arg,
            declare_rosbag_path_arg,
            declare_bag_start_delay_arg,
            stitch_node,
            deep_sorvf_node,
            rosbag_player,
        ]
    )
