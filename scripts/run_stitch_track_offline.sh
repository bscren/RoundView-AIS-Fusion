#!/bin/bash
# 启动 JH_Stitch_Track_Realtime_launch.py 的 ROS2 launch 脚本

set -e

LAUNCH_FILE="src/marnav_vis/launch/Assemble_track_offline_launch.py"
WORKSPACE_DIR="$(dirname $(dirname $(realpath "$0")))"

cd "$WORKSPACE_DIR"

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "请先 source ROS2 环境 (如 source /opt/ros/<distro>/setup.bash)"
    exit 1
fi

# 检查 colcon 构建结果
if [ ! -d "install" ]; then
    echo "未检测到 install 目录，请先 colcon build"
    exit 1
fi


# 自动设置 RV_WS 为当前工作空间路径
export RV_WS="$WORKSPACE_DIR"
echo "✅ RV_WS 已设置为：$RV_WS"

source install/setup.bash

ros2 launch "$LAUNCH_FILE" "$@"
