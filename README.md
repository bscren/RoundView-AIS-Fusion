# 基本环境：
ROS2 Humble
Python 3.10
CUDA 11.8
pytorch 2.7.0

# 部署需要：
Docker,
Nvidia-Docker

# 基于ros2_humble_cuda118:v1镜像创建并启动容器
启动命令:
```bash
docker run -d -it \
  --gpus all \  # 无GPU则删除此行
  --name ros2_marnav \
  -v /home/tl/RV:/workspace \  # 核心：宿主机RV目录挂载到容器/workspace
  --privileged \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2_humble_cuda118:v1
```
重点说明
```bash
  --gpus all \  # 启用GPU
  --name ros2_marnav \  # 给容器起固定名称，方便后续操作
  -v /home/tl/RV:/workspace \  # 关键：挂载宿主机RV目录到容器/workspace
  --privileged \  # 提升权限，避免设备/路径访问权限问题
  --network host \  # 可选：使用主机网络，ROS2节点通信更方便
  -e DISPLAY=$DISPLAY \  # 绑定宿主机显示
  -v /tmp/.X11-unix:/tmp/.X11-unix \  # 挂载X11套接字
  ros2_humble_cuda118:v1  # 你的镜像名（必须和docker images里的一致）
```


#  完整 Docker 启动流程

步骤 1：宿主机 Docker 权限配置（一次性操作）, 解决普通用户执行 Docker 命令报 permission denied 问题：

```bash
# 1. 将当前用户加入docker组
sudo groupadd docker  # 已存在则忽略报错
sudo usermod -aG docker tl
newgrp docker  # 临时生效，注销重登录永久生效
# 2. 验证权限（无报错则成功）
docker ps -a
```

步骤 2：宿主机开放 X11 显示权限（可视化必备）
解决容器内 Qt/OpenCV 可视化报错 qt.qpa.xcb: could not connect to display：
```bash
xhost +local:root  # 允许容器访问宿主机显示器
```

步骤 3：创建并启动 Docker 容器（宿主机执行）, 注意：需在宿主机任意目录执行（无需限定 RV 文件夹），核心是挂载绝对路径：
```bash
运行
# 停止并删除旧容器（若存在）
docker stop ros2_marnav || true
docker rm ros2_marnav || true

# 启动新容器（绑定GPU、显示、网络、工作目录）
docker run -d -it \
  --gpus all \
  --name ros2_marnav \
  -v /home/tl/RV:/workspace \
  --privileged \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2_humble_cuda118:v1
```

步骤 4：进入容器并运行 ROS2 程序
```bash
# 1. 进入运行中的容器（宿主机执行）
docker exec -it ros2_marnav bash
# 2. 容器内加载ROS2环境（必做，路径别写错）
source /opt/ros/humble/setup.bash  # 正确路径：含/ros/层级
cd /workspace
source install/setup.bash
# 3. 启动工程程序
ros2 launch marnav_vis Assemble_track_offline_launch.py
```

步骤 5：容器停止 / 删除（宿主机执行）
```bash
# 仅停止容器（保留实例，后续可重启）
docker stop ros2_marnav
# 重启容器
docker start ros2_marnav
# 彻底删除容器（需先停止）
docker stop ros2_marnav && docker rm ros2_marnav
```