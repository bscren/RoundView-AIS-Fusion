# YAML配置文件使用说明

## 概述

本项目已重构为基于YAML配置文件的参数管理方式，减少了硬编码参数，提高了系统的灵活性和可维护性。

## 配置文件位置

配置文件位于：`src/marnav_vis/config/`

- `track_offline_config.yaml` - 离线数据回放配置
- `track_realtime_config.yaml` - 实时数据采集配置

## 配置文件结构

### 1. 相机配置 (camera)

```yaml
camera:
  video_path: "/path/to/video.mp4"           # 视频文件路径（仅离线模式）
  camera_start_timestamp: 1654315512000      # 相机起始时间戳（毫秒）
  camera_publish_fps: 25                      # 发布帧率
  width_height: [1280, 720]                  # 图像尺寸 [宽, 高]
  noise_range_ns: 10000000                   # 时间戳噪声范围（纳秒）
  
  # 相机列表（支持动态增减）
  cameras:
    - camera_name: "camera_0"                # 相机名称
      topic_name: "/camera_image_topic_0"   # ROS话题名
      camera_index: 0                        # 相机索引
    - camera_name: "camera_1"
      topic_name: "/camera_image_topic_1"
      camera_index: 1
    - camera_name: "camera_2"
      topic_name: "/camera_image_topic_2"
      camera_index: 2
```

### 2. GNSS配置 (gnss)

```yaml
gnss:
  gnss_publish_rate: 5.0                     # 发布频率（Hz）
  gnss_pub_topic: "/gnss_pub_topic"         # 发布话题名
  
  # GNSS位置参数（仅离线模式）
  camera_gnss_para:
    lon: 114.32583                           # 经度
    lat: 30.60139                            # 纬度
    horizontal_orientation: 352.0            # 水平朝向（度）
    vertical_orientation: -4.0               # 垂直朝向（度）
    camera_height: 20.0                      # 相机高度（米）
```

### 3. AIS配置 (ais)

```yaml
ais:
  ais_csv_folder: "/path/to/ais/csv/"       # AIS CSV文件夹路径（仅离线模式）
  ais_start_timestamp: 1654315512000        # AIS起始时间戳（毫秒）
  ais_csv_topic: "/ais_csv_topic"           # CSV话题名
  ais_batch_pub_topic: "/ais_batch_topic_offline"  # 批量发布话题名
```

### 4. DeepSORVF跟踪配置 (DeepSORVF)

```yaml
DeepSORVF:
  fus_trajectory_topic: "/fus_trajectory_topic"     # 融合轨迹话题
  get_camera_params_service: "/get_camera_params_service"  # 相机参数服务
  input_fps: 20                              # 输入帧率
  output_fps: 10                             # 输出帧率
  anti: 1                                    # 抗遮挡处理（0或1）
  anti_rate: 0                               # 抗遮挡阈值
  sync_queue_size: 10                        # 同步队列大小
  sync_slop: 0.1                            # 同步时间误差（秒）
  skip_interval: 1000                       # 处理间隔（毫秒）
```

## 使用方法

### 方式1: 使用默认配置文件

直接启动launch文件，系统会自动使用默认配置：

```bash
ros2 launch marnav_vis Assemble_JH_launch.py
```

### 方式2: 指定配置文件

通过launch参数指定自定义配置文件：

```bash
ros2 launch marnav_vis Assemble_JH_launch.py config_file:=/path/to/your/config.yaml
```

### 方式3: 单独启动节点

单独启动某个节点时指定配置文件：

```bash
# 相机发布节点
ros2 run marnav_vis camera_pub_temporary_Test_node --ros-args -p config_file:=/path/to/config.yaml

# GNSS发布节点
ros2 run marnav_vis gnss_pub_node --ros-args -p config_file:=/path/to/config.yaml

# AIS CSV发布节点
ros2 run marnav_vis ais_csv_pub_node --ros-args -p config_file:=/path/to/config.yaml

# AIS批量发布节点
ros2 run marnav_vis ais_sorted_pub_node --ros-args -p config_file:=/path/to/config.yaml

# DeepSORVF跟踪节点
ros2 run marnav_vis DeepSORVF_JH --ros-args -p config_file:=/path/to/config.yaml
```

## 配置验证

系统启动后会在终端打印详细的配置信息，例如：

```
============================================================
📹 相机发布节点配置
============================================================
配置文件: /home/tl/RV/src/marnav_vis/config/track_offline_config.yaml
视频路径: /home/tl/RV/src/marnav_vis/clip-01/2022_06_04_12_05_12_12_07_02_b.mp4
发布频率: 25 Hz
图像尺寸: 1280x720
起始时间戳: 1654315512000 ms
时间戳噪声: ±10.0 ms
相机话题数量: 3
  - /camera_image_topic_0
  - /camera_image_topic_1
  - /camera_image_topic_2
============================================================
```

请检查这些信息确保配置正确加载。

## 节点改动总结

以下节点已重构为基于YAML配置：

1. **camera_pub_temporary_Test.py** - 相机发布节点
   - 不再使用硬编码参数
   - 支持动态配置相机数量和话题名称

2. **gnss_pub.py** - GNSS发布节点
   - GNSS位置参数从配置文件读取
   - 支持配置发布频率和话题名称

3. **ais_csv_pub.py** - AIS CSV发布节点
   - CSV文件夹路径从配置文件读取
   - 支持配置话题名称

4. **ais_sorted_pub.py** - AIS批量发布节点
   - 起始时间戳和话题名称从配置文件读取

5. **DeepSORVF_ros_v7.py** - 船只跟踪节点
   - 所有参数从配置文件读取
   - 相机映射自动从配置生成
   - 移除了硬编码的GNSS参数

## 配置文件工具

项目提供了`config_loader.py`工具类用于加载和解析YAML配置：

```python
from config_loader import ConfigLoader

# 加载配置
config_loader = ConfigLoader('/path/to/config.yaml')

# 读取配置
camera_config = config_loader.get_camera_config()
gnss_config = config_loader.get_gnss_config()
ais_config = config_loader.get_ais_config()
deepsorvf_config = config_loader.get_deepsorvf_config()

# 访问特定值
video_path = config_loader.get('camera', 'video_path')
```

## 注意事项

1. **配置文件路径**: 确保配置文件路径正确，系统会在启动时验证文件是否存在
2. **相机数量**: 当前系统支持动态配置相机数量，但DeepSORVF跟踪节点针对3相机系统优化
3. **话题名称**: 相机话题名称必须在配置文件的`cameras`列表中定义
4. **时间戳单位**: 注意时间戳单位为毫秒，噪声单位为纳秒
5. **向后兼容**: Launch文件保留了所有原有参数声明，但节点优先使用配置文件中的值

## 故障排除

### 问题1: 找不到配置文件

**错误**: `FileNotFoundError: 配置文件不存在`

**解决**: 
- 检查配置文件路径是否正确
- 使用绝对路径指定配置文件
- 确保配置文件具有读取权限

### 问题2: 配置参数缺失

**错误**: `KeyError` 或参数为空

**解决**:
- 检查YAML文件格式是否正确（注意缩进）
- 参考`track_offline_config.yaml`完善配置文件
- 确保所有必需的字段都已定义

### 问题3: 相机映射错误

**错误**: `ValueError: 未定义相机话题`

**解决**:
- 确保`cameras`列表至少定义了一个相机
- 检查`camera_name`、`topic_name`、`camera_index`字段是否完整

## 更新日志

- **2025-12-11**: 初始版本，完成全部节点的YAML配置化重构
  - 创建统一的`ConfigLoader`工具类
  - 更新所有Python节点以支持YAML配置
  - 更新Launch文件以传递配置文件参数
  - 移除硬编码参数，提高系统灵活性

