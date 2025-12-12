# ROS2节点YAML配置化重构总结

## 重构目标

将ROS2节点从硬编码参数重构为基于YAML配置文件的参数管理方式，提高系统的灵活性、可维护性和可扩展性。

## 修改的文件

### 新增文件

1. **src/marnav_vis/marnav_vis/config_loader.py**
   - 通用YAML配置加载工具类
   - 提供统一的配置文件读取接口
   - 支持多层级配置访问
   - 自动查找ROS包中的配置文件

### 修改的配置文件

2. **src/marnav_vis/config/track_offline_config.yaml**
   - 将`camera_width`和`camera_height`合并为`width_height`数组
   - 修正`noise_range_ns`为纯数值格式
   - 保持其他配置不变

3. **src/marnav_vis/config/track_realtime_config.yaml**
   - 将`camera_width`和`camera_height`合并为`width_height`数组
   - 保持其他配置不变

### 修改的Python节点

4. **src/marnav_vis/marnav_vis/camera_pub_temporary_Test.py**
   - **改动**: 移除硬编码参数，从YAML配置文件读取
   - **主要变化**:
     - 添加`config_file`参数支持
     - 从配置文件读取视频路径、FPS、尺寸、时间戳等参数
     - 从配置的`cameras`列表动态生成相机话题
     - 添加详细的配置信息打印

5. **src/marnav_vis/marnav_vis/gnss_pub.py**
   - **改动**: GNSS参数从配置文件读取
   - **主要变化**:
     - 添加`config_file`参数支持
     - 从配置文件读取发布频率和话题名称
     - 从配置文件读取GNSS位置参数（经纬度、朝向、高度）
     - 添加详细的配置信息打印

6. **src/marnav_vis/marnav_vis/ais_csv_pub.py**
   - **改动**: AIS数据路径和话题从配置文件读取
   - **主要变化**:
     - 添加`config_file`参数支持
     - 从配置文件读取AIS CSV文件夹路径和话题名称
     - 添加详细的配置信息打印

7. **src/marnav_vis/marnav_vis/ais_sorted_pub.py**
   - **改动**: AIS批量发布参数从配置文件读取
   - **主要变化**:
     - 添加`config_file`参数支持
     - 从配置文件读取起始时间戳和话题名称
     - 添加详细的配置信息打印

8. **src/marnav_vis/marnav_vis/DeepSORVF_ros_v7.py**
   - **改动**: 全面重构，移除所有硬编码参数
   - **主要变化**:
     - 添加`config_file`参数支持
     - 从配置文件读取相机配置（尺寸、话题、映射）
     - 从配置文件读取AIS、GNSS、DeepSORVF参数
     - 自动生成相机名称映射（移除硬编码映射）
     - **重要**: 移除`gnss_callback`中的硬编码GNSS参数
     - 改为使用实时GNSS消息更新相机位置参数
     - 在`main`函数中从配置文件读取相机名称列表
     - 添加详细的配置信息打印

### 修改的Launch文件

9. **src/marnav_vis/launch/Assemble_JH_launch.py**
   - **改动**: 添加配置文件参数支持
   - **主要变化**:
     - 添加`declare_config_file_arg`参数声明
     - 默认配置文件路径: `config/track_offline_config.yaml`
     - 所有节点的`parameters`简化为只传递`config_file`
     - 保留原有参数声明（用于可能的覆盖需求）
     - 优化节点启动顺序

### 文档

10. **src/marnav_vis/README_CONFIG.md**
    - 新增配置文件使用说明文档
    - 包含完整的配置文件结构说明
    - 提供多种使用方式示例
    - 包含故障排除指南

11. **REFACTOR_SUMMARY.md** (本文件)
    - 重构总结文档

## 代码改进亮点

### 1. 统一的配置管理

**之前**:
```python
# 硬编码参数
self.video_path = '/home/tl/RV/src/marnav_vis/clip-01/video.mp4'
self.publish_fps = 25
self.width_height = [1280, 720]
```

**现在**:
```python
# 从配置文件读取
config_loader = ConfigLoader(config_file)
camera_config = config_loader.get_camera_config()
self.video_path = camera_config.get('video_path', '')
self.publish_fps = camera_config.get('camera_publish_fps', 25)
self.width_height = camera_config.get('width_height', [1280, 720])
```

### 2. 动态相机配置

**之前**:
```python
# 硬编码相机映射
self.camera_name_mapping = {
    '/camera_image_topic_0': 'camera_0',
    '/camera_image_topic_1': 'camera_1',
    '/camera_image_topic_2': 'camera_2',
}
```

**现在**:
```python
# 从配置文件自动生成映射
self.camera_name_mapping = {}
for cam in camera_config.get('cameras', []):
    topic_name = cam.get('topic_name', '')
    camera_name = cam.get('camera_name', '')
    self.camera_name_mapping[topic_name] = camera_name
```

### 3. 移除硬编码的GNSS参数

**之前** (DeepSORVF_ros_v7.py的gnss_callback):
```python
self.camera_pos_para[idx] = {
    "longitude": 114.32583,  # 硬编码
    "latitude": 30.60139,    # 硬编码
    "horizontal_orientation": 7,  # 硬编码
    "vertical_orientation": -1,   # 硬编码
    "camera_height": 20,          # 硬编码
    'fov_hor': 55,
    'fov_ver': 30.94,
    'fx': 2391.26,
    'fy': 2446.89,
    'u0': 1305.04,
    'v0': 855.214,
}
```

**现在**:
```python
# 使用实时GNSS消息
self.camera_pos_para[idx] = {
    "longitude": msg.longitude,
    "latitude": msg.latitude,
    "horizontal_orientation": horizontal_orientation,
    "vertical_orientation": msg.vertical_orientation,
    "camera_height": msg.camera_height,
    'fov_hor': self.camera_para[idx]['fov_hor'],
    'fov_ver': self.camera_para[idx]['fov_ver'],
    'fx': K[0,0],
    'fy': K[1,1],
    'u0': K[0,2],
    'v0': K[1,2],
}
```

### 4. 详细的启动日志

所有节点启动时会打印清晰的配置信息:
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

## 优势

1. **灵活性提升**
   - 无需修改代码即可调整参数
   - 支持多套配置文件（离线/实时）
   - 便于不同场景的快速切换

2. **可维护性提升**
   - 配置集中管理
   - 代码更清晰，职责分离
   - 减少了魔法数字

3. **可扩展性提升**
   - 支持动态增减相机数量
   - 易于添加新的配置项
   - 配置加载器可复用

4. **错误处理改进**
   - 配置文件验证
   - 清晰的错误提示
   - 默认值机制

5. **开发效率提升**
   - 无需重新编译
   - 快速测试不同配置
   - 便于协作开发

## 使用示例

### 基本使用
```bash
# 使用默认配置
ros2 launch marnav_vis Assemble_JH_launch.py

# 使用自定义配置
ros2 launch marnav_vis Assemble_JH_launch.py config_file:=/path/to/custom_config.yaml
```

### 切换离线/实时模式
```bash
# 离线数据回放
ros2 launch marnav_vis Assemble_JH_launch.py \
  config_file:=/home/tl/RV/src/marnav_vis/config/track_offline_config.yaml

# 实时数据采集
ros2 launch marnav_vis Assemble_JH_launch.py \
  config_file:=/home/tl/RV/src/marnav_vis/config/track_realtime_config.yaml
```

## 向后兼容性

- Launch文件保留了所有原有参数声明
- 节点可以同时支持旧的参数传递方式和新的配置文件方式
- 配置文件参数优先级高于默认参数

## 测试建议

1. **基本功能测试**
   ```bash
   # 测试默认配置
   ros2 launch marnav_vis Assemble_JH_launch.py
   ```

2. **配置文件切换测试**
   ```bash
   # 测试离线配置
   ros2 launch marnav_vis Assemble_JH_launch.py \
     config_file:=/home/tl/RV/src/marnav_vis/config/track_offline_config.yaml
   
   # 测试实时配置
   ros2 launch marnav_vis Assemble_JH_launch.py \
     config_file:=/home/tl/RV/src/marnav_vis/config/track_realtime_config.yaml
   ```

3. **单节点测试**
   ```bash
   # 测试相机节点
   ros2 run marnav_vis camera_pub_temporary_Test_node \
     --ros-args -p config_file:=/path/to/config.yaml
   ```

4. **配置验证**
   - 检查启动日志中的配置信息是否正确
   - 验证话题名称是否与配置文件一致
   - 测试修改配置文件后系统行为是否改变

## 后续改进建议

1. **配置文件验证**
   - 添加JSON Schema验证
   - 提供配置文件检查工具

2. **动态重载**
   - 支持运行时重载配置
   - 添加配置文件监听

3. **参数覆盖**
   - 支持通过环境变量覆盖配置
   - 支持命令行参数优先级

4. **配置模板**
   - 提供更多场景的配置模板
   - 添加配置生成工具

## 贡献者

- 重构实施: Claude (AI助手)
- 需求提出: 用户
- 日期: 2025年12月11日

## 相关文档

- [配置文件使用说明](src/marnav_vis/README_CONFIG.md)
- [YAML配置文件示例](src/marnav_vis/config/)
- [ConfigLoader API文档](src/marnav_vis/marnav_vis/config_loader.py)

