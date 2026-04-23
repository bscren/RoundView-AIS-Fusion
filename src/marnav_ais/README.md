# AIS Parser Node V2 - 批量发布版本

## 📝 概述

AIS解析节点V2版本，专注于高效批量处理AIVDM报文。与原版相比，主要改进：

- ✅ **批量发布**：每秒汇总发布上一秒内的所有AIS数据（`AisBatch`格式）
- ✅ **聚焦AIVDM**：仅解析Type 1/2/3位置报告，移除RMC/GGA处理
- ✅ **高鲁棒性**：多片段组合、数据校验、缓存溢出保护
- ✅ **三接口支持**：Serial/TCP/UDP可配置

## 🚀 快速开始

### 1. 编译

```bash
cd ~/RV
colcon build --packages-select marnav_ais
source install/setup.bash
```

### 2. 配置参数

创建参数文件 `config/ais_batch_params.yaml`（或直接在launch中修改）：

```yaml
ais_batch_parser:
  ros__parameters:
    # 通信方式: serial / tcp / udp
    comm_type: "serial"
    
    # 串口配置(当前设备就是使用串口 ttyS3 通信，波特率 38400）
    serial_port: "/dev/ttyS3"
    baud_rate: 38400
    
    # TCP配置（当comm_type=tcp时使用）
    tcp_ip: "127.0.0.1"
    tcp_port: 5000
    
    # UDP配置（当comm_type=udp时使用）
    udp_ip: "0.0.0.0"
    udp_port: 1300
    
    # 日志配置
    log_directory: "/home/tl/RV/ais_logs"
    log_raw_data: false  # 是否记录原始NMEA数据
    
    # 缓存配置
    max_cache_size: 1000  # 最大缓存条目（防溢出）
```

### 3. 启动节点

#### 方式一：直接运行
```bash
ros2 run marnav_ais ais_parser_node_v2
```

#### 方式二：带参数运行
```bash
ros2 run marnav_ais ais_parser_node_v2 \
  --ros-args \
  -p comm_type:=serial \
  -p serial_port:=/dev/ttyS3 \
  -p baud_rate:=38400
```

#### 方式三：使用参数文件
```bash
ros2 run marnav_ais ais_parser_node_v2 \
  --ros-args \
  --params-file config/ais_batch_params.yaml
```

## 📡 话题接口

### 发布话题

| 话题名称 | 消息类型 | 频率 | 说明 |
|---------|---------|------|------|
| `/ais_batch_data` | `marnav_interfaces/msg/AisBatch` | 1Hz | 批量AIS数据（每秒汇总） |

### 消息格式

**AisBatch.msg:**
```
marnav_interfaces/Ais[] ais_list      # 当前批次的AIS数据列表
builtin_interfaces/Time batch_time    # 批次发布时间戳
```

**Ais.msg:**
```
uint64 mmsi                           # 船舶MMSI号
float64 lat                           # 纬度（度）
float64 lon                           # 经度（度）
float64 speed                         # 对地速度（节）
float64 course                        # 对地航向（度，真北）
float64 heading                       # 船首向（度，真北）
uint8 type                            # AIS消息类型（1/2/3）
builtin_interfaces/Time timestamp     # 接收时间戳
```

## 🔍 功能特性

### 1. 多片段组合
自动处理被拆分的AIVDM报文：
```
!AIVDM,2,1,0,A,56:btmP000030000001`PDQE`PuDQDwGK...,0*2D
!AIVDM,2,2,0,A,OP000164h<5500009f00000,0*1A
→ 自动组合后解析
```

### 2. 数据校验
- MMSI非零检查
- 位置数据有效性验证
- 导航数据缺失容错（使用默认值）

### 3. 缓存管理
- 每秒清空一次缓存（批量发布后）
- 溢出保护（移除最旧数据）
- 线程安全（mutex保护）

### 4. 日志系统
可选的原始NMEA数据日志（CSV格式）：
```
UTC时间,ROS时间戳,原始NMEA
2025-01-09T10:15:30,1736416530.123456789,!AIVDM,1,1,,A,16:aUWU00R8cRIRAs>IqS@0@0000,0*3A
```

## 🛠️ 调试与监控

### 查看批量数据
```bash
ros2 topic echo /ais_batch_data
```

### 查看发布频率
```bash
ros2 topic hz /ais_batch_data
```

### 查看批次统计
```bash
ros2 topic echo /ais_batch_data --field ais_list
# 或查看批次大小
ros2 topic echo /ais_batch_data | grep -c "mmsi:"
```

### 日志级别调整
```bash
# DEBUG级别（显示详细解析信息）
ros2 run marnav_ais ais_parser_node_v2 --ros-args --log-level debug

# INFO级别（仅显示批量发布信息）
ros2 run marnav_ais ais_parser_node_v2 --ros-args --log-level info
```

## 📊 性能说明

- **缓存容量**：默认1000条（可配置`max_cache_size`）
- **发布频率**：固定1Hz
- **处理延迟**：< 1秒（批量模式）
- **内存占用**：约 5-20MB（取决于缓存大小）

### 高负载场景
若AIS数据流量极大（>1000条/秒），建议：
1. 增加 `max_cache_size` 参数
2. 监控日志中的"缓存溢出"警告
3. 考虑使用更快的序列化方式

## 🔄 与原版对比

| 特性 | 原版 (v1) | V2版 (批量) |
|------|-----------|-------------|
| RMC解析 | ✅ | ❌ |
| GGA解析 | ✅ | ❌ |
| AIVDM解析 | ✅ | ✅ |
| 发布模式 | 实时（收到即发） | 批量（1Hz） |
| 消息格式 | `Ais` | `AisBatch` |
| 数据缓存 | 无 | ✅ deque |
| 适用场景 | 实时性要求高 | 数据分析/批处理 |

## ⚙️ 常见问题

### Q1: 为什么批量发布间隔为1秒？
**A:** 平衡实时性与数据完整性。若需更快，可修改定时器：
```cpp
// 改为500ms发布一次
publish_timer_ = create_wall_timer(500ms, ...);
```

### Q2: 如何处理"缓存溢出"警告？
**A:** 增加配置参数：
```yaml
max_cache_size: 5000  # 默认1000
```

### Q3: 为什么有些AIS数据未发布？
**A:** 可能原因：
1. 非Type 1/2/3消息（如Type 5静态数据）
2. 数据校验失败（无效位置/MMSI）
3. 多片段组合失败

查看DEBUG日志排查：
```bash
ros2 run marnav_ais ais_parser_node_v2 --ros-args --log-level debug
```

### Q4: 如何同时使用V1和V2？
**A:** 两个版本可并行运行（话题不冲突）：
```bash
# 终端1：原版（实时发布）
ros2 run marnav_ais ais_parser_node

# 终端2：V2版（批量发布）
ros2 run marnav_ais ais_parser_node_v2
```

## 📦 依赖项

- ROS 2 Humble/Foxy
- marnav (AIS解码库)
- libserial-dev (串口通信)
- GeographicLib (地理计算)
- marnav_interfaces (自定义消息)

## 📝 开发说明

### 添加新AIS类型支持

1. 在 `process_position_report()` 中添加类型判断：
```cpp
else if (message->type() == ais::message_id::static_data_report) {
    // 处理Type 5静态数据
}
```

2. 扩展 `Ais.msg` 添加新字段
3. 重新编译 `marnav_interfaces` 和 `marnav_ais`

### 自定义批量策略

修改 `publish_ais_batch()` 实现自定义逻辑：
- 按MMSI分组
- 按距离过滤
- 数据去重

## 📄 许可证

与原 marnav_ais 包保持一致

## 👥 维护者

基于原版重写，优化批量发布场景

