# AIS Parser V2 调试指南

## 一、前置准备

### 1.1 编译带调试符号的版本

```bash
cd /home/tl/RV
colcon build --packages-select marnav_ais --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash
```

**说明**：
- `Debug` 模式会包含调试符号（-g）并关闭优化（-O0）
- 编译后的可执行文件位于：`install/marnav_ais/lib/marnav_ais/ais_parser_node_v2`

### 1.2 安装 VSCode C++ 扩展

确保已安装以下扩展：
- **C/C++ Extension Pack** (Microsoft)
- **ROS** (Microsoft) - 可选

---

## 二、VSCode 调试配置

### 2.1 配置文件说明

已为你配置好以下文件：

#### `.vscode/launch.json` - 调试启动配置
包含两个 C++ 调试配置：
1. **ROS2 C++: Debug AIS Parser V2** - UDP 模式调试
2. **ROS2 C++: Debug AIS Parser V2 (Serial)** - 串口模式调试

#### `.vscode/tasks.json` - 编译任务
- `colcon build (Debug)` - 编译调试版本（默认）
- `colcon build (Release)` - 编译发布版本
- `colcon build all` - 编译所有包

---

## 三、调试步骤

### 3.1 使用 VSCode 调试（推荐）

#### 步骤 1：设置断点
在 `ais_parser_node_v2.cpp` 中点击行号左侧设置断点，常用位置：
- **第 345 行**：`parse_aivdm` 函数入口（解析 AIVDM 数据）
- **第 412 行**：`process_ais_message` 函数入口（处理 AIS 消息）
- **第 488 行**：`publish_batch_callback` 函数（批量发布回调）
- **第 610 行**：`comm_loop` 函数（通信循环）

#### 步骤 2：启动调试
1. 按 `F5` 或点击侧边栏的"运行和调试"图标
2. 选择 **"ROS2 C++: Debug AIS Parser V2"** 配置
3. 点击绿色三角形启动调试

#### 步骤 3：调试操作
- **F5**：继续执行
- **F10**：单步跳过（Step Over）
- **F11**：单步进入（Step Into）
- **Shift+F11**：单步跳出（Step Out）
- **Ctrl+Shift+F5**：重启调试
- **Shift+F5**：停止调试

#### 步骤 4：查看变量
- **变量窗口**：查看当前作用域内的所有变量
- **监视窗口**：添加自定义表达式（如 `message->type()`）
- **调用堆栈**：查看函数调用链

---

### 3.2 修改调试参数

编辑 `.vscode/launch.json` 中的 `args` 字段：

```json
"args": [
    "--ros-args",
    "-p", "comm_type:=udp",        // 修改通信类型
    "-p", "udp_port:=1800",        // 修改端口
    "-p", "log_raw_data:=true",    // 启用原始数据日志
    "-p", "log_directory:=/tmp/ais_logs"  // 修改日志目录
]
```

---

### 3.3 使用命令行调试（高级）

#### 方法 1：使用 GDB

```bash
cd /home/tl/RV
source install/setup.bash

# 启动 gdb
gdb install/marnav_ais/lib/marnav_ais/ais_parser_node_v2

# 在 gdb 中设置参数
(gdb) set args --ros-args -p comm_type:=udp -p udp_port:=1800

# 设置断点（行号或函数名）
(gdb) break ais_parser_node_v2.cpp:412
(gdb) break AISBatchParserNode::parse_aivdm

# 运行程序
(gdb) run

# 调试命令
(gdb) next      # 下一行（不进入函数）
(gdb) step      # 单步进入
(gdb) continue  # 继续执行
(gdb) print variable_name  # 打印变量值
(gdb) backtrace  # 查看调用堆栈
(gdb) quit      # 退出
```

#### 方法 2：远程调试（可选）

```bash
# 终端 1：启动 gdbserver
gdbserver :1234 install/marnav_ais/lib/marnav_ais/ais_parser_node_v2

# 终端 2：连接 gdb
gdb install/marnav_ais/lib/marnav_ais/ais_parser_node_v2
(gdb) target remote localhost:1234
```

---

## 四、常见调试场景

### 4.1 调试多片段 AIVDM 组合

**断点位置**：
- 第 366-401 行：片段收集逻辑
- 第 391 行：`collect_payload` 调用

**检查点**：
```cpp
// 查看片段数量
vdm_fragments_.size()

// 查看片段编号
fragment, n_fragments

// 查看 payload 内容
payload
```

### 4.2 调试批量发布逻辑

**断点位置**：
- 第 488 行：定时器回调入口
- 第 504 行：构建 AisBatch 消息

**检查点**：
```cpp
// 查看缓存的 AIS 数据数量
current_batch_.size()

// 查看批次时间戳
batch_msg.batch_time.sec, batch_msg.batch_time.nanosec
```

### 4.3 调试通信问题

**断点位置**：
- 第 610 行：通信循环入口
- 第 621 行：读取数据后
- 第 627 行：解析 AIVDM 前

**检查点**：
```cpp
// 查看接收到的原始数据
data

// 检查连接状态
comm_interface_->is_connected()
```

---

## 五、调试技巧

### 5.1 条件断点

在断点上右键 → 编辑断点 → 添加条件：
```cpp
// 仅当 MMSI 为特定值时暂停
mmsi == 413820318

// 仅当片段编号为 1 时暂停
fragment == 1

// 仅当批次数据大于 5 条时暂停
current_batch_.size() > 5
```

### 5.2 日志断点

在断点上右键 → 编辑断点 → 勾选"日志消息"：
```
收到 VDM 片段: {fragment}/{n_fragments}
```

### 5.3 监视表达式

在"监视"窗口添加：
```cpp
vdm_fragments_.size()           // VDM 片段数量
current_batch_.size()           // 当前批次数据量
message->type()                 // AIS 消息类型
ais_msg.mmsi                    // MMSI 号码
```

---

## 六、性能分析（可选）

### 使用 Valgrind 检测内存泄漏

```bash
valgrind --leak-check=full \
    install/marnav_ais/lib/marnav_ais/ais_parser_node_v2 \
    --ros-args -p comm_type:=udp
```

### 使用 gprof 性能分析

```bash
# 编译时添加 -pg 标志
colcon build --packages-select marnav_ais \
    --cmake-args -DCMAKE_CXX_FLAGS="-pg"

# 运行程序生成 gmon.out
ros2 run marnav_ais ais_parser_node_v2

# 分析性能
gprof install/marnav_ais/lib/marnav_ais/ais_parser_node_v2 gmon.out
```

---

## 七、故障排查

### 问题 1：无法找到符号表
**解决**：确保使用 Debug 模式编译
```bash
colcon build --packages-select marnav_ais --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### 问题 2：断点未命中
**原因**：可能编译器优化了代码
**解决**：检查 CMakeLists.txt 是否设置了 `-O0` 优化级别

### 问题 3：VSCode 找不到可执行文件
**解决**：检查路径是否正确
```bash
ls install/marnav_ais/lib/marnav_ais/ais_parser_node_v2
```

### 问题 4：环境变量未加载
**解决**：在启动调试前 source 环境
```bash
source install/setup.bash
```

---

## 八、快捷键速查

| 功能 | 快捷键 |
|------|--------|
| 启动调试 | `F5` |
| 单步跳过 | `F10` |
| 单步进入 | `F11` |
| 单步跳出 | `Shift+F11` |
| 继续执行 | `F5` |
| 停止调试 | `Shift+F5` |
| 重启调试 | `Ctrl+Shift+F5` |
| 切换断点 | `F9` |
| 运行到光标处 | `Ctrl+F10` |

---

## 九、参考资源

- [VSCode C++ 调试文档](https://code.visualstudio.com/docs/cpp/cpp-debug)
- [GDB 官方文档](https://www.gnu.org/software/gdb/documentation/)
- [ROS2 调试技巧](https://docs.ros.org/en/humble/Tutorials/Debugging.html)
- [Marnav 库文档](https://github.com/mariokonrad/marnav)

---

**提示**：首次调试建议从简单场景开始，如单片段 AIVDM 消息，逐步深入到多片段和批量发布逻辑。

