# 图像拼接性能波动分析报告

## 📊 问题描述

在注释掉缝合线刷新定时器后，拼接处理耗时仍然在 60ms-200ms 之间波动，波动幅度达到 3-4 倍。

## 🔍 已添加的性能监控

### 1. **拼接器层面** (`JH_stitcher.cpp::processSubsequentGroupImpl`)

监控以下9个步骤的耗时（仅当总耗时>100ms时输出详细信息）：

```
步骤1: 读取变换数据（含mutex锁）
步骤2: 图像扭曲（CylindricalWarper）
步骤3: 写入缓存（独占shared_mutex锁）
步骤4: 创建Blender
步骤5: 类型转换 & Feed
步骤6: Blend融合
步骤7: 图像缩放
步骤8: 轨迹框投影
步骤9: 裁剪
```

### 2. **节点层面** (`JH_ros2_stitch_node.cpp::processSubsequentGroup`)

监控5个步骤的耗时（仅当总耗时>100ms时输出）：

```
步骤A: 获取轨迹缓存锁
步骤B: 调用拼接器处理（核心）
步骤C: 获取轨迹框
步骤D: 获取GNSS缓存锁
步骤E: 发布拼接图像
```

### 3. **图像发布层面** (`publishStitchedImage`)

监控5个子步骤（当耗时>10ms时输出DEBUG级别日志）：

```
发布步骤1: 图像类型转换
发布步骤2: 构建消息头
发布步骤3: 构建JSON消息
发布步骤4: JPEG编码
发布步骤5: ROS2发布
```

### 4. **回调函数层面** (`image_callback`)

监控图像转换和后续处理的总耗时。

## 🎯 如何使用性能监控

### 运行测试

```bash
cd /home/tl/RV
source install/setup.bash

# 运行您的launch文件
ros2 launch image_stitching_pkg <your_launch_file>.py

# 或者运行ROS2节点
ros2 run image_stitching_pkg JH_ROS_stitch
```

### 查看输出日志

**正常情况（<100ms）**：
```
[INFO] PSG处理耗时: 65 毫秒
[INFO] PSG处理耗时: 67 毫秒
[INFO] PSG处理耗时: 62 毫秒
```

**性能异常（>100ms）**：
```
[INFO] PSG处理耗时: 222 毫秒
⚠️ [回调层性能] 总耗时: 222ms (图像转换:1ms, 后续处理:221ms)
⚠️ [节点层性能] 总耗时: 220 ms
  步骤A-获取轨迹锁: 0 ms | 步骤B-拼接处理: 218 ms | 步骤C-获取轨迹框: 0 ms | 步骤D-获取GNSS锁: 0 ms | 步骤E-发布图像: 2 ms
⚠️ [性能分析] 总耗时: 218ms (>100ms)
  ├─ 步骤1-读取变换数据: 0ms
  ├─ 步骤2-图像扭曲: 120ms        ← 主要瓶颈！
  ├─ 步骤3-写入缓存(锁): 5ms
  ├─ 步骤4-创建Blender: 2ms
  ├─ 步骤5-类型转换&Feed: 10ms
  ├─ 步骤6-Blend融合: 65ms       ← 次要瓶颈
  ├─ 步骤7-图像缩放: 8ms
  ├─ 步骤8-轨迹框投影: 5ms
  └─ 步骤9-裁剪: 3ms
```

## 🔬 可能的性能波动原因分析

### 原因1: GPU与CPU之间的上下文切换 ⭐⭐⭐⭐⭐

**最有可能的原因**

OpenCV使用 `CylindricalWarperGpu` 进行图像扭曲，GPU操作存在：
- **首次调用开销**：GPU上下文初始化
- **数据传输开销**：CPU→GPU内存传输
- **内核切换开销**：GPU在不同任务间切换
- **功率管理**：GPU动态调频（P-state切换）

**验证方法**：
```bash
# 监控GPU使用情况
watch -n 0.1 nvidia-smi
```

**解决方案**：
```cpp
// 在 JH_stitcher.cpp 构造函数中固定GPU频率
cv::cuda::setDevice(0);
cv::cuda::DeviceInfo deviceInfo(0);
std::cout << "GPU: " << deviceInfo.name() << std::endl;

// 或改用CPU warping（稳定但较慢）
Ptr<CylindricalWarper> warper = makePtr<CylindricalWarper>(avg_focal);
```

### 原因2: OpenCV内存分配器的GC周期 ⭐⭐⭐⭐

OpenCV的UMat和Mat内存管理可能触发垃圾回收：
- **小波动周期**：频繁的小对象分配/释放
- **大波动周期**：内存池重组

**解决方案**：
```cpp
// 预分配内存，避免动态分配
static cv::Mat preallocated_warp[3];
if (preallocated_warp[0].empty()) {
    for (int i = 0; i < 3; i++) {
        preallocated_warp[i].create(expected_size, CV_32FC3);
    }
}
```

### 原因3: Blender内部的自适应算法 ⭐⭐⭐

`Blender::NO` 模式虽然快，但可能有内部优化逻辑：
- 根据图像内容调整策略
- 边缘检测的复杂度不同

**验证方法**：观察步骤6（Blend融合）的耗时是否波动明显

### 原因4: CPU调度与负载波动 ⭐⭐⭐

系统中其他进程（ROS2节点、相机接收、AIS处理等）争抢CPU：

**验证方法**：
```bash
# 实时监控CPU使用率
htop

# 查看该进程的CPU亲和性
ps -eLo pid,tid,psr,comm | grep JH_ROS_stitch
```

**解决方案**：
```bash
# 绑定到特定CPU核心
taskset -c 0-3 ros2 run image_stitching_pkg JH_ROS_stitch

# 提高进程优先级
sudo nice -n -10 ros2 run image_stitching_pkg JH_ROS_stitch
```

### 原因5: 图像内容复杂度差异 ⭐⭐

虽然图像大小不变，但内容复杂度会影响：
- **扭曲插值**：纹理密集区域计算量大
- **融合**：边缘复杂度不同

**验证方法**：观察慢帧时的场景是否更复杂（如波浪、密集目标）

### 原因6: ROS2 DDS中间件的网络波动 ⭐

ROS2的DDS层可能有：
- 消息队列堆积
- 网络I/O波动

**验证方法**：
```bash
# 查看ROS2话题统计
ros2 topic hz /rtsp_image_0
ros2 topic bw /rtsp_image_0
```

### 原因7: 共享锁的读者数量波动 ⭐

虽然注释掉了定时器，但 `rw_mutex_` 仍被使用：
```cpp
std::unique_lock<std::shared_mutex> lock(rw_mutex_);  // 步骤3
```

如果有其他地方读取 `latest_warp_images_32F`，会影响锁性能。

**验证方法**：观察步骤3（写入缓存）的耗时是否波动

## 📈 测试与诊断步骤

### Step 1: 运行带性能监控的节点

```bash
cd /home/tl/RV
source install/setup.bash
ros2 run image_stitching_pkg JH_ROS_stitch 2>&1 | tee performance_log.txt
```

### Step 2: 记录至少100帧的数据

让系统运行2-3分钟，记录足够多的样本。

### Step 3: 分析日志

```bash
# 提取慢帧（>100ms）的详细信息
grep "性能分析" performance_log.txt

# 统计各步骤的耗时分布
grep "步骤2-图像扭曲" performance_log.txt | awk '{print $NF}' | sort -n
```

### Step 4: 对比GPU/CPU模式

**当前使用**：`CylindricalWarperGpu`（步骤2）

**测试CPU模式**：
```cpp
// 临时修改 JH_stitcher.cpp:236
// Ptr<cv::detail::CylindricalWarperGpu> warper = makePtr<cv::detail::CylindricalWarperGpu>(avg_focal);
Ptr<cv::detail::CylindricalWarper> warper = makePtr<cv::detail::CylindricalWarper>(avg_focal);
```

如果CPU模式更稳定（虽然可能整体更慢），说明问题在GPU。

## 🎯 预期的诊断结果

根据您的输出模式（60ms → 200ms → 60ms），最可能的情况是：

**主要瓶颈**：步骤2（图像扭曲）- GPU上下文切换或P-state变化
**次要瓶颈**：步骤6（Blend融合）- 内存分配或算法复杂度

观察日志中这两个步骤的耗时波动模式，就能确认根本原因。

## 🛠️ 优化建议

### 短期优化（立即可用）

1. **监控GPU状态**
   ```bash
   nvidia-smi dmon -s u -c 100 > gpu_usage.log &
   ```

2. **固定GPU频率**
   ```bash
   sudo nvidia-smi -pm 1
   sudo nvidia-smi -lgc 1500  # 锁定GPU时钟
   ```

### 中期优化（需要代码修改）

1. **预分配内存池**：避免动态分配
2. **分离GPU流**：异步处理3个相机
3. **使用OpenCV的异步API**：重叠计算与传输

### 长期优化（架构调整）

1. **使用GPU Pipeline**：将扭曲、融合都放在GPU上
2. **多GPU负载均衡**：每个相机用独立GPU
3. **CUDA自定义内核**：优化关键步骤

## 📝 日志格式说明

```
[INFO] PSG处理耗时: XX 毫秒              ← 每帧都输出
⚠️ [回调层性能] ...                      ← 仅当 >100ms
⚠️ [节点层性能] ...                      ← 仅当 >100ms  
⚠️ [性能分析] 总耗时: XXms (>100ms)      ← 仅当 >100ms
  ├─ 步骤1-读取变换数据: Xms           
  ├─ 步骤2-图像扭曲: Xms                ← 关注这个！
  ├─ 步骤3-写入缓存(锁): Xms           
  ├─ 步骤4-创建Blender: Xms            
  ├─ 步骤5-类型转换&Feed: Xms          
  ├─ 步骤6-Blend融合: Xms               ← 关注这个！
  ├─ 步骤7-图像缩放: Xms               
  ├─ 步骤8-轨迹框投影: Xms             
  └─ 步骤9-裁剪: Xms                   
```

## 🚀 下一步行动

1. ✅ 运行节点并收集日志
2. ⏳ 分析慢帧时的步骤耗时分布
3. ⏳ 根据瓶颈步骤采取对应优化措施
4. ⏳ 验证优化效果

---

**更新日期**: 2025-12-12  
**版本**: 1.0

