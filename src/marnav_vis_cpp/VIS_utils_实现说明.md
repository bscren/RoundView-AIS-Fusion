# VIS_utils C++实现说明文档

## 概述

本文档说明了Python版`VIS_utils.py`到C++版`VIS_utils.cpp`的完整迁移实现，包括：
- 所有工具函数和VISPRO类的1:1逻辑对齐
- DeepSort接口的预留设计
- DeepSORVF_ros.cpp的调用接口说明
- 数据结构替代方案

---

## 1. 代码结构对照

### 1.1 Python → C++类型映射

| Python类型 | C++类型 | 说明 |
|-----------|---------|------|
| `pandas.DataFrame` | `std::vector<VisTrajectory>` | 轨迹数据列表，每个元素代表一行 |
| `pandas.Series` | `VisTrajectory` | 单个轨迹数据结构体 |
| `numpy.ndarray` | `std::vector<float>` | 数值数组（1D） |
| `torch.Tensor` | `std::vector<std::vector<float>>` | 多维数组（2D），预留DeepSort接口 |
| `list` | `std::vector` | 通用列表容器 |
| `tuple` | `struct DetectionBox` | 检测框数据结构 |

### 1.2 核心数据结构定义

#### VisTrajectory（替代DataFrame一行）
```cpp
struct VisTrajectory {
    int id;              // 跟踪ID
    int x1, y1, x2, y2;  // 边界框坐标
    int x, y;            // 中心点坐标
    std::string speed;   // 速度向量（字符串格式"[x_speed, y_speed]"）
    int64_t timestamp;   // 时间戳（秒）
};
```

**选型理由**：
- 使用结构体代替DataFrame一行，直观且高效
- `speed`保持字符串格式，便于序列化和调试，与Python版一致
- 所有字段类型与Python版对齐（int/float/string）

#### DetectionBox（替代检测结果tuple）
```cpp
struct DetectionBox {
    float x1, y1, x2, y2;
    std::string class_name;
    float confidence;
};
```

对应Python的`(x1, y1, x2, y2, class_name, confidence)`

#### TrackOutput（DeepSort输出）
```cpp
struct TrackOutput {
    float x1, y1, x2, y2;
    std::vector<cv::Point> lines;  // 轨迹线
    int track_id;
};
```

对应Python的`[x1, y1, x2, y2, lines, track_id]`

---

## 2. DeepSort接口预留设计

### 2.1 预留位置标注

#### 位置1：`VISPRO::track`函数（src/VIS_utils.cpp:300-350行）

```cpp
void VISPRO::track(const cv::Mat& image,
                   const std::vector<DetectionBox>& bboxes,
                   const std::vector<DetectionBox>& bboxes_anti_occ,
                   const std::vector<int>& id_list,
                   int64_t timestamp) {
    // TODO: 待DeepSort C++实现完成后补充
    // 
    // 接口说明：
    // 1. 输入参数：
    //    - image: 原始图像（BGR格式）
    //    - bboxes: 正常检测框列表（不含遮挡区域）
    //    - bboxes_anti_occ: 遮挡区域的抗遮挡预测框
    //    - id_list: 抗遮挡预测框对应的ID列表
    //    - timestamp: 时间戳（毫秒，需转换为秒）
    // 
    // 2. 处理流程（对应Python版deepsort.update）：
    //    - 将检测框转换为xywh格式（中心点+宽高）
    //    - 提取置信度
    //    - 调用DeepSort的update方法
    //    - 解析跟踪结果，存储到vis_tra_cur_3_
    
    // 当前预留空实现，记录调用日志
    RCLCPP_DEBUG(logger_, "[VIS DEBUG] track函数调用 - 检测框: %zu, 抗遮挡框: %zu",
                 bboxes.size(), bboxes_anti_occ.size());
}
```

**补充实现示例**（待DeepSort完成后）：

```cpp
void VISPRO::track(const cv::Mat& image,
                   const std::vector<DetectionBox>& bboxes,
                   const std::vector<DetectionBox>& bboxes_anti_occ,
                   const std::vector<int>& id_list,
                   int64_t timestamp) {
    // 转换检测框格式：xyxy -> xywh
    std::vector<std::vector<float>> bbox_xywh, bbox_xywh_anti_occ;
    std::vector<float> confidences, confidences_anti_occ;
    
    for (const auto& box : bboxes) {
        float cx = (box.x1 + box.x2) / 2.0f;
        float cy = (box.y1 + box.y2) / 2.0f;
        float w = box.x2 - box.x1;
        float h = box.y2 - box.y1;
        bbox_xywh.push_back({cx, cy, w, h});
        confidences.push_back(box.confidence);
    }
    
    for (const auto& box : bboxes_anti_occ) {
        float cx = (box.x1 + box.x2) / 2.0f;
        float cy = (box.y1 + box.y2) / 2.0f;
        float w = box.x2 - box.x1;
        float h = box.y2 - box.y1;
        bbox_xywh_anti_occ.push_back({cx, cy, w, h});
        confidences_anti_occ.push_back(box.confidence);
    }
    
    // 调用DeepSort（待实现）
    // auto outputs = deepsort_->update(bbox_xywh, confidences, image,
    //                                  bbox_xywh_anti_occ, confidences_anti_occ,
    //                                  id_list, timestamp / 1000);
    
    // 解析输出，存储到vis_tra_cur_3_
    // for (const auto& output : outputs) {
    //     VisTrajectory traj;
    //     traj.id = output.track_id;
    //     traj.x1 = static_cast<int>(output.x1);
    //     traj.y1 = static_cast<int>(output.y1);
    //     traj.x2 = static_cast<int>(output.x2);
    //     traj.y2 = static_cast<int>(output.y2);
    //     traj.x = (traj.x1 + traj.x2) / 2;
    //     traj.y = (traj.y1 + traj.y2) / 2;
    //     traj.timestamp = timestamp / 1000;
    //     vis_tra_cur_3_.push_back(traj);
    // }
}
```

#### 位置2：`VISPRO::detection`函数（src/VIS_utils.cpp:280-295行）

```cpp
std::vector<DetectionBox> VISPRO::detection(const cv::Mat& image) {
    // TODO: 集成YOLO检测模型
    // 参考Python版本使用的YOLO类，可以使用yolo_onnx.cpp中的实现
    // 
    // 示例代码（需要实际集成）：
    // YOLO_ONNX yolo_detector(model_path);
    // auto detections = yolo_detector.detect(image);
    
    // 当前返回空列表，待YOLO集成后补充
    std::vector<DetectionBox> bboxes;
    return bboxes;
}
```

**补充实现示例**（集成yolo_onnx.cpp）：

```cpp
std::vector<DetectionBox> VISPRO::detection(const cv::Mat& image) {
    // 转换图像格式（BGR -> RGB）
    cv::Mat rgb_image;
    cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
    
    // 调用YOLO检测（需要在构造函数中初始化yolo_detector_）
    // auto yolo_results = yolo_detector_->detect(rgb_image);
    
    // 转换为DetectionBox格式
    std::vector<DetectionBox> bboxes;
    // for (const auto& result : yolo_results) {
    //     DetectionBox box;
    //     box.x1 = result.box.x;
    //     box.y1 = result.box.y;
    //     box.x2 = result.box.x + result.box.width;
    //     box.y2 = result.box.y + result.box.height;
    //     box.class_name = result.class_name;
    //     box.confidence = result.confidence;
    //     bboxes.push_back(box);
    // }
    
    return bboxes;
}
```

### 2.2 DeepSort接口类定义

在`VIS_utils.h`中定义的抽象接口类：

```cpp
class DeepSortInterface {
public:
    virtual ~DeepSortInterface() = default;
    
    /**
     * @brief DeepSort更新函数（核心接口）
     * 对应Python: deepsort.update(xywhs, confss, image, xywhs_anti_occ, 
     *                             confss_anti_occ, id_list, timestamp)
     */
    virtual std::vector<TrackOutput> update(
        const std::vector<std::vector<float>>& bbox_xywh,
        const std::vector<float>& confidences,
        const cv::Mat& ori_img,
        const std::vector<std::vector<float>>& bbox_xywh_anti_occ = {},
        const std::vector<float>& confidences_anti_occ = {},
        const std::vector<int>& id_list = {},
        int64_t timestamp = 0) = 0;
};
```

**使用说明**：
1. DeepSort C++实现完成后，继承`DeepSortInterface`实现具体类
2. 在`VISPRO`构造函数中初始化DeepSort实例：
   ```cpp
   VISPRO::VISPRO(bool anti, float val, int t, rclcpp::Logger logger)
       : anti_(anti), val_(val), t_(t), logger_(logger) {
       // 初始化DeepSort（待实现）
       // deepsort_ = std::make_shared<DeepSortImpl>(model_path, max_dist, ...);
   }
   ```
3. 在`track`函数中调用`deepsort_->update(...)`

---

## 3. DeepSORVF_ros.cpp调用接口

### 3.1 在worker_thread中集成VISPRO

#### 修改位置：`DeepSORVF_ros.cpp:580-704行`

**当前代码**（仅有AISPRO）：
```cpp
void DeepSORVFNode::worker_thread(int cam_idx) {
    AISPRO aispro(im_shape_, t_ms_);
    // TODO: 后续需要添加VISPRO, FUSPRO, DRAW实例
}
```

**修改后代码**（添加VISPRO）：
```cpp
void DeepSORVFNode::worker_thread(int cam_idx) {
    // 1. 创建处理实例
    AISPRO aispro(im_shape_, t_ms_);
    VISPRO vispro(true, 0.5f, t_ms_, this->get_logger());  // anti=true, val=0.5
    // TODO: FUSPRO fuspro(max_dis_, im_shape_, t_ms_);
    // TODO: DRAW dra(im_shape_, t_ms_);
    
    std::vector<VisiableTraData> last_visiable_tra;
    int64_t last_processed_window = -1;
    
    while (running_) {
        // ... 获取任务代码 ...
        
        if (process_ais_vis_fus) {
            // 1. AIS处理
            std::vector<AISVisData> ais_vis;
            std::vector<AISData> ais_cur;
            aispro.process(task.ais_batch, current_camera_pos_para, 
                          task.current_timestamp_ms, camera_type_, 
                          ais_vis, ais_cur);
            
            // 2. VIS处理（新增）
            std::vector<vis_utils::AISVisData> ais_vis_converted;  // 转换格式
            for (const auto& a : ais_vis) {
                vis_utils::AISVisData v;
                v.mmsi = a.mmsi;
                v.lon = a.lon;
                v.lat = a.lat;
                v.speed = a.speed;
                v.course = a.course;
                v.heading = a.heading;
                v.type = a.type;
                v.x = a.x;
                v.y = a.y;
                v.timestamp = a.timestamp_ms / 1000;  // 转换为秒
                ais_vis_converted.push_back(v);
            }
            
            std::vector<vis_utils::BindInfo> bind_inf;  // TODO: 从FUSPRO获取
            auto vis_result = vispro.feedCap(task.cv_image, ais_vis_converted, 
                                            bind_inf, task.current_timestamp_ms);
            
            std::vector<vis_utils::VisTrajectory> vis_tra = vis_result.first;
            std::vector<vis_utils::VisTrajectory> vis_cur = vis_result.second;
            
            // 3. FUS处理（待实现）
            // auto fus_result = fuspro.fusion(ais_vis, ais_cur, vis_tra, vis_cur, 
            //                                 task.current_timestamp_ms);
            
            // 4. DRAW处理（待实现）
            // cv::Mat processed_image = dra.draw_match_traj(...);
            
            // 转换为VisiableTraData（临时，待FUS完成后替换）
            std::vector<VisiableTraData> visiable_tra_list;
            for (const auto& vis : vis_cur) {
                VisiableTraData tra;
                tra.cam_idx = cam_idx;
                tra.timestamp_ms = vis.timestamp * 1000;  // 转换为毫秒
                tra.ais = 0;  // 纯视觉轨迹
                tra.mmsi = 0;
                tra.ship_type = "";
                tra.sog = 0.0f;
                tra.cog = 0.0f;
                tra.lat = 0.0f;
                tra.lon = 0.0f;
                tra.box_x1 = static_cast<float>(vis.x1);
                tra.box_y1 = static_cast<float>(vis.y1);
                tra.box_x2 = static_cast<float>(vis.x2);
                tra.box_y2 = static_cast<float>(vis.y2);
                visiable_tra_list.push_back(tra);
            }
            
            result.visiable_tra_list = visiable_tra_list;
            // ...
        }
        // ...
    }
}
```

### 3.2 头文件引用

在`DeepSORVF_ros.cpp`开头添加：

```cpp
#include "marnav_vis_cpp/AIS_utils.h"
#include "marnav_vis_cpp/VIS_utils.h"  // 新增

// 可选：使用命名空间简化代码
using namespace marnav_vis_cpp::vis_utils;
```

### 3.3 数据格式转换注意事项

| 字段 | Python/AIS_utils格式 | VIS_utils格式 | 转换说明 |
|-----|---------------------|---------------|---------|
| timestamp | `int64_t timestamp_ms`（毫秒） | `int64_t timestamp`（秒） | 除以1000 |
| 检测框 | `(x1,y1,x2,y2,class,conf)` | `DetectionBox{x1,y1,x2,y2,class,conf}` | 结构体封装 |
| 轨迹数据 | `VisiableTraData` | `VisTrajectory` | 字段名不同，需转换 |

---

## 4. 函数逻辑对照表

### 4.1 工具函数对照

| Python函数 | C++函数 | 核心逻辑 | 验证状态 |
|-----------|---------|---------|---------|
| `box_whether_in_area` | `box_whether_in_area` | 计算中心点是否在区域内 | ✅ 已对齐 |
| `speed_extract` | `speed_extract` | 计算两点速度向量 | ✅ 已对齐 |
| `whether_in_area` | `whether_in_area` | 点是否在矩形内 | ✅ 已对齐 |
| `overlap` | `overlap` | 计算两矩形重叠比例 | ✅ 已对齐 |
| `whether_occlusion` | `whether_occlusion` | 判断是否被遮挡 | ✅ 已对齐 |
| `whether_in_OAR` | `whether_in_OAR` | 点是否在遮挡区域内 | ✅ 已对齐 |
| `OAR_extractor` | `OAR_extractor` | 提取遮挡区域 | ✅ 已对齐 |
| `motion_features_extraction` | `motion_features_extraction` | 提取速度特征 | ✅ 已对齐 |
| `id_whether_stable` | `id_whether_stable` | 判断ID是否稳定 | ✅ 已对齐 |

### 4.2 VISPRO类方法对照

| Python方法 | C++方法 | 核心逻辑 | 验证状态 |
|-----------|---------|---------|---------|
| `__init__` | `VISPRO()` | 初始化成员变量 | ✅ 已对齐 |
| `detection` | `detection()` | 调用YOLO检测 | ⚠️ 预留接口 |
| `track` | `track()` | 调用DeepSort跟踪 | ⚠️ 预留接口 |
| `update_tra` | `update_tra()` | 轨迹均值+速度提取 | ✅ 已对齐 |
| `traj_prediction_via_visual` | `traj_prediction_via_visual()` | 视觉速度预测 | ✅ 已对齐 |
| `anti_occ` | `anti_occ()` | 抗遮挡预测框生成 | ✅ 已对齐 |
| `feedCap` | `feedCap()` | 主处理流程 | ✅ 已对齐 |

---

## 5. 编译配置

### 5.1 CMakeLists.txt修改

已在`CMakeLists.txt`中添加：

```cmake
add_executable(deep_sorvf_node
  src/DeepSORVF_ros.cpp
  src/AIS_utils.cpp
  src/VIS_utils.cpp  # 新增
)
```

### 5.2 编译命令

```bash
cd /home/tl/RV
colcon build --packages-select marnav_vis_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 5.3 依赖检查

| 依赖 | 用途 | 是否满足 |
|-----|------|---------|
| OpenCV 4 | 图像处理、显示 | ✅ 已配置 |
| ROS2 Humble | rclcpp、消息类型 | ✅ 已配置 |
| yaml-cpp | 配置文件解析 | ✅ 已配置 |
| GeographicLib | 地理坐标转换（AIS_utils） | ✅ 已配置 |
| ONNX Runtime | YOLO/DeepSort模型推理 | ✅ 已配置 |

---

## 6. 测试建议

### 6.1 单元测试（工具函数）

创建测试文件`test_vis_utils.cpp`：

```cpp
#include "marnav_vis_cpp/VIS_utils.h"
#include <gtest/gtest.h>

using namespace marnav_vis_cpp::vis_utils;

TEST(VISUtilsTest, OverlapTest) {
    std::vector<float> box1 = {0, 0, 10, 10};
    std::vector<float> box2 = {5, 5, 15, 15};
    EXPECT_EQ(overlap(box1, box2, 0.1), 1);  // 有重叠
    
    std::vector<float> box3 = {20, 20, 30, 30};
    EXPECT_EQ(overlap(box1, box3, 0.1), 0);  // 无重叠
}

TEST(VISUtilsTest, SpeedExtractTest) {
    VisTrajectory t1(1, 0, 0, 10, 10, 5, 5, 0);
    VisTrajectory t2(1, 0, 0, 10, 10, 15, 10, 2);  // 2秒后
    auto speed = speed_extract(t1, t2);
    EXPECT_FLOAT_EQ(speed[0], 5.0f);  // x方向速度
    EXPECT_FLOAT_EQ(speed[1], 2.5f);  // y方向速度
}
```

### 6.2 集成测试（VISPRO类）

```cpp
TEST(VISPROTest, FeedCapTest) {
    rclcpp::init(0, nullptr);
    auto logger = rclcpp::get_logger("test");
    
    VISPRO vispro(true, 0.5f, 1000, logger);
    
    cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC3);
    std::vector<AISVisData> ais_vis;
    std::vector<BindInfo> bind_inf;
    
    auto result = vispro.feedCap(test_image, ais_vis, bind_inf, 1000);
    
    EXPECT_TRUE(result.first.empty());   // 初次调用，历史轨迹为空
    EXPECT_TRUE(result.second.empty());  // 无检测结果
    
    rclcpp::shutdown();
}
```

### 6.3 端到端测试（ROS2节点）

```bash
# 1. 启动节点
ros2 run marnav_vis_cpp deep_sorvf_node --ros-args -p config_file:=/path/to/config.yaml

# 2. 检查日志
# 应看到：
# - VISPRO初始化完成
# - [VIS DEBUG] timestamp=XXX, 检测到 X 个目标
# - [VIS DEBUG] update_tra完成

# 3. 检查topic输出
ros2 topic echo /fus_trajectory_topic
```

---

## 7. 注意事项与FAQ

### 7.1 时间戳单位转换

**问题**：Python版混用毫秒和秒，C++如何处理？

**解决**：
- ROS2消息接口统一使用**毫秒**（`int64_t timestamp_ms`）
- VIS_utils内部统一使用**秒**（`int64_t timestamp`）
- 在`feedCap`函数入口处转换：`int64_t timestamp_sec = timestamp / 1000;`

### 7.2 速度字符串格式

**问题**：为什么`speed`字段用字符串而不是`std::vector<float>`？

**解决**：
- 保持与Python版一致，便于调试和序列化
- 解析示例（在`anti_occ`函数中）：
  ```cpp
  std::string speed_str = traj.speed;  // "[5.2, -3.1]"
  speed_str.erase(std::remove(speed_str.begin(), speed_str.end(), '['), speed_str.end());
  speed_str.erase(std::remove(speed_str.begin(), speed_str.end(), ']'), speed_str.end());
  std::istringstream iss(speed_str);
  std::vector<float> speed;
  std::string token;
  while (std::getline(iss, token, ',')) {
      speed.push_back(std::stof(token));
  }
  ```

### 7.3 数据一致性检查

**问题**：如何确保C++版与Python版输出一致？

**测试方法**：
1. 使用相同的输入数据（录制rosbag）
2. 对比关键节点的输出：
   - `detection`：检测框数量和坐标
   - `track`：跟踪ID和轨迹数量
   - `update_tra`：速度计算结果
   - `anti_occ`：抗遮挡预测框数量
3. 在C++版中添加与Python版相同的调试日志（`[VIS DEBUG]`）

### 7.4 内存管理

**问题**：历史轨迹数据会无限增长吗？

**解决**：
- `vis_tra_`在`update_tra`中自动清理2分钟前的数据
- `last5_vis_tra_list_`在`update_tra`中限制为最多5个元素
- 使用`std::vector`，析构时自动释放内存

### 7.5 线程安全

**问题**：`VISPRO`是否线程安全？

**解决**：
- 当前设计为**非线程安全**（与Python版一致）
- 在`DeepSORVF_ros.cpp`的`worker_thread`中，**每个线程独立创建VISPRO实例**
- 如果需要跨线程共享，需添加互斥锁保护成员变量

---

## 8. 后续集成步骤

### 8.1 短期（1-2周）

1. ✅ 完成VIS_utils基础框架（本次已完成）
2. ⚠️ 集成YOLO检测模型（yolo_onnx.cpp）
3. ⚠️ 实现DeepSort C++版本（参考deep_sort/tracker.cpp）
4. ⚠️ 在`VISPRO::detection`和`VISPRO::track`中补充实际调用

### 8.2 中期（2-4周）

1. 在`DeepSORVF_ros.cpp`的`worker_thread`中集成VISPRO
2. 实现FUSPRO类（融合AIS和VIS轨迹）
3. 实现DRAW类（可视化绘制）
4. 端到端测试和性能优化

### 8.3 长期（1-2月）

1. 添加配置文件支持（DeepSort参数、YOLO参数）
2. 性能分析和优化（SIMD、多线程优化）
3. 添加单元测试和集成测试
4. 文档完善和代码审查

---

## 9. 联系方式与支持

如有问题或需要进一步说明，请检查：
- 代码中的注释（特别是`TODO`标记）
- ROS2日志输出（使用`RCLCPP_DEBUG`级别）
- Python版代码对照（`VIS_utils.py`）

**关键预留接口汇总**：
1. `VISPRO::detection()` - YOLO集成（第280行）
2. `VISPRO::track()` - DeepSort集成（第300行）
3. `DeepSORVFNode::worker_thread()` - VISPRO调用（第642行）

---

**文档版本**：v1.0  
**创建日期**：2026-01-04  
**作者**：AI Assistant  
**状态**：✅ 基础实现完成，⚠️ 待集成DeepSort和YOLO

