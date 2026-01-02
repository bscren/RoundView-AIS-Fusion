# YOLO模型兼容性问题修复说明

## 问题描述

在使用C++代码进行YOLO目标检测时出现以下现象：

1. **yolox.onnx模型（本地转换）**：✅ 正常工作
   - 输出节点数：3
   - 检测结果：`[228, 169, 436, 210]` conf=0.830276
   - 无警告信息

2. **Yolo11m.onnx模型（另一台设备转换）**：❌ 输出异常
   - 输出节点数：1
   - 检测结果：`[58156, 153, 639, 258]`（坐标异常）
   - 大量ONNX Runtime警告
   - 但Python调用同一模型可以获得正确结果

## 问题根本原因

### 1. 模型输出格式不同

**YOLOX模型（多输出格式）：**
- 3个输出节点：`output0`, `output1`, `output2`
- 输出格式：`[batch, channels, h, w]`，例如：
  - `[1, 85, 80, 80]`（大特征图）
  - `[1, 85, 40, 40]`（中特征图）
  - `[1, 85, 20, 20]`（小特征图）
- 需要通过网格点和stride解码坐标
- channels = 5 + num_classes（x, y, w, h, obj_conf, cls_conf...）

**YOLOv11模型（端到端格式）：**
- 1个输出节点：`output0`
- 输出格式：`[batch, num_boxes, 6]`，例如 `[1, 300, 6]`
- 已包含NMS后处理，300个候选框
- 6个值：`[x1, y1, x2, y2, confidence, class_id]`
- 坐标已经解码，不需要网格点处理

### 2. 预处理方式不同

**YOLOX：**
```cpp
// 除以255，然后进行ImageNet标准化
normalized = (img / 255.0 - mean) / std
mean = [0.485, 0.456, 0.406]
std = [0.229, 0.224, 0.225]
```

**YOLOv11：**
```cpp
// 只需要除以255
normalized = img / 255.0
// 不需要ImageNet标准化
```

### 3. C++代码的问题

原始C++代码的`decode_outputs`函数（236-329行）专门为YOLOX的多输出格式设计：
- 假设有多个输出特征图
- 生成网格点并应用stride
- 对坐标进行解码

当使用YOLOv11模型时：
- 只有1个输出，不符合多输出假设
- 坐标已经解码，不应该再次应用网格点解码
- 使用ImageNet归一化导致输入数据异常

## 解决方案

### 修改内容

1. **添加模型类型枚举** (`yolo_onnx.h`)
```cpp
enum class YOLOType {
    YOLOX,      // YOLOX格式（多输出，需要decode）
    YOLOv11     // YOLOv11格式（单输出，端到端）
};
```

2. **修改构造函数**，添加`model_type`参数
```cpp
YOLO_ONNX(const std::string& model_path, 
          const std::string& classes_path,
          int input_h = 640, 
          int input_w = 640,
          float confidence = 0.1f,
          float nms_iou = 0.5f,
          bool letterbox_image = true,
          bool use_cuda = true,
          YOLOType model_type = YOLOType::YOLOX);
```

3. **修改预处理函数** (`yolo_onnx.cpp:207-223`)
```cpp
// 归一化
cv::Mat normalized;
letterboxed_img.convertTo(normalized, CV_32F, 1.0 / 255.0);

// YOLOX需要ImageNet标准化，YOLOv11只需要除以255
if (model_type_ == YOLOType::YOLOX) {
    // ImageNet标准化参数
    cv::Scalar mean(0.485, 0.456, 0.406);
    cv::Scalar std(0.229, 0.224, 0.225);
    
    std::vector<cv::Mat> channels;
    cv::split(normalized, channels);
    for (int i = 0; i < 3; i++) {
        channels[i] = (channels[i] - mean[i]) / std[i];
    }
    cv::merge(channels, normalized);
}
```

4. **添加YOLOv11专用后处理函数** (`yolo_onnx.cpp`)
```cpp
std::vector<Detection> YOLO_ONNX::postprocess_yolov11(
    const std::vector<Ort::Value>& outputs,
    const cv::Size& image_shape,
    float scale,
    const std::pair<int, int>& offset) {
    
    // YOLOv11输出格式: [batch, num_boxes, 6]
    // 6个值: [x1, y1, x2, y2, confidence, class_id]
    // 已经包含NMS，不需要额外处理
    
    std::vector<Detection> detections;
    
    const auto& output = outputs[0];
    auto shape = output.GetTensorTypeAndShapeInfo().GetShape();
    const float* data = output.GetTensorData<float>();
    
    int num_boxes = shape[1];
    int values_per_box = shape[2];
    
    for (int i = 0; i < num_boxes; i++) {
        const float* box = data + i * values_per_box;
        
        float x1 = box[0];
        float y1 = box[1];
        float x2 = box[2];
        float y2 = box[3];
        float confidence = box[4];
        int class_id = static_cast<int>(box[5]);
        
        // 置信度过滤
        if (confidence < confidence_) continue;
        
        // 反向letterbox转换
        if (letterbox_image_) {
            x1 = (x1 - offset.first) / scale;
            y1 = (y1 - offset.second) / scale;
            x2 = (x2 - offset.first) / scale;
            y2 = (y2 - offset.second) / scale;
        } else {
            x1 = x1 * image_shape.width / input_w_;
            y1 = y1 * image_shape.height / input_h_;
            x2 = x2 * image_shape.width / input_w_;
            y2 = y2 * image_shape.height / input_h_;
        }
        
        Detection det;
        det.x1 = std::max(0, static_cast<int>(std::floor(x1)));
        det.y1 = std::max(0, static_cast<int>(std::floor(y1)));
        det.x2 = std::min(image_shape.width - 1, static_cast<int>(std::floor(x2)));
        det.y2 = std::min(image_shape.height - 1, static_cast<int>(std::floor(y2)));
        det.class_name = class_names_[class_id];
        det.score = confidence;
        
        detections.push_back(det);
    }
    
    return detections;
}
```

5. **修改使用代码** (`yolo_example.cpp`)
```cpp
// 使用YOLOv11模型
YOLO_ONNX yolo(model_path, classes_path, 
                640, 640,           // input_shape
                0.1f,               // confidence
                0.5f,               // nms_iou
                true,               // letterbox_image
                true,               // use_cuda
                YOLOType::YOLOv11); // 指定模型类型

// 使用YOLOX模型
YOLO_ONNX yolo(model_path, classes_path, 
                640, 640,           // input_shape
                0.1f,               // confidence
                0.5f,               // nms_iou
                true,               // letterbox_image
                true,               // use_cuda
                YOLOType::YOLOX);   // 指定模型类型
```

## 修复结果

### 修复前
```
Yolo11m.onnx输出:
检测到: Cargo Ship 置信度: 0.301194 坐标: [58156, 153, 639, 258]
检测到: Ship 置信度: 0.285511 坐标: [6413, 0, 639, 47]
... (大量错误检测)
```

### 修复后
```
Yolo11m.onnx输出:
检测到: Cargo Ship 置信度: 0.848491 坐标: [227, 161, 434, 212]
```

### 对比验证
- **Python PyTorch**: `[226.5, 168.5, 431.8, 211.0]` conf=0.8290
- **Python ONNX**: `[227.4, 161.4, 434.4, 212.8]` conf=0.8486
- **C++ ONNX (修复后)**: `[227, 161, 434, 212]` conf=0.8486 ✅

结果完全正确且高度一致！

## 关键要点

1. **YOLOX vs YOLOv11的本质区别**：
   - YOLOX：需要后处理（decode + NMS）
   - YOLOv11：端到端模型（已包含NMS）

2. **预处理的重要性**：
   - 不同模型训练时使用的预处理方式不同
   - YOLOv11使用简单的`/255`归一化
   - YOLOX使用ImageNet标准化

3. **模型导出配置**：
   - YOLOv11导出参数：`nms=True`（包含NMS）
   - 输出格式完全不同

4. **调试技巧**：
   - 先用Python验证ONNX模型是否正确
   - 对比Python和C++的预处理流程
   - 检查模型的输入输出节点数量和形状

## 编译和运行

```bash
# 编译
cd /home/tl/RV
colcon build --packages-select marnav_vis_cpp

# 运行
source install/setup.bash
ros2 run marnav_vis_cpp yolo_onnx_node
```

## 总结

这个问题的核心在于**不同YOLO版本的输出格式和预处理方式存在根本性差异**。通过添加模型类型参数并实现对应的预处理和后处理逻辑，成功实现了对多种YOLO模型的兼容支持。

修复后的代码现在可以正确处理：
- ✅ YOLOX系列模型（多输出，需要decode）
- ✅ YOLOv11系列模型（端到端，已包含NMS）

只需在初始化时指定正确的`YOLOType`即可。

