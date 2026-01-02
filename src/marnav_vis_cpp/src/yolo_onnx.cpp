// src/yolo_onnx.cpp
#include "marnav_vis_cpp/yolo_onnx.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <chrono>

YOLO_ONNX::YOLO_ONNX(const std::string& model_path,
                       const std::string& classes_path,
                       int input_h,
                       int input_w,
                       float confidence,
                       float nms_iou,
                       bool letterbox_image,
                       bool use_cuda,
                       YOLOType model_type)
    : env_(ORT_LOGGING_LEVEL_WARNING, "YOLO"),
      session_(nullptr),
      input_h_(input_h),
      input_w_(input_w),
      confidence_(confidence),
      nms_iou_(nms_iou),
      letterbox_image_(letterbox_image),
      use_cuda_(use_cuda),
      model_type_(model_type) {
    
    // 加载类别
    load_classes(classes_path);
    
    // 配置会话选项
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    
    // 如果使用CUDA，添加CUDA执行提供者
    if (use_cuda) {
        try {
            OrtCUDAProviderOptions cuda_options{};
            session_options_.AppendExecutionProvider_CUDA(cuda_options);
            std::cout << "CUDA执行提供者已启用" << std::endl;
        } catch (...) {
            std::cout << "CUDA不可用，使用CPU" << std::endl;
            use_cuda_ = false;
        }
    }
    
    // 加载ONNX模型
    // #region agent log
    // {
    //     std::ofstream log_file("/home/tl/RV/.cursor/debug.log", std::ios::app);
    //     if (log_file.is_open()) {
    //         log_file << "{\"sessionId\":\"debug-session\",\"runId\":\"pre-fix\",\"hypothesisId\":\"B\",\"location\":\"yolo_onnx.cpp:45\",\"message\":\"Loading ONNX model\",\"data\":{\"model_path\":\"" << model_path << "\",\"path_type\":\"string\"},\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << "}\n";
    //         log_file.close();
    //     }
    // }
    // #endregion
    // Linux上使用const char*，Windows上使用wchar_t*
    #ifdef _WIN32
        std::wstring wmodel_path(model_path.begin(), model_path.end());
        session_ = new Ort::Session(env_, wmodel_path.c_str(), session_options_);
    #else
        session_ = new Ort::Session(env_, model_path.c_str(), session_options_);
    #endif
    // #region agent log
    // {
    //     std::ofstream log_file("/home/tl/RV/.cursor/debug.log", std::ios::app);
    //     if (log_file.is_open()) {
    //         log_file << "{\"sessionId\":\"debug-session\",\"runId\":\"pre-fix\",\"hypothesisId\":\"B\",\"location\":\"yolo_onnx.cpp:52\",\"message\":\"ONNX model loaded successfully\",\"data\":{\"session_created\":true},\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << "}\n";
    //         log_file.close();
    //     }
    // }
    // #endregion
    
    // 获取输入输出信息
    size_t num_input_nodes = session_->GetInputCount();
    size_t num_output_nodes = session_->GetOutputCount();
    
    // 清空并预留空间（使用 reserve 而不是 resize，避免空字符串）
    input_names_.clear();
    output_names_.clear();
    input_shapes_.clear();
    output_shapes_.clear();
    input_names_.reserve(num_input_nodes);
    output_names_.reserve(num_output_nodes);
    input_shapes_.reserve(num_input_nodes);
    output_shapes_.reserve(num_output_nodes);
    
    // 获取输入信息
    for (size_t i = 0; i < num_input_nodes; i++) {
        auto input_name = session_->GetInputNameAllocated(i, allocator_);
        std::string input_name_str(input_name.get());
        input_names_.push_back(input_name_str);
        
        auto input_type_info = session_->GetInputTypeInfo(i);
        auto tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        auto shape = tensor_info.GetShape();
        input_shapes_.push_back(shape);
        
        // #region agent log
        // {
        //     std::ofstream log_file("/home/tl/RV/.cursor/debug.log", std::ios::app);
        //     if (log_file.is_open()) {
        //         log_file << "{\"sessionId\":\"debug-session\",\"runId\":\"post-fix\",\"hypothesisId\":\"C\",\"location\":\"yolo_onnx.cpp:95\",\"message\":\"Input name retrieved\",\"data\":{\"index\":" << i << ",\"name\":\"" << input_name_str << "\",\"name_length\":" << input_name_str.length() << "},\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << "}\n";
        //         log_file.close();
        //     }
        // }
        // #endregion
    }
    
    // 获取输出信息
    for (size_t i = 0; i < num_output_nodes; i++) {
        auto output_name = session_->GetOutputNameAllocated(i, allocator_);
        std::string output_name_str(output_name.get());
        output_names_.push_back(output_name_str);
        
        auto output_type_info = session_->GetOutputTypeInfo(i);
        auto tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        auto shape = tensor_info.GetShape();
        output_shapes_.push_back(shape);
        
        // #region agent log
        // {
        //     std::ofstream log_file("/home/tl/RV/.cursor/debug.log", std::ios::app);
        //     if (log_file.is_open()) {
        //         log_file << "{\"sessionId\":\"debug-session\",\"runId\":\"post-fix\",\"hypothesisId\":\"C\",\"location\":\"yolo_onnx.cpp:115\",\"message\":\"Output name retrieved\",\"data\":{\"index\":" << i << ",\"name\":\"" << output_name_str << "\",\"name_length\":" << output_name_str.length() << "},\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << "}\n";
        //         log_file.close();
        //     }
        // }
        // #endregion
    }
    
    std::cout << "YOLO ONNX模型加载成功: " << model_path << std::endl;
    std::cout << "类别数量: " << num_classes_ << std::endl;
    std::cout << "输入节点数: " << input_names_.size() << std::endl;
    std::cout << "输出节点数: " << output_names_.size() << std::endl;

    // 预热CUDA
    if (use_cuda) {
        std::cout << "预热CUDA" << std::endl;
        // 在构造函数末尾或首次使用前
        cv::Mat dummy = cv::Mat::zeros(640, 640, CV_8UC3);
        detect_image(dummy);  // 预热，初始化CUDA上下文
        std::cout << "CUDA预热完成" << std::endl;
    }
}

YOLO_ONNX::~YOLO_ONNX() {
    if (session_) {
        delete session_;
    }
}

void YOLO_ONNX::load_classes(const std::string& classes_path) {
    std::ifstream file(classes_path);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开类别文件: " + classes_path);
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            class_names_.push_back(line);
        }
    }
    num_classes_ = class_names_.size();
    std::cout << "加载了 " << num_classes_ << " 个类别" << std::endl;
}

std::vector<float> YOLO_ONNX::preprocess_image(const cv::Mat& image,
                                                 cv::Mat& letterboxed_img,
                                                 float& scale,
                                                 std::pair<int, int>& offset) {
    cv::Mat rgb_img;
    
    // 转换为RGB（对应Python中的cvtColor）
    if (image.channels() == 1) {
        cv::cvtColor(image, rgb_img, cv::COLOR_GRAY2RGB);
    } else if (image.channels() == 3) {
        cv::cvtColor(image, rgb_img, cv::COLOR_BGR2RGB);
    } else {
        rgb_img = image.clone();
    }
    
    int img_h = rgb_img.rows;
    int img_w = rgb_img.cols;
    
    cv::Mat resized;
    
    if (letterbox_image_) {
        // Letterbox resize（对应Python中的resize_image with letterbox_image=True）
        scale = std::min(static_cast<float>(input_w_) / img_w, 
                        static_cast<float>(input_h_) / img_h);
        int nw = static_cast<int>(img_w * scale);
        int nh = static_cast<int>(img_h * scale);
        
        cv::resize(rgb_img, resized, cv::Size(nw, nh), 0, 0, cv::INTER_CUBIC);
        
        // 创建灰色背景
        letterboxed_img = cv::Mat::zeros(input_h_, input_w_, CV_8UC3);
        letterboxed_img.setTo(cv::Scalar(128, 128, 128));
        
        // 计算偏移量
        offset.first = (input_w_ - nw) / 2;
        offset.second = (input_h_ - nh) / 2;
        
        // 将resized图像粘贴到中心
        resized.copyTo(letterboxed_img(cv::Rect(offset.first, offset.second, nw, nh)));
    } else {
        // 直接resize
        cv::resize(rgb_img, letterboxed_img, cv::Size(input_w_, input_h_), 0, 0, cv::INTER_CUBIC);
        scale = 1.0f;
        offset = {0, 0};
    }
    
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
    
    // 转换为NCHW格式 [1, 3, H, W]
    std::vector<cv::Mat> channels;
    cv::split(normalized, channels);
    
    std::vector<float> input_data;
    input_data.reserve(1 * 3 * input_h_ * input_w_);
    
    for (int c = 0; c < 3; c++) {
        for (int h = 0; h < input_h_; h++) {
            for (int w = 0; w < input_w_; w++) {
                input_data.push_back(channels[c].at<float>(h, w));
            }
        }
    }
    
    return input_data;
}

cv::Mat YOLO_ONNX::decode_outputs(const std::vector<Ort::Value>& outputs) {
    // 对应Python中的decode_outputs函数
    std::vector<cv::Mat> decoded_outputs;
    
    // 获取每个输出特征图的尺寸
    std::vector<std::pair<int, int>> hw_list;
    std::vector<int> strides;
    
    for (const auto& output : outputs) {
        auto shape = output.GetTensorTypeAndShapeInfo().GetShape();
        int h = shape[2];
        int w = shape[3];
        hw_list.push_back({h, w});
        strides.push_back(input_h_ / h);
    }
    
    // 将输出展平并连接
    std::vector<float> all_outputs;
    int total_anchors = 0;
    
    for (size_t i = 0; i < outputs.size(); i++) {
        float* data = const_cast<float*>(outputs[i].GetTensorData<float>());
        auto shape = outputs[i].GetTensorTypeAndShapeInfo().GetShape();
        int batch = shape[0];
        int channels = shape[1];
        int h = shape[2];
        int w = shape[3];
        
        int num_anchors = h * w;
        total_anchors += num_anchors;
        
        // 展平: [batch, channels, h, w] -> [batch, h*w, channels]
        for (int b = 0; b < batch; b++) {
            for (int y = 0; y < h; y++) {
                for (int x = 0; x < w; x++) {
                    for (int c = 0; c < channels; c++) {
                        int idx = b * channels * h * w + c * h * w + y * w + x;
                        all_outputs.push_back(data[idx]);
                    }
                }
            }
        }
    }
    
    // 创建输出矩阵 [batch, total_anchors, channels]
    int channels = 5 + num_classes_;  // x, y, w, h, obj_conf, cls_conf...
    cv::Mat predictions(1, total_anchors, CV_32FC(channels));
    float* pred_data = reinterpret_cast<float*>(predictions.data);
    
    memcpy(pred_data, all_outputs.data(), total_anchors * channels * sizeof(float));
    
    // 对类别置信度应用sigmoid
    for (int i = 0; i < total_anchors; i++) {
        float* pred = pred_data + i * channels;
        // 对obj_conf和cls_conf应用sigmoid
        pred[4] = 1.0f / (1.0f + expf(-pred[4]));  // obj_conf
        for (int j = 5; j < channels; j++) {
            pred[j] = 1.0f / (1.0f + expf(-pred[j]));  // cls_conf
        }
    }
    
    // 解码坐标
    int anchor_idx = 0;
    for (size_t i = 0; i < outputs.size(); i++) {
        int h = hw_list[i].first;
        int w = hw_list[i].second;
        int stride = strides[i];
        
        // 生成网格
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                float* pred = pred_data + anchor_idx * channels;
                
                // 解码中心点坐标
                pred[0] = (pred[0] + x) * stride;  // x
                pred[1] = (pred[1] + y) * stride;  // y
                
                // 解码宽高
                pred[2] = expf(pred[2]) * stride;  // w
                pred[3] = expf(pred[3]) * stride;  // h
                
                // 归一化到[0,1]
                pred[0] /= input_w_;
                pred[2] /= input_w_;
                pred[1] /= input_h_;
                pred[3] /= input_h_;
                
                anchor_idx++;
            }
        }
    }
    
    return predictions;
}

std::vector<Detection> YOLO_ONNX::non_max_suppression(const cv::Mat& predictions,
                                                       const cv::Size& image_shape,
                                                       float scale,
                                                       const std::pair<int, int>& offset) {
    // 对应Python中的non_max_suppression函数
    std::vector<Detection> detections;
    
    int num_anchors = predictions.cols;
    int channels = predictions.channels();
    const float* pred_data = predictions.ptr<float>();
    
    // 转换为左上右下格式
    std::vector<std::vector<float>> boxes;
    std::vector<float> scores;
    std::vector<int> classes;
    
    for (int i = 0; i < num_anchors; i++) {
        const float* pred = pred_data + i * channels;
        
        // 获取类别置信度
        float obj_conf = pred[4];
        float max_cls_conf = 0.0f;
        int max_cls_idx = 0;
        
        for (int j = 0; j < num_classes_; j++) {
            float cls_conf = pred[5 + j];
            if (cls_conf > max_cls_conf) {
                max_cls_conf = cls_conf;
                max_cls_idx = j;
            }
        }
        
        float conf = obj_conf * max_cls_conf;
        
        // 置信度过滤
        if (conf < confidence_) {
            continue;
        }
        
        // 转换为左上右下格式
        float cx = pred[0];
        float cy = pred[1];
        float w = pred[2];
        float h = pred[3];
        
        float x1 = cx - w / 2.0f;
        float y1 = cy - h / 2.0f;
        float x2 = cx + w / 2.0f;
        float y2 = cy + h / 2.0f;
        
        boxes.push_back({x1, y1, x2, y2});
        scores.push_back(conf);
        classes.push_back(max_cls_idx);
    }
    
    if (boxes.empty()) {
        return detections;
    }
    
    // 按类别分组进行NMS
    std::vector<bool> suppressed(boxes.size(), false);
    
    // 按置信度排序
    std::vector<int> indices(boxes.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&scores](int a, int b) {
        return scores[a] > scores[b];
    });
    
    // NMS
    for (size_t i = 0; i < indices.size(); i++) {
        if (suppressed[indices[i]]) continue;
        
        int idx_i = indices[i];
        detections.push_back(Detection{});
        Detection& det = detections.back();
        
        // 坐标转换（对应Python中的yolo_correct_boxes）
        float x1 = boxes[idx_i][0];
        float y1 = boxes[idx_i][1];
        float x2 = boxes[idx_i][2];
        float y2 = boxes[idx_i][3];
        
        if (letterbox_image_) {
            // 反向letterbox变换
            float new_w = image_shape.width * scale;
            float new_h = image_shape.height * scale;
            float offset_x = static_cast<float>(offset.first) / input_w_;
            float offset_y = static_cast<float>(offset.second) / input_h_;
            float scale_x = static_cast<float>(input_w_) / new_w;
            float scale_y = static_cast<float>(input_h_) / new_h;
            
            y1 = (y1 - offset_y) * scale_y;
            x1 = (x1 - offset_x) * scale_x;
            y2 = (y2 - offset_y) * scale_y;
            x2 = (x2 - offset_x) * scale_x;
        }
        
        // 转换到原图坐标
        x1 *= image_shape.width;
        y1 *= image_shape.height;
        x2 *= image_shape.width;
        y2 *= image_shape.height;
        
        // 裁剪到图像边界
        det.x1 = std::max(0, static_cast<int>(std::floor(x1)));
        det.y1 = std::max(0, static_cast<int>(std::floor(y1)));
        det.x2 = std::min(image_shape.width - 1, static_cast<int>(std::floor(x2)));
        det.y2 = std::min(image_shape.height - 1, static_cast<int>(std::floor(y2)));
        
        det.class_name = class_names_[classes[idx_i]];
        det.score = scores[idx_i];
        
        // 抑制重叠框
        for (size_t j = i + 1; j < indices.size(); j++) {
            if (suppressed[indices[j]]) continue;
            if (classes[indices[i]] != classes[indices[j]]) continue;
            
            int idx_j = indices[j];
            cv::Rect box1(det.x1, det.y1, det.x2 - det.x1, det.y2 - det.y1);
            cv::Rect box2(static_cast<int>(boxes[idx_j][0] * image_shape.width),
                         static_cast<int>(boxes[idx_j][1] * image_shape.height),
                         static_cast<int>((boxes[idx_j][2] - boxes[idx_j][0]) * image_shape.width),
                         static_cast<int>((boxes[idx_j][3] - boxes[idx_j][1]) * image_shape.height));
            
            float iou = calculate_iou(box1, box2);
            if (iou > nms_iou_) {
                suppressed[indices[j]] = true;
            }
        }
    }
    
    return detections;
}

float YOLO_ONNX::calculate_iou(const cv::Rect& box1, const cv::Rect& box2) {
    int x1 = std::max(box1.x, box2.x);
    int y1 = std::max(box1.y, box2.y);
    int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
    int y2 = std::min(box1.y + box1.height, box2.y + box2.height);
    
    if (x2 < x1 || y2 < y1) {
        return 0.0f;
    }
    
    int intersection = (x2 - x1) * (y2 - y1);
    int area1 = box1.width * box1.height;
    int area2 = box2.width * box2.height;
    int union_area = area1 + area2 - intersection;
    
    return union_area > 0 ? static_cast<float>(intersection) / union_area : 0.0f;
}

std::vector<Detection> YOLO_ONNX::postprocess(const std::vector<Ort::Value>& outputs,
                                                const cv::Size& image_shape,
                                                float scale,
                                                const std::pair<int, int>& offset) {
    // 根据模型类型选择不同的后处理逻辑
    if (model_type_ == YOLOType::YOLOX) {
        return postprocess_yolox(outputs, image_shape, scale, offset);
    } else {
        return postprocess_yolov11(outputs, image_shape, scale, offset);
    }
}

std::vector<Detection> YOLO_ONNX::postprocess_yolox(const std::vector<Ort::Value>& outputs,
                                                     const cv::Size& image_shape,
                                                     float scale,
                                                     const std::pair<int, int>& offset) {
    // 解码输出
    cv::Mat predictions = decode_outputs(outputs);
    
    // 非极大抑制
    return non_max_suppression(predictions, image_shape, scale, offset);
}

std::vector<Detection> YOLO_ONNX::postprocess_yolov11(const std::vector<Ort::Value>& outputs,
                                                        const cv::Size& image_shape,
                                                        float scale,
                                                        const std::pair<int, int>& offset) {
    // YOLOv11输出格式: [batch, num_boxes, 6]
    // 6个值: [x1, y1, x2, y2, confidence, class_id]
    // 已经包含NMS，不需要额外处理
    
    std::vector<Detection> detections;
    
    if (outputs.empty()) {
        return detections;
    }
    
    const auto& output = outputs[0];
    auto shape = output.GetTensorTypeAndShapeInfo().GetShape();
    const float* data = output.GetTensorData<float>();
    
    int num_boxes = shape[1];
    int values_per_box = shape[2];  // 应该是6
    
    for (int i = 0; i < num_boxes; i++) {
        const float* box = data + i * values_per_box;
        
        float x1 = box[0];
        float y1 = box[1];
        float x2 = box[2];
        float y2 = box[3];
        float confidence = box[4];
        int class_id = static_cast<int>(box[5]);
        
        // 置信度过滤
        if (confidence < confidence_) {
            continue;
        }
        
        // 反向letterbox转换
        if (letterbox_image_) {
            x1 = (x1 - offset.first) / scale;
            y1 = (y1 - offset.second) / scale;
            x2 = (x2 - offset.first) / scale;
            y2 = (y2 - offset.second) / scale;
        } else {
            // 直接缩放到原图
            x1 = x1 * image_shape.width / input_w_;
            y1 = y1 * image_shape.height / input_h_;
            x2 = x2 * image_shape.width / input_w_;
            y2 = y2 * image_shape.height / input_h_;
        }
        
        // 裁剪到图像边界
        Detection det;
        det.x1 = std::max(0, static_cast<int>(std::floor(x1)));
        det.y1 = std::max(0, static_cast<int>(std::floor(y1)));
        det.x2 = std::min(image_shape.width - 1, static_cast<int>(std::floor(x2)));
        det.y2 = std::min(image_shape.height - 1, static_cast<int>(std::floor(y2)));
        
        // 检查类别ID是否有效
        if (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
            det.class_name = class_names_[class_id];
        } else {
            det.class_name = "Unknown";
        }
        det.score = confidence;
        
        detections.push_back(det);
    }
    
    return detections;
}

std::vector<Detection> YOLO_ONNX::detect_image(const cv::Mat& image) {
    if (image.empty()) {
        return {};
    }
    
    // 预处理
    cv::Mat letterboxed_img;
    float scale;
    std::pair<int, int> offset;
    std::vector<float> input_data = preprocess_image(image, letterboxed_img, scale, offset);
    
    // 创建输入张量
    std::vector<int64_t> input_shape = {1, 3, input_h_, input_w_};
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator, OrtMemTypeDefault);
    
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        input_data.data(),
        input_data.size(),
        input_shape.data(),
        input_shape.size());

    // 推理
    std::vector<const char*> input_names_cstr;
    std::vector<const char*> output_names_cstr;
    input_names_cstr.reserve(input_names_.size());
    output_names_cstr.reserve(output_names_.size());
     
    for (const auto& name : input_names_) {
        // #region agent log
        // {
        //     std::ofstream log_file("/home/tl/RV/.cursor/debug.log", std::ios::app);
        //     if (log_file.is_open()) {
        //         log_file << "{\"sessionId\":\"debug-session\",\"runId\":\"post-fix\",\"hypothesisId\":\"C\",\"location\":\"yolo_onnx.cpp:520\",\"message\":\"Converting input name for inference\",\"data\":{\"name\":\"" << name << "\",\"name_length\":" << name.length() << "},\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << "}\n";
        //         log_file.close();
        //     }
        // }
        // #endregion
        input_names_cstr.push_back(name.c_str());
    }
    for (const auto& name : output_names_) {
        output_names_cstr.push_back(name.c_str());
    }

    // #region agent log
    // {
    //     std::ofstream log_file("/home/tl/RV/.cursor/debug.log", std::ios::app);
    //     if (log_file.is_open()) {
    //         log_file << "{\"sessionId\":\"debug-session\",\"runId\":\"post-fix\",\"hypothesisId\":\"C\",\"location\":\"yolo_onnx.cpp:535\",\"message\":\"Running inference\",\"data\":{\"input_count\":" << input_names_cstr.size() << ",\"output_count\":" << output_names_cstr.size() << "},\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << "}\n";
    //         log_file.close();
    //     }
    // }
    // #endregion

    std::vector<Ort::Value> outputs = session_->Run(
        Ort::RunOptions{nullptr},
        input_names_cstr.data(),  // 使用转换后的数组
        &input_tensor,
        1,
        output_names_cstr.data(),  // 使用转换后的数组
        output_names_cstr.size());
    
    // 后处理
    cv::Size image_shape(image.cols, image.rows);
    return postprocess(outputs, image_shape, scale, offset);
}