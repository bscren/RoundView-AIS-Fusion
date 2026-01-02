// include/marnav_vis_cpp/yolo_onnx.h
#ifndef YOLO_ONNX_H
#define YOLO_ONNX_H

#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>

struct Detection {
    int x1, y1, x2, y2;           // 检测框坐标
    std::string class_name;        // 类别名称
    float score;                   // 置信度分数
};

// YOLO模型类型
enum class YOLOType {
    YOLOX,      // YOLOX格式（多输出，需要decode）
    YOLOv11     // YOLOv11格式（单输出，端到端）
};

class YOLO_ONNX {
public:
    YOLO_ONNX(const std::string& model_path, 
               const std::string& classes_path,
               int input_h = 640, 
               int input_w = 640,
               float confidence = 0.1f,
               float nms_iou = 0.5f,
               bool letterbox_image = true,
               bool use_cuda = true,
               YOLOType model_type = YOLOType::YOLOX);
    
    ~YOLO_ONNX();
    
    // 检测图像，返回检测结果（与Python版本功能相同）
    std::vector<Detection> detect_image(const cv::Mat& image);
    
private:
    // 预处理函数
    std::vector<float> preprocess_image(const cv::Mat& image, 
                                         cv::Mat& letterboxed_img,
                                         float& scale, 
                                         std::pair<int, int>& offset);
    
    // 后处理函数
    std::vector<Detection> postprocess(const std::vector<Ort::Value>& outputs,
                                       const cv::Size& image_shape,
                                       float scale,
                                       const std::pair<int, int>& offset);
    
    // YOLOX后处理
    std::vector<Detection> postprocess_yolox(const std::vector<Ort::Value>& outputs,
                                              const cv::Size& image_shape,
                                              float scale,
                                              const std::pair<int, int>& offset);
    
    // YOLOv11后处理（端到端，已包含NMS）
    std::vector<Detection> postprocess_yolov11(const std::vector<Ort::Value>& outputs,
                                                const cv::Size& image_shape,
                                                float scale,
                                                const std::pair<int, int>& offset);
    
    // 解码输出（对应Python中的decode_outputs）
    cv::Mat decode_outputs(const std::vector<Ort::Value>& outputs);
    
    // 非极大抑制（对应Python中的non_max_suppression）
    std::vector<Detection> non_max_suppression(const cv::Mat& predictions,
                                                const cv::Size& image_shape,
                                                float scale,
                                                const std::pair<int, int>& offset);
    
    // 读取类别文件
    void load_classes(const std::string& classes_path);
    
    // 计算IoU
    float calculate_iou(const cv::Rect& box1, const cv::Rect& box2);
    
    // ONNX Runtime相关
    Ort::Env env_;
    Ort::Session* session_;
    Ort::SessionOptions session_options_;
    Ort::AllocatorWithDefaultOptions allocator_;
    
    // 模型参数
    int input_h_, input_w_;
    float confidence_, nms_iou_;
    bool letterbox_image_, use_cuda_;
    YOLOType model_type_;
    
    // 类别信息
    std::vector<std::string> class_names_;
    int num_classes_;
    
    // 输入输出名称
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<std::vector<int64_t>> input_shapes_;
    std::vector<std::vector<int64_t>> output_shapes_;
};

#endif // YOLO_ONNX_H