// src/yolo_example.cpp
#include "marnav_vis_cpp/yolo_onnx.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

int main() {
    // 初始化YOLO
    std::string model_path = "src/marnav_vis_cpp/model/Yolo11m.onnx";
    std::string classes_path = "src/marnav_vis_cpp/model/Yolo11m_ship_classes.txt";
    
    YOLO_ONNX yolo(model_path, classes_path, 
                    640, 640,         // input_shape
                    0.1f,             // confidence
                    0.5f,             // nms_iou
                    true,             // letterbox_image
                    true,             // use_cuda
                    YOLOType::YOLOv11); // model_type
    
    // 读取图像
    cv::Mat image = cv::imread("src/marnav_vis_cpp/test/test.jpg");
    if (image.empty()) {
        std::cerr << "无法读取图像" << std::endl;
        return -1;
    }
    
    // 检测
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<Detection> detections = yolo.detect_image(image);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "检测时间: " << duration.count() << "毫秒" << std::endl;

    // 绘制结果
    for (const auto& det : detections) {
        cv::rectangle(image, 
                     cv::Point(det.x1, det.y1), 
                     cv::Point(det.x2, det.y2),
                     cv::Scalar(0, 255, 0), 2);
        
        std::string label = det.class_name + ": " + std::to_string(det.score);
        cv::putText(image, label,
                   cv::Point(det.x1, det.y1 - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5,
                   cv::Scalar(0, 255, 0), 2);
        
        std::cout << "检测到: " << det.class_name 
                  << " 置信度: " << det.score
                  << " 坐标: [" << det.x1 << ", " << det.y1 
                  << ", " << det.x2 << ", " << det.y2 << "]" << std::endl;
    }
    
    // 保存结果
    cv::imwrite("src/marnav_vis_cpp/test/result.jpg", image);
    
    return 0;
}