// src/ckpt_onnx.cpp
// 基于ONNX Runtime的DeepSORT特征提取模型推理
// 输入：图像文件
// 输出：[1, 512]格式的特征向量

#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <cmath>
#include <fstream>
#include <algorithm>

class CKPT_ONNX {
public:
    CKPT_ONNX(const std::string& model_path, bool use_cuda = true)
        : env_(ORT_LOGGING_LEVEL_WARNING, "CKPT"),
          session_(nullptr),
          use_cuda_(use_cuda),
          input_h_(128),
          input_w_(64) {
        
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
        #ifdef _WIN32
            std::wstring wmodel_path(model_path.begin(), model_path.end());
            session_ = new Ort::Session(env_, wmodel_path.c_str(), session_options_);
        #else
            session_ = new Ort::Session(env_, model_path.c_str(), session_options_);
        #endif
        
        // 获取输入输出信息
        size_t num_input_nodes = session_->GetInputCount();
        size_t num_output_nodes = session_->GetOutputCount();
        
        input_names_.clear();
        output_names_.clear();
        input_names_.reserve(num_input_nodes);
        output_names_.reserve(num_output_nodes);
        
        // 获取输入信息
        for (size_t i = 0; i < num_input_nodes; i++) {
            auto input_name = session_->GetInputNameAllocated(i, allocator_);
            input_names_.push_back(std::string(input_name.get()));
        }
        
        // 获取输出信息
        for (size_t i = 0; i < num_output_nodes; i++) {
            auto output_name = session_->GetOutputNameAllocated(i, allocator_);
            output_names_.push_back(std::string(output_name.get()));
        }
        
        std::cout << "CKPT ONNX模型加载成功: " << model_path << std::endl;
        std::cout << "输入节点数: " << input_names_.size() << std::endl;
        std::cout << "输出节点数: " << output_names_.size() << std::endl;
    }
    
    ~CKPT_ONNX() {
        if (session_) {
            delete session_;
        }
    }
    
    // 提取特征，返回[1, 512]格式的特征向量
    std::vector<float> extract_feature(const cv::Mat& image) {
        if (image.empty()) {
            return {};
        }
        
        // 预处理
        std::vector<float> input_data = preprocess_image(image);
        
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
        
        // 准备输入输出名称
        std::vector<const char*> input_names_cstr;
        std::vector<const char*> output_names_cstr;
        input_names_cstr.reserve(input_names_.size());
        output_names_cstr.reserve(output_names_.size());
        
        for (const auto& name : input_names_) {
            input_names_cstr.push_back(name.c_str());
        }
        for (const auto& name : output_names_) {
            output_names_cstr.push_back(name.c_str());
        }
        
        // 推理
        std::vector<Ort::Value> outputs = session_->Run(
            Ort::RunOptions{nullptr},
            input_names_cstr.data(),
            &input_tensor,
            1,
            output_names_cstr.data(),
            output_names_cstr.size());
        
        // 提取输出特征向量 [1, 512]
        if (outputs.empty()) {
            return {};
        }
        
        const float* output_data = outputs[0].GetTensorData<float>();
        auto output_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
        
        size_t feature_dim = output_shape[1];  // 512
        std::vector<float> features(output_data, output_data + feature_dim);
        
        return features;
    }
    
private:
    // 预处理图像
    std::vector<float> preprocess_image(const cv::Mat& image) {
        cv::Mat rgb_img;
        
        // 转换为RGB
        if (image.channels() == 1) {
            cv::cvtColor(image, rgb_img, cv::COLOR_GRAY2RGB);
        } else if (image.channels() == 3) {
            cv::cvtColor(image, rgb_img, cv::COLOR_BGR2RGB);
        } else {
            rgb_img = image.clone();
        }
        
        // 先归一化到[0,1]（与Python版本保持一致：先归一化再resize）
        cv::Mat normalized_input;
        rgb_img.convertTo(normalized_input, CV_32F, 1.0 / 255.0);
        
        // Resize到128x64 (高x宽) - 在归一化后进行，与Python版本一致
        cv::Mat resized;
        cv::resize(normalized_input, resized, cv::Size(input_w_, input_h_), 0, 0, cv::INTER_LINEAR);
        
        // ImageNet标准化: mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225]
        cv::Scalar mean(0.485, 0.456, 0.406);
        cv::Scalar std(0.229, 0.224, 0.225);
        
        std::vector<cv::Mat> channels;
        cv::split(resized, channels);  // 使用resized进行标准化
        for (int i = 0; i < 3; i++) {
            channels[i] = (channels[i] - mean[i]) / std[i];
        }
        cv::Mat normalized;
        cv::merge(channels, normalized);
        
        // 转换为NCHW格式 [1, 3, 128, 64]
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
    
    Ort::Env env_;
    Ort::Session* session_;
    Ort::SessionOptions session_options_;
    Ort::AllocatorWithDefaultOptions allocator_;
    
    bool use_cuda_;
    int input_h_, input_w_;
    
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
};

int main(int argc, char* argv[]) {
    // 默认路径
    std::string model_path = "src/marnav_vis_cpp/model/ckpt.onnx";
    std::string image_path = "src/marnav_vis_cpp/test/ckpt_demo.jpg";
    
    // 可以从命令行参数读取
    if (argc >= 2) {
        model_path = argv[1];
    }
    if (argc >= 3) {
        image_path = argv[2];
    }
    
    std::cout << "模型路径: " << model_path << std::endl;
    std::cout << "图像路径: " << image_path << std::endl;
    
    // 创建CKPT_ONNX实例
    CKPT_ONNX ckpt(model_path, true);  // 使用CUDA
    
    // 读取图像
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) {
        std::cerr << "无法读取图像: " << image_path << std::endl;
        return -1;
    }
    
    std::cout << "图像尺寸: " << image.cols << "x" << image.rows << std::endl;
    
    // 提取特征
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<float> features = ckpt.extract_feature(image);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "特征提取时间: " << duration.count() << "毫秒" << std::endl;
    std::cout << "特征向量维度: " << features.size() << std::endl;
    
    if (features.empty()) {
        std::cerr << "特征提取失败" << std::endl;
        return -1;
    }
    
    // 输出特征向量（前10个元素）
    std::cout << "特征向量前10个元素: [";
    for (size_t i = 0; i < std::min(size_t(10), features.size()); i++) {
        std::cout << features[i];
        if (i < std::min(size_t(9), features.size() - 1)) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
    
    // 计算L2范数（应该接近1.0，因为输出是L2归一化的）
    float norm = 0.0f;
    for (float f : features) {
        norm += f * f;
    }
    norm = std::sqrt(norm);
    std::cout << "特征向量L2范数: " << norm << " (应接近1.0)" << std::endl;
    
    // 输出完整的特征向量到文件（可选）
    std::string output_path = "src/marnav_vis_cpp/test/ckpt_feature.txt";
    std::ofstream out_file(output_path);
    if (out_file.is_open()) {
        out_file << "[";
        for (size_t i = 0; i < features.size(); i++) {
            out_file << features[i];
            if (i < features.size() - 1) {
                out_file << ", ";
            }
        }
        out_file << "]" << std::endl;
        out_file.close();
        std::cout << "特征向量已保存到: " << output_path << std::endl;
    }
    
    return 0;
}

