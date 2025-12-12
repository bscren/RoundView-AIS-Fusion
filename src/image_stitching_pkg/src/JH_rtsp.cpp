
// 版本6的改进：支持从YAML配置文件读取RTSP地址和话题名称,由串行连接RTSP流修改为并行连接RTSP流
// 实现了帧率同步，但是如果目标帧率和输入帧率不等，那么帧同步效果随缘————要么整体摄像头同步，要么部分摄像头同步
// 
// 运行方式：
// 1. 通过命令行参数指定配置文件路径：
//    ros2 run image_stitching_pkg JH_rtsp <配置文件路径>
//    例如：ros2 run image_stitching_pkg JH_rtsp /path/to/rtsp_camera_config.yaml
// 
// 2. 通过ROS参数指定配置文件路径：
//    ros2 run image_stitching_pkg JH_rtsp --ros-args -p config_file:=/path/to/rtsp_camera_config.yaml
// 
// 3. 如果不指定配置文件，将使用默认路径：
//    <package_share_directory>/config/rtsp_camera_config.yaml
// 
// 配置文件格式：请参考 config/rtsp_camera_config.yaml
// 配置文件支持动态增减相机数量，只需在YAML文件中添加或删除相机配置项即可
// 
// ROS bag 录制：
// 运行前先在终端移动至src/image_stitching_pkg/ros2bag目录下，
// 指令：ros2 bag record -o multi_camera_record <话题名1> <话题名2> <话题名3>
// 结束录制：Ctrl+C
// 录制完成后，使用指令：ros2 bag play -l multi_camera_record
// 录制的ROS bag文件将包含所有摄像头的图像数据，供后续处理和分析
// 
// 注意：此代码需要OpenCV和ROS 2环境支持，并且需要安装cv_bridge和image_transport等ROS 2包
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <vector>
#include <memory>
#include <chrono>
#include <fstream>

// ROS 2 相关头文件
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

// YAML解析库
#include <yaml-cpp/yaml.h>

// ROS 2包路径查找
#include <ament_index_cpp/get_package_share_directory.hpp>

// 包含时间戳的帧结构体
struct TimestampedFrame {
    cv::Mat frame;
    std::chrono::high_resolution_clock::time_point timestamp;
};

// 帧队列类，用于多线程处理
class FrameQueue {
private:
    std::queue<TimestampedFrame> queue;
    std::mutex mtx;
    std::atomic<size_t> max_size;
    bool drop_frames;
public:
    FrameQueue(size_t size, bool drop = true) : max_size(size), drop_frames(drop) {}
    void push(const cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(mtx);
        if (drop_frames && queue.size() >= max_size) {
            queue.pop();
        }
        TimestampedFrame tf;
        tf.frame = frame.clone();
        tf.timestamp = std::chrono::high_resolution_clock::now();
        queue.push(tf);
    }
    bool pop(TimestampedFrame& tf) {
        std::lock_guard<std::mutex> lock(mtx);
        if (queue.empty()) {
            return false;
        }
        tf = queue.front();
        queue.pop();
        return true;
    }
    void clear() {
        std::lock_guard<std::mutex> lock(mtx);
        while (!queue.empty()) {
            queue.pop();
        }
    }
    int size() {
        std::lock_guard<std::mutex> lock(mtx);
        return queue.size();
    }
};

// 帧同步器类
class FrameSynchronizer {
private:
    std::vector<std::deque<TimestampedFrame>> camera_buffers; // 每个相机的帧缓冲区
    size_t num_cameras;
    std::chrono::milliseconds sync_window; // 同步时间窗口大小
    
public:
    FrameSynchronizer(size_t num_cameras, int window_ms = 50)
        : num_cameras(num_cameras), sync_window(window_ms) {
        camera_buffers.resize(num_cameras);
    }
    
    // 添加一帧到对应相机的缓冲区
    void add_frame(size_t camera_idx, const TimestampedFrame& frame) {
        std::lock_guard<std::mutex> lock(mtx);
        camera_buffers[camera_idx].push_back(frame);
        
        // 保持缓冲区大小合理
        if (camera_buffers[camera_idx].size() > 10) {
            camera_buffers[camera_idx].pop_front();
        }
    }
    
    // 尝试获取所有相机的同步帧
    bool get_synchronized_frames(std::vector<TimestampedFrame>& synchronized_frames) {
        std::lock_guard<std::mutex> lock(mtx);
        synchronized_frames.resize(num_cameras);
        
        // 检查所有相机是否有足够的帧
        for (size_t i = 0; i < num_cameras; ++i) {
            if (camera_buffers[i].size() < 2) {
                return false; // 帧不足，无法同步
            }
        }
        
        // 1. 确定参考时间点（选择中间相机的最新帧时间）
        auto reference_time = camera_buffers[num_cameras/2].back().timestamp; //num_cameras/2如果为小数，就取整数位
        
        // 2. 为每个相机找到最接近参考时间的帧
        for (size_t i = 0; i < num_cameras; ++i) {
            auto& buffer = camera_buffers[i];
            
            // 找到第一个时间戳大于参考时间的帧
            auto it = std::upper_bound(
                buffer.begin(), buffer.end(), 
                reference_time,
                [](const auto& time, const auto& frame) {
                    return time < frame.timestamp;
                }
            );
            
            // 如果找不到这样的帧，或者距离太远，使用最后一帧
            if (it == buffer.end() || 
                std::chrono::duration_cast<std::chrono::milliseconds>(it->timestamp - reference_time).count() > sync_window.count()) {
                synchronized_frames[i] = buffer.back();
            } else if (it == buffer.begin()) {
                // 如果是第一个帧，使用它
                synchronized_frames[i] = *it;
            } else {
                // 在两个帧之间，进行插值
                auto prev = std::prev(it);
                double alpha = std::chrono::duration_cast<std::chrono::duration<double>>(reference_time - prev->timestamp) /
                               std::chrono::duration_cast<std::chrono::duration<double>>(it->timestamp - prev->timestamp);
                
                synchronized_frames[i].timestamp = reference_time;
                cv::addWeighted(prev->frame, 1.0 - alpha, it->frame, alpha, 0.0, synchronized_frames[i].frame);
            }
        }
        
        return true;
    }
    
private:
    std::mutex mtx;
};

// 相机配置结构体
struct CameraConfig {
    int camera_id;
    std::string rtsp_url;
    std::string topic_name;
    std::string description;
};

// 从YAML文件读取相机配置和帧率参数
bool loadCameraConfigFromYAML(const std::string& config_file_path, 
                               std::vector<CameraConfig>& camera_configs,
                               int& input_frame_rate,
                               int& target_frame_rate,
                               rclcpp::Logger logger) {
    try {
        YAML::Node config = YAML::LoadFile(config_file_path);
        
        // 读取帧率参数（如果未定义则使用默认值）
        if (config["input_frame_rate"]) {
            input_frame_rate = config["input_frame_rate"].as<int>();
        } else {
            input_frame_rate = 25; // 默认值
            RCLCPP_WARN(logger, "YAML配置文件中未找到'input_frame_rate'，使用默认值: %d", input_frame_rate);
        }
        
        if (config["target_frame_rate"]) {
            target_frame_rate = config["target_frame_rate"].as<int>();
        } else {
            target_frame_rate = 25; // 默认值
            RCLCPP_WARN(logger, "YAML配置文件中未找到'target_frame_rate'，使用默认值: %d", target_frame_rate);
        }
        
        RCLCPP_INFO(logger, "帧率配置: 输入帧率=%d FPS, 目标帧率=%d FPS", input_frame_rate, target_frame_rate);
        
        if (!config["cameras"]) {
            RCLCPP_ERROR(logger, "YAML配置文件中未找到'cameras'节点");
            return false;
        }
        
        const YAML::Node& cameras = config["cameras"];
        if (!cameras.IsSequence()) {
            RCLCPP_ERROR(logger, "YAML配置文件中的'cameras'节点必须是列表格式");
            return false;
        }
        
        camera_configs.clear();
        for (size_t i = 0; i < cameras.size(); ++i) {
            CameraConfig cam_config;
            
            if (!cameras[i]["camera_id"] || !cameras[i]["rtsp_url"] || !cameras[i]["topic_name"]) {
                RCLCPP_WARN(logger, "跳过相机配置项 %zu：缺少必需字段(camera_id, rtsp_url, topic_name)", i);
                continue;
            }
            
            cam_config.camera_id = cameras[i]["camera_id"].as<int>();
            cam_config.rtsp_url = cameras[i]["rtsp_url"].as<std::string>();
            cam_config.topic_name = cameras[i]["topic_name"].as<std::string>();
            
            if (cameras[i]["description"]) {
                cam_config.description = cameras[i]["description"].as<std::string>();
            } else {
                cam_config.description = "相机" + std::to_string(cam_config.camera_id);
            }
            
            camera_configs.push_back(cam_config);
            RCLCPP_INFO(logger, "加载相机配置: ID=%d, URL=%s, Topic=%s", 
                       cam_config.camera_id, cam_config.rtsp_url.c_str(), cam_config.topic_name.c_str());
        }
        
        if (camera_configs.empty()) {
            RCLCPP_ERROR(logger, "未找到有效的相机配置");
            return false;
        }
        
        RCLCPP_INFO(logger, "成功加载 %zu 个相机配置", camera_configs.size());
        return true;
        
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(logger, "解析YAML配置文件失败: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "读取配置文件时发生错误: %s", e.what());
        return false;
    }
}

int main(int argc, char * argv[])
{   
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rtsp_video_publisher");
    
    // 声明并获取配置文件路径参数（支持通过ROS参数或命令行指定）
    node->declare_parameter<std::string>("config_file", "");
    std::string config_file_path = node->get_parameter("config_file").as_string();
    
    // 如果ROS参数未设置，尝试从命令行参数获取
    if (config_file_path.empty() && argc >= 2) {
        config_file_path = argv[1];
    }
    
    // 如果仍未指定配置文件，使用默认路径
    if (config_file_path.empty()) {
        // 默认配置文件路径（相对于包目录）
        std::string package_path = ament_index_cpp::get_package_share_directory("image_stitching_pkg");
        config_file_path = package_path + "/config/JH_rtsp_config.yaml";
        RCLCPP_INFO(node->get_logger(), "未指定配置文件，使用默认路径: %s", config_file_path.c_str());
    }
    
    // 检查配置文件是否存在
    std::ifstream file_check(config_file_path);
    if (!file_check.good()) {
        RCLCPP_ERROR(node->get_logger(), "配置文件不存在: %s", config_file_path.c_str());
        RCLCPP_ERROR(node->get_logger(), "使用方法: ros2 run image_stitching_pkg JH_rtsp <配置文件路径>");
        RCLCPP_ERROR(node->get_logger(), "或者通过ROS参数: --ros-args -p config_file:=<配置文件路径>");
        rclcpp::shutdown();
        return -1;
    }
    file_check.close();
    
    // 从YAML文件加载相机配置和帧率参数
    std::vector<CameraConfig> camera_configs;
    int input_frame_rate = 25;  // 默认值
    int target_frame_rate = 25; // 默认值
    if (!loadCameraConfigFromYAML(config_file_path, camera_configs, input_frame_rate, target_frame_rate, node->get_logger())) {
        RCLCPP_ERROR(node->get_logger(), "加载相机配置失败，程序退出");
        rclcpp::shutdown();
        return -1;
    }
    
    // 从配置中提取RTSP URL和话题名称
    std::vector<std::string> rtsp_urls;
    std::vector<std::string> topic_names;
    for (const auto& config : camera_configs) {
        rtsp_urls.push_back(config.rtsp_url);
        topic_names.push_back(config.topic_name);
        RCLCPP_INFO(node->get_logger(), "相机%d: RTSP=%s, Topic=%s", 
                   config.camera_id, config.rtsp_url.c_str(), config.topic_name.c_str());
    }
    
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers;

    // 为每个摄像头创建发布者（配置 BestEffort QoS 策略）
    // 图像传输使用 BestEffort 可靠性策略，适合高频率、大数据量传输
    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile);
    
    for (size_t i = 0; i < topic_names.size(); ++i) {
        auto pub = node->create_publisher<sensor_msgs::msg::Image>(
            topic_names[i], 
            image_qos
        );
        publishers.push_back(pub);
        RCLCPP_INFO(node->get_logger(), "创建发布者: %s", topic_names[i].c_str());
    }

    // 核心修改：用unique_ptr包装atomic<bool>，避免拷贝
    std::vector<cv::VideoCapture> caps(rtsp_urls.size());
    std::vector<std::thread> read_threads;
    // 用智能指针存储atomic<bool>，避免vector对atomic的直接拷贝
    std::vector<std::unique_ptr<std::atomic<bool>>> thread_runnings;

    // 初始化线程运行标志（关键：逐个构造，不触发拷贝）
    for (size_t i = 0; i < rtsp_urls.size(); ++i) {
        thread_runnings.emplace_back(std::make_unique<std::atomic<bool>>(true));
    }

    // 新增：创建帧同步器
    FrameSynchronizer frame_synchronizer(rtsp_urls.size(), 50); // 50ms同步窗口

    // ========== 优化：并行连接所有RTSP流 ==========
    RCLCPP_INFO(node->get_logger(), "开始并行连接 %zu 个RTSP流...", rtsp_urls.size());
    
    // 用于存储连接结果的标志
    std::vector<std::atomic<bool>> connection_success(rtsp_urls.size());
    std::vector<std::thread> connection_threads;
    
    // 并行连接所有RTSP流
    for (size_t i = 0; i < rtsp_urls.size(); ++i) {
        connection_success[i].store(false);
        connection_threads.emplace_back([&, i]() {
            RCLCPP_INFO(node->get_logger(), "⏳ 开始连接相机 %zu: %s", i, rtsp_urls[i].c_str());
            
            // 打开RTSP流
            bool opened = caps[i].open(rtsp_urls[i]);
            if (!opened) {
                RCLCPP_ERROR(node->get_logger(), "❌ 无法打开RTSP流 %s，请检查URL和网络连接！", rtsp_urls[i].c_str());
                return;
            }
            
            // 设置视频捕获参数
            caps[i].set(cv::CAP_PROP_BUFFERSIZE, 1);
            caps[i].set(cv::CAP_PROP_FPS, input_frame_rate);
            caps[i].set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            caps[i].set(cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY);
            caps[i].set(900, 0);

            // 初始化捕获器
            cv::Mat dummy;
            for (int j = 0; j < 3; j++) {
                caps[i] >> dummy;
            }
            
            int frame_width = caps[i].get(cv::CAP_PROP_FRAME_WIDTH);
            int frame_height = caps[i].get(cv::CAP_PROP_FRAME_HEIGHT);
            RCLCPP_INFO(node->get_logger(), "✅ 相机 %zu 连接成功: %s (分辨率: %dx%d)", 
                       i, rtsp_urls[i].c_str(), frame_width, frame_height);
            
            connection_success[i].store(true);
        });
    }
    
    // 等待所有连接线程完成（最多等待30秒）
    auto connection_start_time = std::chrono::high_resolution_clock::now();
    for (auto& thread : connection_threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    auto connection_end_time = std::chrono::high_resolution_clock::now();
    auto connection_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        connection_end_time - connection_start_time).count();
    
    // 检查所有连接是否成功
    bool all_connected = true;
    for (size_t i = 0; i < rtsp_urls.size(); ++i) {
        if (!connection_success[i].load()) {
            RCLCPP_ERROR(node->get_logger(), "❌ 相机 %zu 连接失败: %s", i, rtsp_urls[i].c_str());
            all_connected = false;
        }
    }
    
    if (!all_connected) {
        RCLCPP_ERROR(node->get_logger(), "部分相机连接失败，程序退出");
        rclcpp::shutdown();
        return -1;
    }
    
    RCLCPP_INFO(node->get_logger(), "🎉 所有 %zu 个RTSP流连接成功！总耗时: %ld ms", 
               rtsp_urls.size(), connection_duration);
    // ========== 并行连接结束 ==========

    // 启动帧读取线程（为每个相机创建独立线程）
    for (size_t i = 0; i < rtsp_urls.size(); ++i) {
        read_threads.emplace_back([&, i]() 
        {
            cv::Mat frame;
            // 用->load()访问atomic的值
            while (thread_runnings[i]->load()) 
            {
                if (caps[i].read(frame)) {
                    // 新增：将帧添加到同步器
                    TimestampedFrame tf;
                    tf.frame = frame.clone();
                    tf.timestamp = std::chrono::high_resolution_clock::now();
                    frame_synchronizer.add_frame(i, tf);
                } 
                else {
                    RCLCPP_ERROR(node->get_logger(), "读取帧失败，尝试重新连接到 %s...", rtsp_urls[i].c_str());
                    caps[i].release();
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    caps[i].open(rtsp_urls[i], cv::CAP_GSTREAMER);
                }
            }
        }); //线程执行体，每个线程执行的代码是一个 lambda 表达式
    }

    // 主线程发布最新帧
    std::vector<TimestampedFrame> display_frames(rtsp_urls.size());
    std::vector<int> frames_displayed(rtsp_urls.size(), 0);
    std::vector<std::chrono::high_resolution_clock::time_point> last_times(rtsp_urls.size());
    for (size_t i = 0; i < rtsp_urls.size(); ++i) {
        last_times[i] = std::chrono::high_resolution_clock::now();
    }

    rclcpp::Rate loop_rate(target_frame_rate); // 降低帧率以提高同步质量
    while (rclcpp::ok()) {
        std::vector<TimestampedFrame> synchronized_frames;
        
        // 尝试获取同步帧
        if (frame_synchronizer.get_synchronized_frames(synchronized_frames)) {
            // 发布同步帧
            for (size_t i = 0; i < rtsp_urls.size(); ++i) {
                // RCLCPP_INFO(node->get_logger(), 
                //     "发送相机[%zu]（URL: %s）的图片尺寸：宽度=%d 像素，高度=%d 像素",
                //     i,                      // 相机索引
                //     rtsp_urls[i].c_str(),   // 相机RTSP地址（便于区分不同相机）
                //     synchronized_frames[i].frame.cols,     // 图片宽度（OpenCV中cols表示列数，即宽度）
                //     synchronized_frames[i].frame.rows);    // 图片高度（OpenCV中rows表示行数，即高度）
                
                auto msg = cv_bridge::CvImage(
                    std_msgs::msg::Header(), 
                    "bgr8", 
                    synchronized_frames[i].frame
                ).toImageMsg();
                
                // 设置相同的时间戳，确保ROS认为这些帧是同步的
                msg->header.stamp = node->get_clock()->now();
                publishers[i]->publish(*msg);  // 使用 -> 访问智能指针，并解引用 msg
                
                // 更新帧率统计
                frames_displayed[i]++;
                auto current_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_times[i]).count();
                
                if (duration >= 1) {
                    // RCLCPP_INFO(node->get_logger(), "摄像头 %zu 当前帧率: %ld FPS", i, frames_displayed[i] / duration);
                    frames_displayed[i] = 0;
                    last_times[i] = current_time;
                }
            }
        }
        
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    // 停止所有线程（用->store(false)设置atomic的值）
    for (size_t i = 0; i < rtsp_urls.size(); ++i) {
        thread_runnings[i]->store(false);
        if (read_threads[i].joinable()) {
            read_threads[i].join();
        }
    }
    
    // 释放资源
    for (auto& cap : caps) {
        cap.release();
    }
    rclcpp::shutdown();
    return 0;
}