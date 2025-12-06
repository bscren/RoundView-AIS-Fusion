// 版本5的改进，实现了帧率同步，但是如果目标帧率和输入帧率不等，那么帧同步效果随缘————要么整体摄像头同步，要么部分摄像头同步
// 运行方式：ros2 run image_stitching_pkg JH_rtsp x y z，其中x、y、z为摄像头IP尾号ID
// ROS bag 运行
// 运行前请先确保摄像头的IP地址和尾号ID正确，并且摄像头已连接到网络
// 运行前先在终端移动至src/image_stitching_pkg/ros2bag目录下，
// 指令：ros2 bag record -o multi_camera_record /rtsp_image_0 /rtsp_image_1 /rtsp_image_2
// 结束录制：Ctrl+C
// 录制完成后，使用指令：ros2 bag play -l multi_camera_record
// 录制的ROS bag文件将包含所有摄像头的图像数据，供后续处理和分析
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

// ROS 2 相关头文件
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

const int input_frame_rate = 20; // 相机设置的输入帧率
const int target_frame_rate = 15; // 目标帧率，测试结果为与相机设置的输入帧率一致表现最好
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
    FrameQueue(size_t size = target_frame_rate, bool drop = true) : max_size(size), drop_frames(drop) {}
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

int main(int argc, char * argv[])
{   
    // 如果参数为空
    if (argc < 2) {
        std::cerr << "请提供至少一个摄像头尾号ID作为参数！" << std::endl;
        return -1;
    }
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rtsp_video_publisher");
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers;

    // 解析命令行参数，生成RTSP URL列表
    std::vector<std::string> rtsp_urls;
    for (int i = 1; i < argc; ++i) {
        int camera_id = std::stoi(argv[i]);
        std::string url = "rtsp://admin:juhai00" + std::to_string(camera_id) + "@192.168.1." + std::to_string(000 + camera_id) + ":554/Streaming/Channels/2";
        // std::string url = "rtsp://admin:juhai003@192.168.1.003:554/Streaming/Channels/1"; // 替换为实际的RTSP URL
        // 主码流：
        // rtsp://admin:12345@192.168.1.64:554/Streaming/Channels/1
        // 子码流：
        // rtsp://admin:12345@192.168.1.64:554/Streaming/Channels/2
        // rtsp://admin:12345@192.168.1.64:554/h264/ch1/sub/av_stream

        rtsp_urls.push_back(url);
    }

    // 为每个摄像头创建发布者（配置 BestEffort QoS 策略）
    // 图像传输使用 BestEffort 可靠性策略，适合高频率、大数据量传输
    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile);
    
    for (size_t i = 0; i < rtsp_urls.size(); ++i) {
        auto pub = node->create_publisher<sensor_msgs::msg::Image>(
            "rtsp_image_" + std::to_string(i), 
            image_qos
        );
        publishers.push_back(pub);
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

    for (size_t i = 0; i < rtsp_urls.size(); ++i) {
        // 打开RTSP流
        bool opened = caps[i].open(rtsp_urls[i]);
        if (!opened) {
            RCLCPP_ERROR(node->get_logger(), "无法打开RTSP流 %s，请检查URL和网络连接！", rtsp_urls[i].c_str());
            rclcpp::shutdown();
            return -1;
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
        
        RCLCPP_INFO(node->get_logger(), "成功连接到RTSP流 %s", rtsp_urls[i].c_str());
        int frame_width = caps[i].get(cv::CAP_PROP_FRAME_WIDTH);
        int frame_height = caps[i].get(cv::CAP_PROP_FRAME_HEIGHT);
        RCLCPP_INFO(node->get_logger(), "视频尺寸: %dx%d", frame_width, frame_height);

        // 启动帧读取线程（用智能指针访问atomic），为每个相机创建独立线程
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