// 基于ROS 2的DeepSORVF跟踪节点，实现多相机图像的AIS+VIS+FUS处理
// 也就是与python版本的DeepSORVF跟踪节点功能一致
// 但是使用C++实现，而不是Python
// 使用ROS 2的特性，实现多相机图像的同步处理
// 注意：C++版本使用多线程而不是多进程（因为C++没有GIL限制），性能足够好
// 与Python版本的多进程功能等价，但实现更简单高效

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <marnav_interfaces/msg/ais_batch.hpp>
#include <marnav_interfaces/msg/gnss.hpp>
#include <marnav_interfaces/msg/visiable_tra_batch.hpp>
#include <marnav_interfaces/msg/visiable_tra.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <map>
#include <deque>
#include <algorithm>
#include <cmath>

#include "marnav_vis_cpp/AIS_utils.h"

using namespace std::chrono_literals;

// 相机配置结构
struct CameraConfig {
    std::string camera_name;
    std::string topic_name;
    int camera_index;
    // 相机内参
    double fov_hor;
    double fov_ver;
    double fx;
    double fy;
    double u0;
    double v0;
    // 畸变系数（鱼眼相机）
    double k1 = 0.0;
    double k2 = 0.0;
    double k3 = 0.0;
    double k4 = 0.0;
};

// 注意：CameraPosPara和AISData在AIS_utils.h中定义

// 轨迹数据结构（简化的Visiable_Tra）
struct VisiableTraData {
    int cam_idx;
    int64_t timestamp_ms;
    uint8_t ais;  // 0或1
    uint32_t mmsi;
    std::string ship_type;
    float sog;
    float cog;
    float lat;
    float lon;
    float box_x1;
    float box_y1;
    float box_x2;
    float box_y2;
};

// 任务数据结构（发送给worker）
struct ProcessingTask {
    int cam_idx;
    cv::Mat cv_image;
    int64_t current_timestamp_ms;
    std::vector<AISData> ais_batch;
    CameraPosPara camera_pos_para;
    // bin_inf 可以后续添加
};

// 结果数据结构（从worker接收）
struct ProcessingResult {
    int cam_idx;
    cv::Mat processed_image;
    int64_t timestamp_ms;
    std::vector<VisiableTraData> visiable_tra_list;
    double time_cost;
};

class DeepSORVFNode : public rclcpp::Node
{
public:
    DeepSORVFNode();
    ~DeepSORVFNode();

private:
    // 配置加载
    bool loadConfig(const std::string& config_file);
    
    // 回调函数
    void synchronized_camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img1,
                                     const sensor_msgs::msg::Image::ConstSharedPtr& img2,
                                     const sensor_msgs::msg::Image::ConstSharedPtr& img3);
    void aisbatch_callback(const marnav_interfaces::msg::AisBatch::SharedPtr msg);
    void gnss_callback(const marnav_interfaces::msg::Gnss::SharedPtr msg);
    void refresh_window_callback();
    void publish_trajectory_callback();
    
    // Worker线程函数（每个相机一个线程，等价于Python的多进程worker）
    void worker_thread(int cam_idx);
    
    // 辅助函数
    int64_t timestamp_to_ms(const builtin_interfaces::msg::Time& stamp);
    std::vector<AISData> convert_ais_batch(const marnav_interfaces::msg::AisBatch::SharedPtr& msg);
    
    // 配置参数
    std::vector<CameraConfig> camera_configs_;
    std::map<std::string, std::string> camera_name_mapping_; // topic -> camera_name
    std::pair<int, int> im_shape_; // width, height
    double angle_between_cameras_;
    std::string camera_type_; // "normal" or "fisheye"
    
    std::string ais_batch_topic_;
    std::string gnss_topic_;
    std::string fus_trajectory_topic_;
    
    int input_fps_;
    int output_fps_;
    int t_ms_; // 时间间隔（毫秒）
    int sync_queue_size_;
    double sync_slop_;
    int skip_interval_ms_;
    int max_dis_;
    
    // ROS 2相关
    std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>> camera_subscribers_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;
    
    rclcpp::Subscription<marnav_interfaces::msg::AisBatch>::SharedPtr aisbatch_subscriber_;
    rclcpp::Subscription<marnav_interfaces::msg::Gnss>::SharedPtr gnss_subscriber_;
    rclcpp::Publisher<marnav_interfaces::msg::VisiableTraBatch>::SharedPtr fus_trajectory_publisher_;
    
    rclcpp::TimerBase::SharedPtr window_timer_;
    rclcpp::TimerBase::SharedPtr trajectory_publish_timer_;
    
    // 数据缓存
    std::vector<AISData> aisbatch_cache_;
    std::mutex aisbatch_mutex_;
    
    std::map<int, CameraPosPara> camera_pos_para_;
    std::mutex camera_pos_para_mutex_;
    
    // 多线程处理（每个相机一个线程，等价于Python的多进程）
    std::vector<std::thread> worker_threads_;
    std::vector<std::queue<ProcessingTask>> input_queues_;
    std::vector<std::queue<ProcessingResult>> output_queues_;
    std::vector<std::shared_ptr<std::mutex>> input_queue_mutexes_;
    std::vector<std::shared_ptr<std::mutex>> output_queue_mutexes_;
    std::vector<std::shared_ptr<std::condition_variable>> input_queue_cvs_;
    std::atomic<bool> running_{true};
    
    // 处理结果缓存
    std::map<std::string, cv::Mat> latest_processed_images_;
    std::mutex processed_images_mutex_;
    
    // 融合轨迹缓存（每个相机一个）
    std::vector<std::vector<VisiableTraData>> fus_trajectory_;
    std::mutex fus_trajectory_mutex_;
    
    // 显示窗口名称
    std::string window_name_ = "ROS version 2 demo";
    
    // 最大队列大小
    static constexpr size_t MAX_QUEUE_SIZE = 10;
};

DeepSORVFNode::DeepSORVFNode()
    : Node("ais_vis_node")
{
    // 声明并获取配置文件参数
    this->declare_parameter<std::string>("config_file", "");
    std::string config_file = this->get_parameter("config_file").as_string();
    
    // 如果未指定配置文件，使用默认路径
    if (config_file.empty()) {
        try {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("marnav_vis");
            config_file = package_share_dir + "/config/track_realtime_config.yaml";
            RCLCPP_INFO(this->get_logger(), "未指定配置文件，使用默认路径: %s", config_file.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "查找默认配置文件失败: %s", e.what());
            throw;
        }
    }
    
    // 加载配置
    if (!loadConfig(config_file)) {
        RCLCPP_FATAL(this->get_logger(), "加载配置文件失败");
        throw std::runtime_error("配置文件加载失败");
    }
    
    // 打印配置信息
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "🚢 船只跟踪节点配置");
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "配置文件: %s", config_file.c_str());
    RCLCPP_INFO(this->get_logger(), "图像尺寸: %dx%d", im_shape_.first, im_shape_.second);
    RCLCPP_INFO(this->get_logger(), "相机数量: %zu", camera_configs_.size());
    for (size_t i = 0; i < camera_configs_.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "  相机映射关系%zu: %s -> %s", 
                   i, camera_configs_[i].topic_name.c_str(), camera_configs_[i].camera_name.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "输入/输出FPS: %d/%d", input_fps_, output_fps_);
    RCLCPP_INFO(this->get_logger(), "处理间隔: %d ms", skip_interval_ms_);
    RCLCPP_INFO(this->get_logger(), "同步队列: %d, 同步误差: %.3fs", sync_queue_size_, sync_slop_);
    RCLCPP_INFO(this->get_logger(), "============================================================");
    
    // 初始化处理队列和互斥锁
    size_t num_cameras = camera_configs_.size();
    input_queues_.resize(num_cameras);
    output_queues_.resize(num_cameras);
    // input_queue_mutexes_.resize(num_cameras);
    // output_queue_mutexes_.resize(num_cameras);
    // input_queue_cvs_.resize(num_cameras);
    fus_trajectory_.resize(num_cameras);
    
    // 1. 正确初始化互斥锁和条件变量（使用emplace_back）
    input_queue_mutexes_.clear();
    output_queue_mutexes_.clear();
    input_queue_cvs_.clear();
    for (size_t i = 0; i < num_cameras; ++i) {
        input_queue_mutexes_.emplace_back(std::make_shared<std::mutex>());
        output_queue_mutexes_.emplace_back(std::make_shared<std::mutex>());
        input_queue_cvs_.emplace_back(std::make_shared<std::condition_variable>());
    }

    // 初始化每个相机的最新处理图像为空白图像
    cv::Mat blank_image = cv::Mat::zeros(im_shape_.second, im_shape_.first, CV_8UC3);
    for (size_t i = 0; i < num_cameras; ++i) {
        latest_processed_images_[camera_configs_[i].topic_name] = blank_image.clone();
    }
    
    // 创建worker线程（每个相机一个线程，等价于Python的多进程worker）
    worker_threads_.reserve(num_cameras);
    for (size_t i = 0; i < num_cameras; ++i) {
        worker_threads_.emplace_back(&DeepSORVFNode::worker_thread, this, i);
    }
    
    // 创建显示窗口
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    
    // 配置QoS
    rclcpp::QoS image_qos(3);
    image_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    image_qos.history(rclcpp::HistoryPolicy::KeepLast);
    
    // 创建相机图像订阅器（目前只支持3个相机）
    if (num_cameras >= 3) {
        camera_subscribers_.resize(3);
        camera_subscribers_[0] = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, camera_configs_[0].topic_name, image_qos.get_rmw_qos_profile());
        camera_subscribers_[1] = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, camera_configs_[1].topic_name, image_qos.get_rmw_qos_profile());
        camera_subscribers_[2] = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, camera_configs_[2].topic_name, image_qos.get_rmw_qos_profile());
        
        // 创建时间同步器
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        
        // ApproximateTime策略构造函数参数：时间窗口（纳秒）
        // 将sync_slop_（秒）转换为纳秒
        uint32_t age_ns = static_cast<uint32_t>(sync_slop_ * 1e9);
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(age_ns),
            *camera_subscribers_[0], *camera_subscribers_[1], *camera_subscribers_[2]);
        
        sync_->registerCallback(std::bind(&DeepSORVFNode::synchronized_camera_callback, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    } else {
        RCLCPP_ERROR(this->get_logger(), "当前只支持3个相机，但配置中只有 %zu 个", num_cameras);
    }
    
    // 订阅AIS和GNSS数据
    aisbatch_subscriber_ = this->create_subscription<marnav_interfaces::msg::AisBatch>(
        ais_batch_topic_, 10,
        std::bind(&DeepSORVFNode::aisbatch_callback, this, std::placeholders::_1));
    
    gnss_subscriber_ = this->create_subscription<marnav_interfaces::msg::Gnss>(
        gnss_topic_, 10,
        std::bind(&DeepSORVFNode::gnss_callback, this, std::placeholders::_1));
    
    // 创建定时器
    window_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / output_fps_),
        std::bind(&DeepSORVFNode::refresh_window_callback, this));
    
    trajectory_publish_timer_ = this->create_wall_timer(
        1s,
        std::bind(&DeepSORVFNode::publish_trajectory_callback, this));
    
    // 创建轨迹发布器
    fus_trajectory_publisher_ = this->create_publisher<marnav_interfaces::msg::VisiableTraBatch>(
        fus_trajectory_topic_, image_qos);
}

DeepSORVFNode::~DeepSORVFNode()
{
    // 停止worker线程
    running_ = false;
    for (size_t i = 0; i < input_queue_cvs_.size(); ++i) {
        input_queue_cvs_[i]->notify_all();
    }
    
    // 等待所有线程结束
    for (auto& thread : worker_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    cv::destroyAllWindows();
}

bool DeepSORVFNode::loadConfig(const std::string& config_file)
{
    try {
        YAML::Node config = YAML::LoadFile(config_file);
        
        // 加载相机配置
        if (!config["camera"]) {
            RCLCPP_ERROR(this->get_logger(), "配置文件中未找到'camera'节点");
            return false;
        }
        
        const YAML::Node& camera_config = config["camera"];
        auto wh = camera_config["width_height"].as<std::vector<int>>();
        im_shape_ = {wh[0], wh[1]};
        angle_between_cameras_ = camera_config["angle_between_cameras"].as<double>(60.0);
        max_dis_ = std::min(im_shape_.first, im_shape_.second) / 2;
        
        camera_configs_.clear();
        for (const auto& cam : camera_config["camera_parameters"]) {
            CameraConfig cam_cfg;
            cam_cfg.camera_name = cam["camera_name"].as<std::string>();
            cam_cfg.topic_name = cam["topic_name"].as<std::string>();
            cam_cfg.camera_index = cam["camera_index"].as<int>();
            
            const auto& matrix = cam["camera_matrix"];
            cam_cfg.fov_hor = matrix["fov_hor"].as<double>();
            cam_cfg.fov_ver = matrix["fov_ver"].as<double>();
            cam_cfg.fx = matrix["fx"].as<double>();
            cam_cfg.fy = matrix["fy"].as<double>();
            cam_cfg.u0 = matrix["u0"].as<double>();
            cam_cfg.v0 = matrix["v0"].as<double>();
            
            if (cam["distortion_coefficients"]) {
                const auto& dist = cam["distortion_coefficients"];
                cam_cfg.k1 = dist["k1"].as<double>();
                cam_cfg.k2 = dist["k2"].as<double>();
                cam_cfg.k3 = dist["k3"].as<double>();
                cam_cfg.k4 = dist["k4"].as<double>();
            }
            
            camera_configs_.push_back(cam_cfg);
            camera_name_mapping_[cam_cfg.topic_name] = cam_cfg.camera_name;
        }
        
        // 加载AIS配置
        if (config["ais"]) {
            ais_batch_topic_ = config["ais"]["ais_batch_pub_topic"].as<std::string>("/ais_batch_topic");
        } else {
            ais_batch_topic_ = "/ais_batch_topic";
        }
        
        // 加载GNSS配置
        if (config["gnss"]) {
            gnss_topic_ = config["gnss"]["gnss_pub_topic"].as<std::string>("/gnss_topic");
        } else {
            gnss_topic_ = "/gnss_topic";
        }
        
        // 加载DeepSORVF配置
        if (config["DeepSORVF"]) {
            const auto& ds_config = config["DeepSORVF"];
            fus_trajectory_topic_ = ds_config["fus_trajectory_topic"].as<std::string>("/fus_trajectory_topic");
            input_fps_ = ds_config["input_fps"].as<int>(20);
            output_fps_ = ds_config["output_fps"].as<int>(10);
            sync_queue_size_ = ds_config["sync_queue_size"].as<int>(10);
            sync_slop_ = ds_config["sync_slop"].as<double>(0.1);
            skip_interval_ms_ = ds_config["skip_interval"].as<int>(1000);
            camera_type_ = ds_config["camera_type"].as<std::string>("normal");
        } else {
            fus_trajectory_topic_ = "/fus_trajectory_topic";
            input_fps_ = 20;
            output_fps_ = 10;
            sync_queue_size_ = 10;
            sync_slop_ = 0.1;
            skip_interval_ms_ = 1000;
            camera_type_ = "normal";
        }
        
        t_ms_ = 1000 / input_fps_;
        
        return true;
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "解析YAML配置文件失败: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "读取配置文件时发生错误: %s", e.what());
        return false;
    }
}

int64_t DeepSORVFNode::timestamp_to_ms(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<int64_t>(stamp.sec) * 1000 + stamp.nanosec / 1000000;
}

std::vector<AISData> DeepSORVFNode::convert_ais_batch(const marnav_interfaces::msg::AisBatch::SharedPtr& msg)
{
    std::vector<AISData> result;
    for (const auto& ais : msg->ais_list) {
        AISData data;
        data.mmsi = ais.mmsi;
        data.lon = ais.lon;
        data.lat = ais.lat;
        data.speed = ais.speed;
        data.course = ais.course;
        data.heading = ais.heading;
        data.type = ais.type;
        data.timestamp_ms = timestamp_to_ms(ais.timestamp);
        result.push_back(data);
    }
    return result;
}

void DeepSORVFNode::synchronized_camera_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img1,
    const sensor_msgs::msg::Image::ConstSharedPtr& img2,
    const sensor_msgs::msg::Image::ConstSharedPtr& img3)
{
    // 检查相机位置参数是否已初始化
    {
        std::lock_guard<std::mutex> lock(camera_pos_para_mutex_);
        if (camera_pos_para_.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "等待GNSS数据更新相机位置参数，暂不处理图像帧");
            return;
        }
    }
    
    // 检查AIS数据（非必需，仅警告）
    {
        std::lock_guard<std::mutex> lock(aisbatch_mutex_);
        if (aisbatch_cache_.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "未收到AIS数据，将仅使用视觉跟踪模式");
        }
    }
    
    // 转换图像
    std::vector<cv::Mat> cv_images;
    try {
        cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(img1, sensor_msgs::image_encodings::BGR8);
        cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(img2, sensor_msgs::image_encodings::BGR8);
        cv_bridge::CvImagePtr cv_ptr3 = cv_bridge::toCvCopy(img3, sensor_msgs::image_encodings::BGR8);
        cv_images.push_back(cv_ptr1->image);
        cv_images.push_back(cv_ptr2->image);
        cv_images.push_back(cv_ptr3->image);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        return;
    }
    
    int64_t current_timestamp = timestamp_to_ms(img1->header.stamp);
    
    // 获取AIS数据和相机位置参数
    std::vector<AISData> ais_batch;
    std::map<int, CameraPosPara> camera_pos_para;
    {
        std::lock_guard<std::mutex> lock(aisbatch_mutex_);
        ais_batch = aisbatch_cache_;
    }
    {
        std::lock_guard<std::mutex> lock(camera_pos_para_mutex_);
        camera_pos_para = camera_pos_para_;
    }
    
    // 为每个相机提交任务
    for (size_t cam_idx = 0; cam_idx < camera_configs_.size() && cam_idx < cv_images.size(); ++cam_idx) {
        // 检查相机位置参数
        if (camera_pos_para.find(cam_idx) == camera_pos_para.end()) {
            RCLCPP_WARN(this->get_logger(), "camera_pos_para[%zu] 缺失，跳过该帧", cam_idx);
            continue;
        }
        
        ProcessingTask task;
        task.cam_idx = cam_idx;
        task.cv_image = cv_images[cam_idx].clone();
        task.current_timestamp_ms = current_timestamp;
        task.ais_batch = ais_batch;
        task.camera_pos_para = camera_pos_para[cam_idx];
        
        // 提交到输入队列
        {
            std::unique_lock<std::mutex> lock(*input_queue_mutexes_[cam_idx]);
            if (input_queues_[cam_idx].size() < MAX_QUEUE_SIZE) {
                input_queues_[cam_idx].push(task);
                input_queue_cvs_[cam_idx]->notify_one();
            } else {
                RCLCPP_DEBUG(this->get_logger(), "多线程输入队列满，丢弃 cam%zu 当前帧", cam_idx);
            }
        }
    }
}

void DeepSORVFNode::aisbatch_callback(const marnav_interfaces::msg::AisBatch::SharedPtr msg)
{
    std::vector<AISData> converted = convert_ais_batch(msg);
    
    std::lock_guard<std::mutex> lock(aisbatch_mutex_);
    aisbatch_cache_ = converted;
    
    // 限制缓存大小（最多100条）
    if (aisbatch_cache_.size() > 100) {
        aisbatch_cache_.erase(aisbatch_cache_.begin(), 
                             aisbatch_cache_.begin() + (aisbatch_cache_.size() - 100));
    }
}

void DeepSORVFNode::gnss_callback(const marnav_interfaces::msg::Gnss::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(camera_pos_para_mutex_);
    
    // 更新每个相机的位置参数
    for (size_t idx = 0; idx < camera_configs_.size(); ++idx) {
        // 计算水平朝向（中间相机不变，左右相机调整±N度）
        double horizontal_orientation = fmod(msg->horizontal_orientation + 
            (static_cast<int>(idx) - 1) * angle_between_cameras_, 360.0);
        
        CameraPosPara para;
        para.longitude = msg->longitude;
        para.latitude = msg->latitude;
        para.horizontal_orientation = horizontal_orientation;
        para.vertical_orientation = msg->vertical_orientation;
        para.camera_height = msg->camera_height;
        
        // 从相机配置中获取内参
        para.fov_hor = camera_configs_[idx].fov_hor;
        para.fov_ver = camera_configs_[idx].fov_ver;
        para.fx = camera_configs_[idx].fx;
        para.fy = camera_configs_[idx].fy;
        para.u0 = camera_configs_[idx].u0;
        para.v0 = camera_configs_[idx].v0;
        
        // 如果是鱼眼相机，添加畸变系数
        if (camera_type_ == "fisheye") {
            para.k1 = camera_configs_[idx].k1;
            para.k2 = camera_configs_[idx].k2;
            para.k3 = camera_configs_[idx].k3;
            para.k4 = camera_configs_[idx].k4;
        }
        
        camera_pos_para_[idx] = para;
    }
}

void DeepSORVFNode::worker_thread(int cam_idx)
{
    // Worker线程处理函数（等价于Python的multi_proc_worker）
    // 按照Python版本的逻辑实现AIS/VIS/FUS/DRAW处理
    
    RCLCPP_INFO(this->get_logger(), "Worker线程 %d 已启动", cam_idx);
    
    // 创建AISPRO实例（类似Python: aispro = AISPRO(im_shape, t)）
    AISPRO aispro(im_shape_, t_ms_);
    // TODO: 后续需要添加VISPRO, FUSPRO, DRAW实例
    // VISPRO vispro(1, 0, t_ms_);
    // FUSPRO fuspro(max_dis_, im_shape_, t_ms_);
    // DRAW dra(im_shape_, t_ms_);
    
    // 缓存上一窗口的融合轨迹（类似Python: Last_Visiable_Tra = pd.DataFrame()）
    std::vector<VisiableTraData> last_visiable_tra;
    // 记录上次处理时所属的时间窗口（类似Python: last_processed_window = -1）
    int64_t last_processed_window = -1;
    
    while (running_) {
        ProcessingTask task;
        bool has_task = false;
        
        // 从输入队列获取任务
        {
            std::unique_lock<std::mutex> lock(*input_queue_mutexes_[cam_idx]);
            input_queue_cvs_[cam_idx]->wait(lock, [this, cam_idx, &has_task, &task]() {
                if (!running_) return true;
                if (!input_queues_[cam_idx].empty()) {
                    task = input_queues_[cam_idx].front();
                    input_queues_[cam_idx].pop();
                    has_task = true;
                    return true;
                }
                return false;
            });
        }
        
        if (!running_ && !has_task) break;
        if (!has_task) continue;
        
        auto start_time = std::chrono::steady_clock::now();
        
        try {
            // 计算当前时间戳属于哪个窗口（类似Python: current_window = current_timestamp // skip_interval）
            int64_t current_window = task.current_timestamp_ms / skip_interval_ms_;
            
            // 只有进入新窗口时才处理（类似Python: process_ais_vis_fus = (current_window != last_processed_window)）
            bool process_ais_vis_fus = (current_window != last_processed_window);
            
            ProcessingResult result;
            result.cam_idx = cam_idx;
            result.timestamp_ms = task.current_timestamp_ms;
            
            if (process_ais_vis_fus) {
                // 1. AIS处理（类似Python: AIS_vis, AIS_cur = aispro.process(...)）
                std::vector<AISVisData> ais_vis;
                std::vector<AISData> ais_cur;
                aispro.process(task.ais_batch, task.camera_pos_para, 
                              task.current_timestamp_ms, camera_type_, 
                              ais_vis, ais_cur);
                
                // TODO: 2. VIS处理（类似Python: Vis_tra, Vis_cur = vispro.feedCap(...)）
                // std::vector<VisData> vis_tra, vis_cur;
                // vispro.feedCap(task.cv_image, ais_vis, bin_inf, task.current_timestamp_ms, vis_tra, vis_cur);
                
                // TODO: 3. FUS处理（类似Python: Fus_tra, updated_bin = fuspro.fusion(...)）
                // std::vector<FusTraData> fus_tra;
                // fuspro.fusion(ais_vis, ais_cur, vis_tra, vis_cur, task.current_timestamp_ms, fus_tra);
                
                // TODO: 4. DRAW处理（类似Python: im, Visiable_Tra = dra.draw_match_traj(...)）
                // 目前暂时只复制原图，等待DRAW实现后再替换
                cv::Mat processed_image = task.cv_image.clone();
                
                // 将AIS_vis转换为VisiableTraData（目前只包含AIS数据，等待FUS完成后会有完整的轨迹数据）
                std::vector<VisiableTraData> visiable_tra_list;
                for (const auto& vis : ais_vis) {
                    VisiableTraData tra;
                    tra.cam_idx = cam_idx;
                    tra.timestamp_ms = vis.timestamp_ms * 1000;  // AISVisData中timestamp_ms是秒，转换为毫秒
                    tra.ais = 1;  // 表示包含AIS信息
                    tra.mmsi = vis.mmsi;
                    tra.ship_type = "";  // TODO: 从type字段获取船只类型，目前为空
                    tra.sog = static_cast<float>(vis.speed);
                    tra.cog = static_cast<float>(vis.course);
                    tra.lat = static_cast<float>(vis.lat);
                    tra.lon = static_cast<float>(vis.lon);
                    // TODO: 需要实际的检测框坐标，目前使用图像坐标点周围50像素的占位框
                    tra.box_x1 = static_cast<float>(std::max(0, vis.x - 50));
                    tra.box_y1 = static_cast<float>(std::max(0, vis.y - 50));
                    tra.box_x2 = static_cast<float>(std::min(im_shape_.first - 1, vis.x + 50));
                    tra.box_y2 = static_cast<float>(std::min(im_shape_.second - 1, vis.y + 50));
                    visiable_tra_list.push_back(tra);
                }
                
                result.processed_image = processed_image;
                result.visiable_tra_list = visiable_tra_list;
                last_visiable_tra = visiable_tra_list;  // 更新缓存
                last_processed_window = current_window;  // 更新窗口标记
            } else {
                // 跳过处理，直接使用原图（类似Python: im = dra.draw_no_match_traj(cv_image)）
                result.processed_image = task.cv_image.clone();
                result.visiable_tra_list = last_visiable_tra;  // 使用缓存的轨迹
            }
            
            auto end_time = std::chrono::steady_clock::now();
            result.time_cost = std::chrono::duration<double>(end_time - start_time).count();
            
            // 将结果放入输出队列
            {
                std::unique_lock<std::mutex> lock(*output_queue_mutexes_[cam_idx]);
                if (output_queues_[cam_idx].size() < MAX_QUEUE_SIZE) {
                    output_queues_[cam_idx].push(result);
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "输出队列满，丢弃 cam%d 的结果", cam_idx);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Worker cam%d 处理出错: %s", cam_idx, e.what());
            // 继续处理下一帧，不中断线程
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Worker线程 %d 已退出", cam_idx);
}

void DeepSORVFNode::refresh_window_callback()
{
    // 回收worker线程的输出结果
    for (size_t cam_idx = 0; cam_idx < camera_configs_.size(); ++cam_idx) {
        std::queue<ProcessingResult> temp_results;
        
        // 从输出队列取出所有结果
        {
            std::unique_lock<std::mutex> lock(*output_queue_mutexes_[cam_idx]);
            temp_results = output_queues_[cam_idx];
            while (!output_queues_[cam_idx].empty()) {
                output_queues_[cam_idx].pop();
            }
        }
        
        // 处理结果
        ProcessingResult latest_result;
        bool has_result = false;
        while (!temp_results.empty()) {
            latest_result = temp_results.front();
            temp_results.pop();
            has_result = true;
        }
        
        if (has_result) {
            // 更新处理后的图像
            {
                std::lock_guard<std::mutex> lock(processed_images_mutex_);
                latest_processed_images_[camera_configs_[cam_idx].topic_name] = 
                    latest_result.processed_image.clone();
            }
            
            // 更新融合轨迹
            {
                std::lock_guard<std::mutex> lock(fus_trajectory_mutex_);
                fus_trajectory_[cam_idx] = latest_result.visiable_tra_list;
            }
        }
    }
    
    // 拼接图像并显示
    std::vector<cv::Mat> images_to_concat;
    {
        std::lock_guard<std::mutex> lock(processed_images_mutex_);
        
        cv::Size target_size;
        bool all_valid = true;
        
        for (const auto& config : camera_configs_) {
            auto it = latest_processed_images_.find(config.topic_name);
            if (it == latest_processed_images_.end() || it->second.empty()) {
                all_valid = false;
                break;
            }
            
            if (target_size.width == 0) {
                target_size = it->second.size();
            } else if (it->second.size() != target_size) {
                // 调整图像尺寸以匹配
                cv::Mat resized;
                cv::resize(it->second, resized, target_size);
                images_to_concat.push_back(resized);
                continue;
            }
            
            images_to_concat.push_back(it->second);
        }
        
        if (all_valid && images_to_concat.size() == camera_configs_.size()) {
            try {
                cv::Mat stitched_image;
                cv::hconcat(images_to_concat, stitched_image);
                cv::putText(stitched_image, "Unified Time: " + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
                cv::imshow(window_name_, stitched_image);
                cv::waitKey(1);
            } catch (const cv::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "图像拼接失败: %s", e.what());
            }
        }
    }
}

void DeepSORVFNode::publish_trajectory_callback()
{
    // 定时发布融合轨迹（1秒一次）
    marnav_interfaces::msg::VisiableTraBatch msg_batch;
    msg_batch.visiable_tra_list.clear();
    
    std::lock_guard<std::mutex> lock(fus_trajectory_mutex_);
    
    for (size_t cam_idx = 0; cam_idx < fus_trajectory_.size(); ++cam_idx) {
        if (fus_trajectory_[cam_idx].empty()) {
            continue;
        }
        
        // 使用第一个轨迹的时间戳作为batch的时间戳
        if (!fus_trajectory_[cam_idx].empty()) {
            int64_t batch_timestamp_ms = fus_trajectory_[cam_idx][0].timestamp_ms;
            msg_batch.timestamp.sec = batch_timestamp_ms / 1000;
            msg_batch.timestamp.nanosec = (batch_timestamp_ms % 1000) * 1000000;
        }
        
        for (const auto& tra : fus_trajectory_[cam_idx]) {
            marnav_interfaces::msg::VisiableTra msg;
            
            // 使用映射后的相机名称
            std::string topic_name = camera_configs_[cam_idx].topic_name;
            msg.camera_name = camera_name_mapping_[topic_name];
            
            // 时间戳
            msg.timestamp.sec = tra.timestamp_ms / 1000;
            msg.timestamp.nanosec = (tra.timestamp_ms % 1000) * 1000000;
            
            // AIS相关字段
            msg.ais = tra.ais;
            msg.mmsi = tra.mmsi;
            msg.ship_type = tra.ship_type.empty() ? "cargo ship" : tra.ship_type;
            
            // 速度和航向
            msg.sog = tra.sog >= 0.0f ? tra.sog : 0.0f;
            msg.cog = tra.cog >= 0.0f ? tra.cog : 0.0f;
            
            // 经纬度
            msg.lat = tra.lat;
            msg.lon = tra.lon;
            
            // 检测框坐标
            msg.box_x1 = tra.box_x1;
            msg.box_y1 = tra.box_y1;
            msg.box_x2 = tra.box_x2;
            msg.box_y2 = tra.box_y2;
            
            msg_batch.visiable_tra_list.push_back(msg);
        }
    }
    
    // 只有在有数据时才发布
    if (!msg_batch.visiable_tra_list.empty()) {
        fus_trajectory_publisher_->publish(msg_batch);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<DeepSORVFNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "节点异常: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
