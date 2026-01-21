#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

// jh消息头文件
#include "mycustface/msg/j_hjpg.hpp"
#include "mycustface/msg/header.hpp"


// 船只跟踪消息体文件
#include "marnav_interfaces/msg/visiable_tra.hpp"
#include "marnav_interfaces/msg/visiable_tra_batch.hpp"
#include "marnav_interfaces/msg/gnss.hpp"


#include "JH_SeamStitcher.hpp"
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <atomic>
#include <nlohmann/json.hpp>
#include <fstream>
#include <array>

// YAML解析库
#include <yaml-cpp/yaml.h>

// ROS 2包路径查找
#include <ament_index_cpp/get_package_share_directory.hpp>

using json = nlohmann::json;
using namespace std;
using namespace sensor_msgs::msg;
using namespace message_filters;

// 消息类型别名和服务类型别名
using JHjpgMsg = mycustface::msg::JHjpg;
using VisiableTraMsg = marnav_interfaces::msg::VisiableTra;
using VisiableTraBatchMsg = marnav_interfaces::msg::VisiableTraBatch;
using GnssMsg = marnav_interfaces::msg::Gnss;

// 相机配置结构体
struct CameraConfig {
    std::string camera_name;
    std::string topic_name;
    int camera_index;
};

// 配置结构体
struct NodeConfig {
    std::vector<CameraConfig> cameras;
    std::string jhjpg_msg_topic;
    int publish_timeout;
    std::string fus_trajectory_topic;
    std::string gnss_topic;
    std::string get_camera_params_service;
    
    // Stitcher参数
    int refresh_time;
    int min_key_points;
    double min_confidence;
    int min_inliers;
    double max_focal_variance;
    double y_tolerance;
    float roi_threshold;
    double scale;
    bool crop_or_not;
    int crop_top;
    int crop_bottom;
    int crop_left;
    int crop_right;
    bool draw_box_or_not;
    bool save_camera_params;
    std::string remap_dir;
    std::string cache_file;
    std::string calib_dir;
    bool use_saved_camera_params;
    double FOV_hor;
    double FOV_ver;
};

/**
 * 自动获取 RV 工作空间根路径（如 /home/tl/RV/）
 * 优先级：RV_WS 环境变量 > 从包路径反向推导 > 当前目录
 */
string getRVWorkspaceRoot(rclcpp::Logger logger) {
    // 1. 优先读取环境变量 RV_WS（用户需在 .bashrc 中 export RV_WS=/home/xxx/RV & source ~/.bashrc）
    const char* rv_ws_env = std::getenv("RV_WS");
    if (rv_ws_env != nullptr && !std::string(rv_ws_env).empty()) {
        std::string env_path = std::filesystem::canonical(rv_ws_env).string();
        RCLCPP_INFO(logger, "✅ 从环境变量 RV_WS 获取工作空间根路径: %s", env_path.c_str());
        return env_path;
    }
    else {
        RCLCPP_WARN(logger, "❌ 未找到环境变量 RV_WS，尝试从包路径反向推导");
    }
    // 2. 从包路径反向推导（如 /home/tl/RV/src/image_stitching_pkg）
    try {
        string package_path = ament_index_cpp::get_package_share_directory("image_stitching_pkg");
        std::filesystem::path pkg_path(package_path);
        std::filesystem::path ws_root = pkg_path;
        // 循环向上找，直到找到包含 "install" 的目录
        while (ws_root.has_parent_path() && ws_root.filename() != "install") {
            ws_root = ws_root.parent_path();
        }
        // 再向上跳一级，得到RV根目录
        if (ws_root.filename() == "install") {
            ws_root = ws_root.parent_path();
        }
        RCLCPP_INFO(logger, "✅ 从包路径反向推导工作空间根路径: %s", ws_root.c_str());
        return ws_root;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "❌ 从包路径反向推导工作空间根路径失败: %s", e.what());
        throw;
    }
    // 3. 降级：使用当前可执行文件所在目录反向推导（兜底）
    std::filesystem::path exe_path = std::filesystem::canonical("/proc/self/exe").parent_path();
    // 可执行文件路径示例：/home/tl/RV/install/image_stitching_pkg/lib/image_stitching_pkg/JH_ROS_stitch
    // 向上跳 4 级到 RV 根目录：lib → image_stitching_pkg → install → RV
    std::filesystem::path fallback_ws = exe_path.parent_path().parent_path().parent_path().parent_path();
    if (std::filesystem::exists(fallback_ws)) {
        std::string fallback_path = fallback_ws.string();
        RCLCPP_WARN(logger, "⚠️ 使用兜底路径作为工作空间: %s", fallback_path.c_str());
        return fallback_path;
    }
    else {
        RCLCPP_ERROR(logger, "❌ 无法确定工作空间根路径，请检查可执行文件路径或配置 RV_WS 环境变量");
        throw std::runtime_error("无法确定工作空间根路径");
    }
}

/**
 * 拼接完整路径（相对于工作空间根路径）
*/ 
string resolveFullPath(const std::string& ws_root, const std::string& relative_path, rclcpp::Logger logger) {
    if (std::filesystem::path(relative_path).is_absolute()) {
        RCLCPP_WARN(logger, "⚠️ YAML 中配置的路径已是绝对路径，直接使用: %s", relative_path.c_str());
        return relative_path;
    }
    std::filesystem::path full_path = std::filesystem::path(ws_root) / relative_path;
    RCLCPP_INFO(logger, "📌 拼接后完整路径: %s", full_path.string().c_str());
    return full_path.string();
}

// 从YAML文件加载配置
bool loadConfigFromYAML(const std::string& config_file_path, NodeConfig& config, rclcpp::Logger logger) {
    try {
        YAML::Node yaml_config = YAML::LoadFile(config_file_path);
        
        if (!yaml_config["parameters"]) {
            RCLCPP_ERROR(logger, "YAML配置文件中未找到'parameters'节点");
            return false;
        }
        
        const YAML::Node& params = yaml_config["parameters"];
        
        // 读取主程序参数
        if (!params["Main_parameters"]) {
            RCLCPP_ERROR(logger, "YAML配置文件中未找到'Main_parameters'节点");
            return false;
        }
        
        const YAML::Node& main_params = params["Main_parameters"];
        
        // 读取相机配置
        if (!main_params["cameras"] || !main_params["cameras"].IsSequence()) {
            RCLCPP_ERROR(logger, "YAML配置文件中未找到有效的'cameras'列表");
            return false;
        }
        
        config.cameras.clear();
        for (const auto& cam : main_params["cameras"]) {
            CameraConfig camera_config;
            if (!cam["camera_name"] || !cam["topic_name"] || !cam["camera_index"]) {
                RCLCPP_WARN(logger, "跳过无效的相机配置项");
                continue;
            }
            camera_config.camera_name = cam["camera_name"].as<std::string>();
            camera_config.topic_name = cam["topic_name"].as<std::string>();
            camera_config.camera_index = cam["camera_index"].as<int>();
            config.cameras.push_back(camera_config);
        }
        
        if (config.cameras.empty()) {
            RCLCPP_ERROR(logger, "未找到有效的相机配置");
            return false;
        }
        
        // 读取其他主程序参数
        config.jhjpg_msg_topic = main_params["JHjpgMsg_topic"].as<std::string>();
        config.publish_timeout = main_params["publish_timeout"].as<int>();
        config.fus_trajectory_topic = main_params["fus_trajectory_topic"].as<std::string>();
        config.gnss_topic = main_params["gnss_topic"].as<std::string>();
        config.get_camera_params_service = main_params["get_camera_params_service"].as<std::string>();
        
        // 读取Stitcher参数
        if (!params["Stitcher_parameters"]) {
            RCLCPP_ERROR(logger, "YAML配置文件中未找到'Stitcher_parameters'节点");
            return false;
        }
        
        const YAML::Node& stitcher_params = params["Stitcher_parameters"];
        config.refresh_time = stitcher_params["refresh_time"].as<int>();
        config.min_key_points = stitcher_params["min_key_points"].as<int>();
        config.min_confidence = stitcher_params["min_confidence"].as<double>();
        config.min_inliers = stitcher_params["min_inliers"].as<int>();
        config.max_focal_variance = stitcher_params["max_focal_variance"].as<double>();
        config.y_tolerance = stitcher_params["y_tolerance"].as<double>();
        config.roi_threshold = stitcher_params["roi_threshold"].as<float>();
        config.scale = stitcher_params["scale"].as<double>();
        config.crop_or_not = stitcher_params["crop_or_not"].as<bool>();
        config.crop_top = stitcher_params["crop_top"].as<int>();
        config.crop_bottom = stitcher_params["crop_bottom"].as<int>();
        config.crop_left = stitcher_params["crop_left"].as<int>();
        config.crop_right = stitcher_params["crop_right"].as<int>();
        config.draw_box_or_not = stitcher_params["draw_box_or_not"].as<bool>();
        config.save_camera_params = stitcher_params["save_camera_params"].as<bool>();
        string relative_remap_dir = stitcher_params["remap_dir"].as<std::string>();
        string relative_cache_file = stitcher_params["cache_file"].as<std::string>();
        string relative_calib_dir = stitcher_params["calib_dir"].as<std::string>();
        // 1. 自动获取工作空间根路径
        string ws_root = getRVWorkspaceRoot(logger);
        // 2. 拼接完整路径
        config.remap_dir = resolveFullPath(ws_root, relative_remap_dir, logger);
        config.cache_file = resolveFullPath(ws_root, relative_cache_file, logger);
        config.calib_dir = resolveFullPath(ws_root, relative_calib_dir, logger);
        config.use_saved_camera_params = stitcher_params["use_saved_camera_params"].as<bool>();
        config.FOV_hor = stitcher_params["FOV_hor"].as<double>();
        config.FOV_ver = stitcher_params["FOV_ver"].as<double>();
        
        RCLCPP_INFO(logger, "成功加载配置文件: %s", config_file_path.c_str());
        RCLCPP_INFO(logger, "加载了 %zu 个相机配置", config.cameras.size());
        
        return true;
        
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(logger, "解析YAML配置文件失败: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "读取配置文件时发生错误: %s", e.what());
        return false;
    }
}

class JHRos2StitchNode : public rclcpp::Node {
public:
    JHRos2StitchNode() : Node("topic_stitch") {
        // 创建可重入回调组
        callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        
        // 获取配置文件路径（支持通过ROS参数配置）
        this->declare_parameter<std::string>("config_file", "");
        std::string config_file_path = this->get_parameter("config_file").as_string();
        
        // 如果未指定配置文件，使用默认路径
        if (config_file_path.empty()) {
            std::string package_path = ament_index_cpp::get_package_share_directory("image_stitching_pkg");
            config_file_path = package_path + "/config/JH_stitch_config.yaml";
            RCLCPP_INFO(this->get_logger(), "未指定配置文件，使用默认路径: %s", config_file_path.c_str());
        }
        
        // 检查配置文件是否存在
        std::ifstream file_check(config_file_path);
        if (!file_check.good()) {
            RCLCPP_FATAL(this->get_logger(), "配置文件不存在: %s", config_file_path.c_str());
            throw std::runtime_error("配置文件不存在: " + config_file_path);
        }
        file_check.close();
        
        // 从YAML文件加载配置
        NodeConfig config;
        if (!loadConfigFromYAML(config_file_path, config, this->get_logger())) {
            RCLCPP_FATAL(this->get_logger(), "加载配置文件失败，程序退出");
            throw std::runtime_error("加载配置文件失败");
        }
        
        // 从配置中构建相机名称到索引的映射
        cam_name_to_idx_.clear();
        std::vector<std::string> camera_topic_names;
        for (const auto& cam : config.cameras) {
            cam_name_to_idx_[cam.camera_name] = cam.camera_index;
            camera_topic_names.push_back(cam.topic_name);
            RCLCPP_INFO(this->get_logger(), "相机配置: name=%s, topic=%s, index=%d", 
                       cam.camera_name.c_str(), cam.topic_name.c_str(), cam.camera_index);
        }
        
        // 检查相机数量（当前实现固定为3个）
        if (camera_topic_names.size() != 3) {
            RCLCPP_FATAL(this->get_logger(), "当前实现仅支持3个相机，但配置文件中定义了 %zu 个相机", camera_topic_names.size());
            throw std::runtime_error("相机数量不匹配");
        }
        
        // 使用配置创建拼接器
        stitcher_ = std::make_unique<JHStitcher>(
            cam_name_to_idx_, //相机名称到索引的映射
            config.refresh_time,
            config.min_key_points,
            config.min_confidence,
            config.min_inliers,
            config.max_focal_variance,
            config.y_tolerance,
            config.roi_threshold,
            config.scale,
            config.crop_or_not,
            config.crop_top,
            config.crop_bottom,
            config.crop_left,
            config.crop_right,
            config.draw_box_or_not,
            config.save_camera_params,
            config.remap_dir,
            config.cache_file,
            config.calib_dir,
            config.use_saved_camera_params,
            config.FOV_hor,
            config.FOV_ver
        );
        
        // 初始化订阅器（配置QoS策略以匹配发布者）
        // 图像话题通常使用 BEST_EFFORT 可靠性策略
        auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile);
        
        // 从配置中读取话题名称创建订阅器
        if (!use_timer_stitch_) {
            img1_sub_ = std::make_shared<Subscriber>(this, camera_topic_names[0], image_qos.get_rmw_qos_profile());
            img2_sub_ = std::make_shared<Subscriber>(this, camera_topic_names[1], image_qos.get_rmw_qos_profile());
            img3_sub_ = std::make_shared<Subscriber>(this, camera_topic_names[2], image_qos.get_rmw_qos_profile());
        } else {
            img1_cache_sub_ = this->create_subscription<Image>(
                camera_topic_names[0], image_qos,
                [this](const Image::ConstSharedPtr& msg) { cache_image_callback(0, msg); });
            img2_cache_sub_ = this->create_subscription<Image>(
                camera_topic_names[1], image_qos,
                [this](const Image::ConstSharedPtr& msg) { cache_image_callback(1, msg); });
            img3_cache_sub_ = this->create_subscription<Image>(
                camera_topic_names[2], image_qos,
                [this](const Image::ConstSharedPtr& msg) { cache_image_callback(2, msg); });
        }
        
        RCLCPP_INFO(this->get_logger(), "订阅相机话题: %s, %s, %s", 
                   camera_topic_names[0].c_str(), camera_topic_names[1].c_str(), camera_topic_names[2].c_str());

        if (!use_timer_stitch_) {
            // 创建图像接收同步器
            const int sync_queue_size = 20;
            const auto sync_slop = rclcpp::Duration::from_seconds(0.08); // 80ms
            sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
                    SyncPolicy(sync_queue_size),
                    *img1_sub_, *img2_sub_, *img3_sub_);
            sync_->setMaxIntervalDuration(sync_slop);

            // 绑定图像接收回调函数
            sync_->registerCallback(
                std::bind(&JHRos2StitchNode::image_callback, this,
                          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        } else {
            const auto timer_period = std::chrono::milliseconds(50); // 16.67Hz定时器 = 60ms周期
            image_timer_ = this->create_wall_timer(
                timer_period,
                std::bind(&JHRos2StitchNode::timer_stitch_callback, this),
                callback_group_);
            RCLCPP_INFO(this->get_logger(), "启用定时器驱动拼接模式，周期: %ld ms",
                        timer_period.count());
        }

        // 创建定时更新拼缝线的定时器（使用配置中的refresh_time）
        stitch_timer_ = this->create_wall_timer(
            std::chrono::seconds(config.refresh_time),
            std::bind(&JHRos2StitchNode::update_stitch_line, this),
            callback_group_);

        // 初始化拼接图像的发布器（使用配置中的话题名称）
        // 优化QoS配置以减少发布延迟波动：
        // 1. BestEffort: 不等待ACK，允许丢帧（视频流可接受）
        // 2. KeepLast(1): 只保留最新帧，避免队列堵塞
        // 3. Volatile: 不持久化历史消息
        auto stitched_qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile);
        
        stitched_pub_ = this->create_publisher<JHjpgMsg>(config.jhjpg_msg_topic, stitched_qos);
        RCLCPP_INFO(this->get_logger(), "发布拼接图像话题: %s (QoS: BestEffort, KeepLast(1))", 
                   config.jhjpg_msg_topic.c_str());

        // 初始化监控发布超时的定时器（使用配置中的publish_timeout）
        watchdog_pub_timeout = config.publish_timeout;
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::seconds(30), // 每30秒检查一次
            std::bind(&JHRos2StitchNode::check_publish_timeout, this),
            callback_group_);   

        // 初始化船只跟踪的订阅器（使用配置中的话题名称）
        auto trajectory_qos = rclcpp::QoS(rclcpp::KeepLast(3))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = callback_group_;
        visiable_tra_sub_ = this->create_subscription<VisiableTraBatchMsg>(
            config.fus_trajectory_topic,
            trajectory_qos,
            std::bind(&JHRos2StitchNode::visiable_tra_callback, this, std::placeholders::_1),
            sub_options);
        RCLCPP_INFO(this->get_logger(), "订阅船只跟踪话题: %s", config.fus_trajectory_topic.c_str());
        
        // 初始化船只跟踪缓存队列，每个队列对应一个相机的船只跟踪消息
        latest_visiable_tra_cache_.resize(cam_name_to_idx_.size());
        for (size_t i = 0; i < cam_name_to_idx_.size(); i++) {
            latest_visiable_tra_cache_[i] = std::queue<VisiableTra>();
        }

        // 创建GNSS的订阅器（使用配置中的话题名称）
        gnss_sub_ = this->create_subscription<GnssMsg>(
            config.gnss_topic,
            rclcpp::QoS(rclcpp::KeepLast(5))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort),
            std::bind(&JHRos2StitchNode::gnss_callback, this, std::placeholders::_1),
            sub_options);
        RCLCPP_INFO(this->get_logger(), "订阅GNSS话题: %s", config.gnss_topic.c_str());


        // 初始化标志位
        is_first_group_processed_ = false;
        has_received_images_ = false;
        still_detecting = false;

        // 检查是否使用已保存的相机参数（使用配置中的参数）
        bool use_saved_params = config.use_saved_camera_params;
        std::string params_path = config.cache_file;
        
        if (use_saved_params) {
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "📁 use_saved_camera_params=true");
            RCLCPP_INFO(this->get_logger(), "✅ 将加载已保存的相机参数，跳过耗时的特征检测和匹配步骤");
            RCLCPP_INFO(this->get_logger(), "📂 参数文件路径: %s", params_path.c_str());
            RCLCPP_INFO(this->get_logger(), "⏱️  首次处理时间将大幅缩短（约节省80%%时间）");
            RCLCPP_INFO(this->get_logger(), "========================================");
        } else {
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "📁 use_saved_camera_params=false");
            RCLCPP_INFO(this->get_logger(), "🔍 将进行完整的首次处理：特征检测 → 特征匹配 → 相机参数估计");
            RCLCPP_INFO(this->get_logger(), "⏱️  首次处理可能需要较长时间（取决于图像分辨率）");
            RCLCPP_INFO(this->get_logger(), "💾 如需保存参数以加速后续启动，请设置 save_camera_params=true");
            RCLCPP_INFO(this->get_logger(), "========================================");
        }
        
    }

    

private:
    // 同步策略
    using SyncPolicy = sync_policies::ApproximateTime<Image, Image, Image>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    using Subscriber = message_filters::Subscriber<Image>;

    // 拼接器实例
    std::unique_ptr<JHStitcher> stitcher_;

    // ROS相关成员
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::shared_ptr<Subscriber> img1_sub_, img2_sub_, img3_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Subscription<Image>::SharedPtr img1_cache_sub_;
    rclcpp::Subscription<Image>::SharedPtr img2_cache_sub_;
    rclcpp::Subscription<Image>::SharedPtr img3_cache_sub_;
    rclcpp::TimerBase::SharedPtr image_timer_;
    rclcpp::Subscription<VisiableTraBatchMsg>::SharedPtr visiable_tra_sub_;
    rclcpp::Publisher<JHjpgMsg>::SharedPtr stitched_pub_;
    rclcpp::TimerBase::SharedPtr stitch_timer_;
    
    // 相机名称到索引的映射（成员变量）
    std::unordered_map<std::string, int> cam_name_to_idx_;

    // 状态变量
    std::atomic<bool> is_first_group_processed_;
    std::atomic<bool> has_received_images_;
    std::atomic<bool> still_detecting;
    std::array<cv::Mat, 3> latest_images_;
    std::array<rclcpp::Time, 3> latest_stamps_;
    std::array<bool, 3> latest_ready_{{false, false, false}};
    std::mutex latest_images_mutex_;
    const bool use_timer_stitch_ = true;

    std::thread stitch_thread; // 拼縫檢測的类成员变量，而非局部变量

    // 实际定义在 image_stitching_pkg/include/image_stitching_pkg/JHstitcher.hpp 头文件里。
    // 存储投影后的检测框和对应的AIS信息
    std::vector<TrajectoryBoxInfo> trajectory_boxes;

    // 存储船只跟踪消息的缓存队列,分为 cam_name_to_idx 个队列，每个队列对应一个相机的船只跟踪消息
    std::vector<std::queue<VisiableTra>> latest_visiable_tra_cache_;
    std::mutex latest_visiable_tra_cache_mutex_;
    
    // 存储GNSS消息的缓存队列
    std::queue<GnssMsg> latest_gnss_cache_;
    std::mutex latest_gnss_cache_mutex_;
    rclcpp::Subscription<GnssMsg>::SharedPtr gnss_sub_;

     // 新增：监控发布超时的变量
    rclcpp::TimerBase::SharedPtr watchdog_timer_;  // 定时检查的定时器
    std::atomic<int64_t> watchdog_lastpub_time = 0;   // 最后一次发布的时间戳（纳秒）
    int watchdog_pub_timeout;                          // 超时阈值（秒），可通过参数配置


    // 线程信息打印
    std::string thread_info() {
        std::ostringstream thread_str;
        thread_str << "线程ID：" << std::this_thread::get_id();
        return thread_str.str();
    }

    // 图像回调函数
    void image_callback(const Image::ConstSharedPtr& img1,
                       const Image::ConstSharedPtr& img2,
                       const Image::ConstSharedPtr& img3) {

        // ROS图像转换为OpenCV格式
        std::vector<cv::Mat> images; 
        try {
            images.push_back(cv_bridge::toCvShare(img1, "bgr8")->image.clone());
            images.push_back(cv_bridge::toCvShare(img2, "bgr8")->image.clone());
            images.push_back(cv_bridge::toCvShare(img3, "bgr8")->image.clone());
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge转换错误: %s", e.what());
            return;
        }
        if (!is_first_group_processed_) {
            // 首次处理，生成变换数据
            if(processFirstGroup(images)) {
                is_first_group_processed_ = true;
                RCLCPP_INFO(this->get_logger(), "首次处理组成功: %s", thread_info().c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "首次处理组失败: %s", thread_info().c_str());
                is_first_group_processed_ = false;
                return;
            }
        } else {
            // 后续处理（主要耗时）
            processSubsequentGroup(images);
        }
    }

    void cache_image_callback(size_t idx, const Image::ConstSharedPtr& msg) {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
            std::lock_guard<std::mutex> lock(latest_images_mutex_);
            latest_images_[idx] = std::move(frame);
            latest_stamps_[idx] = rclcpp::Time(msg->header.stamp);
            latest_ready_[idx] = true;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge转换错误: %s", e.what());
        }
    }

    void timer_stitch_callback() {
        std::array<cv::Mat, 3> local_images;
        std::array<rclcpp::Time, 3> local_stamps;
        {
            std::lock_guard<std::mutex> lock(latest_images_mutex_);
            if (!latest_ready_[0] || !latest_ready_[1] || !latest_ready_[2]) {
                return;
            }
            local_images = latest_images_;
            local_stamps = latest_stamps_;
        }

        const double diff_ms = (std::max({local_stamps[0], local_stamps[1], local_stamps[2]}) -
                                std::min({local_stamps[0], local_stamps[1], local_stamps[2]})).seconds() * 1000.0;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "三路图像时间戳差: %.2f ms", diff_ms);

        std::vector<cv::Mat> images;
        images.reserve(3);
        for (const auto& img : local_images) {
            if (img.empty()) {
                return;
            }
            images.push_back(img);
        }

        if (!is_first_group_processed_) {
            if (processFirstGroup(images)) {
                is_first_group_processed_ = true;
                RCLCPP_INFO(this->get_logger(), "首次处理组成功: %s", thread_info().c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "首次处理组失败: %s", thread_info().c_str());
                is_first_group_processed_ = false;
            }
        } else {
            processSubsequentGroup(images);
        }
    }

    // 首次处理组
    bool processFirstGroup(std::vector<cv::Mat> images) {
        RCLCPP_INFO(this->get_logger(), "开始首次处理组: %s", thread_info().c_str());

        // 调用拼接器处理
        return stitcher_->processFirstGroupImpl(images);
    }

    // 后续处理组
    void processSubsequentGroup(std::vector<cv::Mat> images) {
        // ========== 性能监控：节点层面的计时 ==========
        auto node_total_start = std::chrono::high_resolution_clock::now();
        
        // 性能监控：检查是否有缝合线检测线程在运行
        bool detecting = still_detecting.load();
        if (detecting) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "⚠️ 缝合线检测线程正在运行，可能影响性能");
        }
        
        // ========== 步骤A：获取轨迹缓存锁 ==========
        auto stepA_start = std::chrono::high_resolution_clock::now();
        std::lock_guard<std::mutex> lock1(latest_visiable_tra_cache_mutex_);
        auto stepA_end = std::chrono::high_resolution_clock::now();
        auto stepA_duration = std::chrono::duration_cast<std::chrono::milliseconds>(stepA_end - stepA_start).count();
        
        // ========== 步骤B：调用拼接器处理（核心耗时） ==========
        auto stepB_start = std::chrono::high_resolution_clock::now();
        cv::Mat stitched_image = stitcher_->processSubsequentGroupImpl(images, latest_visiable_tra_cache_);
        auto stepB_end = std::chrono::high_resolution_clock::now();
        auto stepB_duration = std::chrono::duration_cast<std::chrono::milliseconds>(stepB_end - stepB_start).count();
        
        if (stitched_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "后续处理失败，无法生成拼接图像");
            return;
        }
        
        // ========== 步骤C：获取轨迹框 ==========
        auto stepC_start = std::chrono::high_resolution_clock::now();
        const std::vector<TrajectoryBoxInfo>& trajectory_boxes = stitcher_->getTrajectoryBoxes();
        auto stepC_end = std::chrono::high_resolution_clock::now();
        auto stepC_duration = std::chrono::duration_cast<std::chrono::milliseconds>(stepC_end - stepC_start).count();
        
        // ========== 步骤D：获取GNSS缓存锁 ==========
        auto stepD_start = std::chrono::high_resolution_clock::now();
        std::lock_guard<std::mutex> lock2(latest_gnss_cache_mutex_);
        auto stepD_end = std::chrono::high_resolution_clock::now();
        auto stepD_duration = std::chrono::duration_cast<std::chrono::milliseconds>(stepD_end - stepD_start).count();
        
        // ========== 步骤E：发布拼接图像 ==========
        auto stepE_start = std::chrono::high_resolution_clock::now();
        publishStitchedImage(stitched_image, trajectory_boxes, latest_gnss_cache_.front());
        auto stepE_end = std::chrono::high_resolution_clock::now();
        auto stepE_duration = std::chrono::duration_cast<std::chrono::milliseconds>(stepE_end - stepE_start).count();
        
        // ========== 节点层面总耗时 ==========
        auto node_total_end = std::chrono::high_resolution_clock::now();
        auto node_total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(node_total_end - node_total_start).count();
        
        // ========== 性能报告（仅在总耗时>5ms时输出） ==========
        if (node_total_duration > 5) {
            // RCLCPP_INFO(this->get_logger(), "⚠️ [节点层性能] 总耗时: %ld ms", node_total_duration);
            // RCLCPP_INFO(this->get_logger(), 
            //     "  步骤A-获取轨迹锁: %ld ms | 步骤B-拼接处理: %ld ms | 步骤C-获取轨迹框: %ld ms | 步骤D-获取GNSS锁: %ld ms | 步骤E-发布图像: %ld ms",
            //     stepA_duration, stepB_duration, stepC_duration, stepD_duration, stepE_duration);
            // 计算每次处理的间隔时间
            static int64_t last_process_time = 0;
            int64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            if (last_process_time != 0) {
                int64_t interval = current_time - last_process_time;
                // RCLCPP_INFO(this->get_logger(), "  距上次处理间隔: %ld ms", interval);
            }
            last_process_time = current_time;
        }
    }

    // 定时更新拼缝线
    void update_stitch_line() {
        if(is_first_group_processed_ && !still_detecting && !stitch_thread.joinable()) {
            RCLCPP_INFO(this->get_logger(), "🔄 开始更新拼缝线（此过程可能影响主处理线程性能）: %s", thread_info().c_str());
            still_detecting = true;
            

            // 启动独立线程执行耗时操作
            stitch_thread = std::thread([this]() {
                // stitcher_->detectStitchLine(); // 内部包含 seam_finder->find
                try {
                    auto start_time = std::chrono::high_resolution_clock::now();
                    stitcher_->detectStitchLine();
                    auto end_time = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                    RCLCPP_INFO(this->get_logger(), "✅ 缝合线检测完成，耗时: %ld 毫秒", duration);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "子线程异常: %s", e.what());
                }
                still_detecting = false;
            });
            // 可选：用单独的线程管理 join，避免阻塞
            std::thread(&std::thread::join, &stitch_thread).detach();

            // 更新拼缝线
                // stitcher_->detectStitchLine();
                // still_detecting = false;
                      
        } else {
            if (!is_first_group_processed_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                    "由于首次处理组尚未完成，跳过这一次拼缝线更新: %s", thread_info().c_str());
            }
            // still_detecting=true 说明上一次检测还在进行
        }
    }

    // 船只跟踪回调
    // 将获得的跟踪消息保存到一个缓存队列中，在拼接图像发布时，将跟踪消息附带在jh_msg里也发布出去
    void visiable_tra_callback(const VisiableTraBatchMsg::SharedPtr msg_batch) {
        if (!msg_batch) {
            RCLCPP_ERROR(this->get_logger(), "收到空的船只跟踪消息");
            return;
        }
        
        // 按照相机名称分别保存到对应的缓存队列中
        std::lock_guard<std::mutex> lock(latest_visiable_tra_cache_mutex_);
        
        // 清空每个相机的队列（保留最新的批次消息）
        for (auto& queue : latest_visiable_tra_cache_) {
            while (!queue.empty()) {
                queue.pop();
            }
        }

        // 遍历 msg_batch 中的每个 VisiableTraMsg，
        // 然后将 VisiableTraMsg 格式转化为 VisiableTra 格式，
        // 按照相机名称分别保存到对应的缓存队列中
        for (const auto& msg : msg_batch->visiable_tra_list) {
            // 容错处理：检查相机名称是否在映射中
            auto it = cam_name_to_idx_.find(msg.camera_name);
            if (it != cam_name_to_idx_.end()) {
                int cam_idx = it->second;  // 获取相机索引
                
                // 进行格式转换
                VisiableTra vt;
                vt.camera_name = msg.camera_name;
                // 将 builtin_interfaces::Time 转换为纳秒（int64_t）
                vt.timestamp_ns = static_cast<int64_t>(msg.timestamp.sec) * 1000000000LL + 
                                  static_cast<int64_t>(msg.timestamp.nanosec);
                vt.ais_or_not = msg.ais;
                vt.mmsi = msg.mmsi;
                vt.ship_type = msg.ship_type;
                vt.sog = msg.sog;
                vt.cog = msg.cog;
                vt.latitude = msg.lat;
                vt.longitude = msg.lon;
                vt.box_x1 = msg.box_x1;
                vt.box_y1 = msg.box_y1;
                vt.box_x2 = msg.box_x2;
                vt.box_y2 = msg.box_y2;
                
                // 关键步骤：将转换后的消息push到对应相机的队列中
                latest_visiable_tra_cache_[cam_idx].push(vt);
                
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "收到未知相机名称的轨迹消息: %s", msg.camera_name.c_str());
            }
        }
    }

    // GNSS回调
    void gnss_callback(const GnssMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(latest_gnss_cache_mutex_);
        latest_gnss_cache_.push(*msg);
        while (latest_gnss_cache_.size() > 5) {
            latest_gnss_cache_.pop();
        }
    }
    // 发布拼接图像
    void publishStitchedImage(const cv::Mat& stitched_image, 
                             const std::vector<TrajectoryBoxInfo>& trajectory_boxes,
                             const GnssMsg& gnss_msg) {
        auto pub_total_start = std::chrono::high_resolution_clock::now();
        
        if (stitched_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "拼接图像为空，无法发布");
            return;
        }
        
        // ========== 发布步骤1：图像类型转换 ==========
        auto pub_step1_start = std::chrono::high_resolution_clock::now();
        cv::Mat stitched_8u;
        if (stitched_image.type() == CV_16SC3) {
            cv::Mat stitched_32s;
            stitched_image.convertTo(stitched_32s, CV_32SC3);
            cv::Mat stitched_32u;
            add(stitched_32s, cv::Scalar(32768, 32768, 32768), stitched_32u, cv::noArray(), CV_32SC3);
            normalize(stitched_32u, stitched_8u, 0, 255, cv::NORM_MINMAX, CV_8UC3);
        } else if (stitched_image.type() == CV_32FC3) {
            normalize(stitched_image, stitched_8u, 0, 255, cv::NORM_MINMAX, CV_8UC3);
        } else {
            stitched_image.convertTo(stitched_8u, CV_8UC3);
        }
        // =================================================== DEBUG ===================================================
        // 只为了好看：
        // stitched_8u = stitched_8u(cv::Rect(0, 0, stitched_image.cols, stitched_image.rows*6/7));
        // =================================================== DEBUG ===================================================
        auto pub_step1_end = std::chrono::high_resolution_clock::now();
        auto pub_step1_duration = std::chrono::duration_cast<std::chrono::milliseconds>(pub_step1_end - pub_step1_start).count();

        // ========== 发布步骤2：构建消息头 ==========
        auto pub_step2_start = std::chrono::high_resolution_clock::now();
        JHjpgMsg jh_msg;
        mycustface::msg::Header custom_header;
        custom_header.timestamp = this->get_clock()->now().nanoseconds();
        custom_header.id = "stitched_image_";
        jh_msg.mheader = custom_header;
        jh_msg.index = 1;
        auto pub_step2_end = std::chrono::high_resolution_clock::now();
        auto pub_step2_duration = std::chrono::duration_cast<std::chrono::milliseconds>(pub_step2_end - pub_step2_start).count();

        // ========== 发布步骤3：构建JSON消息 ==========
        auto pub_step3_start = std::chrono::high_resolution_clock::now();
        JHmessagetoJson(jh_msg.message, trajectory_boxes, gnss_msg);
        auto pub_step3_end = std::chrono::high_resolution_clock::now();
        auto pub_step3_duration = std::chrono::duration_cast<std::chrono::milliseconds>(pub_step3_end - pub_step3_start).count();

        // ========== 发布步骤4：JPEG编码 ==========
        auto pub_step4_start = std::chrono::high_resolution_clock::now();
        std::vector<int> params;
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(75);  // 从90降到75，减少编码时间30-40%，图像质量影响很小
        
        // 记录编码前的图像信息
        size_t image_pixels = stitched_8u.rows * stitched_8u.cols;
        
        bool success = cv::imencode(".jpg", stitched_8u, jh_msg.picture, params);
        auto pub_step4_end = std::chrono::high_resolution_clock::now();
        auto pub_step4_duration = std::chrono::duration_cast<std::chrono::milliseconds>(pub_step4_end - pub_step4_start).count();
        
        size_t jpeg_size = 0;
        if (success) {
            jh_msg.size = jh_msg.picture.size();
            jpeg_size = jh_msg.picture.size();
        }

        // ========== 发布步骤5：ROS2发布 ==========
        auto pub_step5_start = std::chrono::high_resolution_clock::now();
        watchdog_lastpub_time = this->get_clock()->now().nanoseconds();
        
        // 记录消息大小
        size_t total_msg_size = jh_msg.picture.size() + jh_msg.message.size();
        
        stitched_pub_->publish(jh_msg);
        auto pub_step5_end = std::chrono::high_resolution_clock::now();
        auto pub_step5_duration = std::chrono::duration_cast<std::chrono::milliseconds>(pub_step5_end - pub_step5_start).count();
        

        // ========== 发布总耗时 ==========
        auto pub_total_end = std::chrono::high_resolution_clock::now();
        auto pub_total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(pub_total_end - pub_total_start).count();
        
        // 总是输出详细的发布性能信息（因为这是主要瓶颈）
        // if (pub_total_duration > 1) {
        //     RCLCPP_INFO(this->get_logger(), 
        //         "📤 [发布性能详细] 总耗时: %ldms", pub_total_duration);
        //     RCLCPP_INFO(this->get_logger(),
        //         "  发布步骤1-图像转换: %ldms | 步骤2-构建头: %ldms | 步骤3-JSON: %ldms | 步骤4-JPEG编码: %ldms | 步骤5-ROS发布: %ldms",
        //         pub_step1_duration, pub_step2_duration, pub_step3_duration, 
        //         pub_step4_duration, pub_step5_duration);
        // }
    }

    // 将轨迹框信息 和 此刻的GNSS消息 转换为JSON格式
    void JHmessagetoJson(std::string& message, const std::vector<TrajectoryBoxInfo>& trajectory_boxes, const GnssMsg& gnss_msg)
    {
        // 检测trajectory_boxes和gnss_msg是否为空
        if (trajectory_boxes.empty() || gnss_msg.latitude == 0 || gnss_msg.longitude == 0) {
            // RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "轨迹框或GNSS消息为空，无法转换为JSON");
            message = "{\"gnss\":{},\"trajectories\":[]}";  // 返回空JSON
            return;
        }

        // 1. 构建根JSON对象
        json root;

        // 2. 添加GNSS信息
        root["gnss"] = {
            {"latitude", gnss_msg.latitude},
            {"longitude", gnss_msg.longitude},
            {"horizontal_orientation", gnss_msg.horizontal_orientation},
            {"vertical_orientation", gnss_msg.vertical_orientation},
            {"camera_height", gnss_msg.camera_height}
        };

        // 3. 添加轨迹框数组
        root["trajectories"] = json::array();
        for (const auto& traj : trajectory_boxes) {
            root["trajectories"].push_back({
                {"message_type", traj.message_type},
                {"ship_type", traj.ship_type},
                {"x1", traj.top_left.x},
                {"y1", traj.top_left.y},
                {"x2", traj.bottom_right.x},
                {"y2", traj.bottom_right.y},
                {"mmsi", traj.mmsi},
                {"sog", traj.sog},
                {"cog", traj.cog},
                {"lat", traj.lat},
                {"lon", traj.lon}
            });
        }

        // 4. 将JSON对象转换为字符串
        message = root.dump();
    }


    void check_publish_timeout() {
        // cout<<"进了check_publish_timeout"<<endl;
        // 若从未发布过消息（初始状态），直接返回
        if (watchdog_lastpub_time == 0) {
            RCLCPP_DEBUG(this->get_logger(), "尚未收到任何发布消息，等待中...");
            return;
        }

        // 计算当前时间与最后发布时间的差值（秒）
        int64_t current_time = this->get_clock()->now().nanoseconds();
        int64_t elapsed_ns = current_time - watchdog_lastpub_time;
        double elapsed_sec = static_cast<double>(elapsed_ns) / 1e9;  // 转换为秒
        // cout<<"current_time = "<<current_time<<endl;
        // cout<<"elapsed_ns = "<<elapsed_ns<<endl;
        // cout<<"elapsed_sec = "<<elapsed_sec<<endl;
        // cout<<"watchdog_pub_timeout = "<<watchdog_pub_timeout<<endl;
        // 若超时，终止节点
        if (elapsed_sec > watchdog_pub_timeout) {
            RCLCPP_FATAL(this->get_logger(), 
                "image_topic_all 话题已超过 %.2f 秒未发布消息，终止节点！", 
                elapsed_sec);
            rclcpp::shutdown();  // 终止整个ROS 2节点
        } else {
            RCLCPP_DEBUG(this->get_logger(), 
                "距离上次发布已过去 %.2f 秒（超时阈值: %d 秒）", 
                elapsed_sec, watchdog_pub_timeout);
        }
    }

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JHRos2StitchNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
