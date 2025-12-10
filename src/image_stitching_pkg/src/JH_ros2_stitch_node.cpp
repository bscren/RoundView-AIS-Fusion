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

// 服务头文件
#include "detect_interfaces/srv/get_camera_params.hpp"

// 船只跟踪消息体文件
#include "marnav_interfaces/msg/visiable_tra.hpp"
#include "marnav_interfaces/msg/visiable_tra_batch.hpp"
#include "marnav_interfaces/msg/gnss.hpp"


#include "JHstitcher.hpp"
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <atomic>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std;
using namespace sensor_msgs::msg;
using namespace message_filters;

// 消息类型别名和服务类型别名
using JHjpgMsg = mycustface::msg::JHjpg;
using GetCameraParamsSrv = detect_interfaces::srv::GetCameraParams;
using VisiableTraMsg = marnav_interfaces::msg::VisiableTra;
using VisiableTraBatchMsg = marnav_interfaces::msg::VisiableTraBatch;
using GnssMsg = marnav_interfaces::msg::Gnss;

class JHRos2StitchNode : public rclcpp::Node {
public:
    JHRos2StitchNode() : Node("topic_stitch") {
        // 创建可重入回调组
        callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        
        // 相机名称到索引的映射
        cam_name_to_idx_ = {
            {"rtsp_image_0", 0},
            {"rtsp_image_1", 1},
            {"rtsp_image_2", 2}
        };            
        
        stitcher_ = std::make_unique<JHStitcher>(
            cam_name_to_idx_, //相机名称到索引的映射
            this->declare_parameter("refresh_time", 2),
            this->declare_parameter("min_keypoints", 50),
            this->declare_parameter("min_confidence", 0.4),
            this->declare_parameter("min_inliers", 50),
            this->declare_parameter("max_focal_variance", 50000.0),
            this->declare_parameter("y_tolerance", 200.0),
            this->declare_parameter("roi_threshold", 0.95f),
            this->declare_parameter("scale", 0.75),
            this->declare_parameter("cropornot",true),
            this->declare_parameter("drawboxornot",true),
            this->declare_parameter("save_CameraParams",false),
            this->declare_parameter("save_CameraParams_path","/home/tl/RV/src/image_stitching_pkg/config/CameraParams.yaml"),
            this->declare_parameter("use_saved_CameraParams",true),
            this->declare_parameter("FOV_hor",105.0),
            this->declare_parameter("FOV_ver",57.0)
        );

        // 在节点中检查参数
        bool use_sim_time;
        this->get_parameter("use_sim_time", use_sim_time);
        if (use_sim_time) {
        RCLCPP_INFO(this->get_logger(), "使用仿真时间");
        } else {
        RCLCPP_INFO(this->get_logger(), "使用实际系统时间");
        }
        
        // 初始化订阅器（配置QoS策略以匹配发布者）
        // 图像话题通常使用 BEST_EFFORT 可靠性策略
        auto image_qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile);
        
        //  =================================================== DEBUG ===================================================
        img1_sub_ = std::make_shared<Subscriber>(this, "/camera_image_topic_0", image_qos.get_rmw_qos_profile());
        img2_sub_ = std::make_shared<Subscriber>(this, "/camera_image_topic_1", image_qos.get_rmw_qos_profile());
        img3_sub_ = std::make_shared<Subscriber>(this, "/camera_image_topic_2", image_qos.get_rmw_qos_profile());
        // =================================================== DEBUG ===================================================

        // 创建图像接收同步器
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
                SyncPolicy(100000000),  // 最大时间差参数（纳秒）
                *img1_sub_, *img2_sub_, *img3_sub_);

        // 绑定图像接收回调函数
        sync_->registerCallback(
            std::bind(&JHRos2StitchNode::image_callback, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // 创建定时更新拼缝线的定时器
        stitch_timer_ = this->create_wall_timer(
            std::chrono::seconds(this->get_parameter("refresh_time").as_int()),
            std::bind(&JHRos2StitchNode::update_stitch_line, this),
            callback_group_);

        // 初始化拼接图像的发布器
        stitched_pub_ = this->create_publisher<JHjpgMsg>("image_topic_all", 10);

        // 初始化监控发布超时的定时器
        watchdog_pub_timeout = this->declare_parameter("publish_timeout", 5); // 默认5秒
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::seconds(10), // 每30秒检查一次
            std::bind(&JHRos2StitchNode::check_publish_timeout, this),
            callback_group_);   

        // 初始化船只跟踪的订阅器（使用与发布者相同的QoS策略）
        auto qos = rclcpp::QoS(rclcpp::KeepLast(3))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = callback_group_;
        visiable_tra_sub_ = this->create_subscription<VisiableTraBatchMsg>(
            "/fus_trajectory_topic",
            qos,
            std::bind(&JHRos2StitchNode::visiable_tra_callback, this, std::placeholders::_1),
            sub_options);
        
        // 初始化船只跟踪缓存队列，每个队列对应一个相机的船只跟踪消息
        latest_visiable_tra_cache_.resize(cam_name_to_idx_.size());
        for (size_t i = 0; i < cam_name_to_idx_.size(); i++) {
            latest_visiable_tra_cache_[i] = std::queue<VisiableTra>();
        }

        // 创建GNSS的订阅器
        gnss_sub_ = this->create_subscription<GnssMsg>(
            "/gnss_topic",
            rclcpp::QoS(rclcpp::KeepLast(5))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort),
            std::bind(&JHRos2StitchNode::gnss_callback, this, std::placeholders::_1),
            sub_options);

        // 创建获取相机参数服务
        get_camera_params_srv_ = this->create_service<GetCameraParamsSrv>(
            "get_camera_params",
            std::bind(&JHRos2StitchNode::getCameraParamsCallback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

             // 新增：读取超时阈值参数（默认5秒，可在launch或参数文件中配置）
    // watchdog_pub_timeout = this->declare_parameter("publish_timeout", 5);

    // // 新增：创建看门狗定时器，每秒检查一次发布状态
    // watchdog_timer_ = this->create_wall_timer(
    //     std::chrono::seconds(1),  // 1秒检查一次
    //     std::bind(&JHRos2StitchNode::check_publish_timeout, this),
    //     callback_group_  // 使用已有的可重入回调组
    // );
        // 初始化标志位
        is_first_group_processed_ = false;
        has_received_images_ = false;
        still_detecting = false;

        // 检查是否使用已保存的相机参数
        bool use_saved_params = this->get_parameter("use_saved_CameraParams").as_bool();
        std::string params_path = this->get_parameter("save_CameraParams_path").as_string();
        
        if (use_saved_params) {
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "📁 use_saved_CameraParams=true");
            RCLCPP_INFO(this->get_logger(), "✅ 将加载已保存的相机参数，跳过耗时的特征检测和匹配步骤");
            RCLCPP_INFO(this->get_logger(), "📂 参数文件路径: %s", params_path.c_str());
            RCLCPP_INFO(this->get_logger(), "⏱️  首次处理时间将大幅缩短（约节省80%%时间）");
            RCLCPP_INFO(this->get_logger(), "========================================");
        } else {
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "📁 use_saved_CameraParams=false");
            RCLCPP_INFO(this->get_logger(), "🔍 将进行完整的首次处理：特征检测 → 特征匹配 → 相机参数估计");
            RCLCPP_INFO(this->get_logger(), "⏱️  首次处理可能需要较长时间（取决于图像分辨率）");
            RCLCPP_INFO(this->get_logger(), "💾 如需保存参数以加速后续启动，请设置 save_CameraParams=true");
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
    rclcpp::Subscription<VisiableTraBatchMsg>::SharedPtr visiable_tra_sub_;
    rclcpp::Publisher<JHjpgMsg>::SharedPtr stitched_pub_;
    rclcpp::Service<GetCameraParamsSrv>::SharedPtr get_camera_params_srv_;
    rclcpp::TimerBase::SharedPtr stitch_timer_;
    
    // 相机名称到索引的映射（成员变量）
    std::unordered_map<std::string, int> cam_name_to_idx_;

    // 状态变量
    std::atomic<bool> is_first_group_processed_;
    std::atomic<bool> has_received_images_;
    std::atomic<bool> still_detecting;

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

        // 转换ROS图像消息为OpenCV格式
        std::vector<cv::Mat> images; 
        try {
            // cout<<"进了image callback ing"<<endl;
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
            // 后续处理，使用已有变换数据
            auto start_time = std::chrono::high_resolution_clock::now();
            processSubsequentGroup(images);
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            // RCLCPP_INFO(this->get_logger(), "PSG处理耗时: %ld 毫秒", duration);
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
        // cout<<"又进了process Subsequent Group"<<endl;
        // 调用拼接器处理时，传入检测数据（加锁保护）
        std::lock_guard<std::mutex> lock1(latest_visiable_tra_cache_mutex_); // 确保读取visiable_tra_cache_时线程安全
        cv::Mat stitched_image = stitcher_->processSubsequentGroupImpl(images,latest_visiable_tra_cache_);
        if (stitched_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "后续处理失败，无法生成拼接图像");
            return;
        }
        
        // 获取投影后的轨迹框（通过拼接器的getter接口）
        const std::vector<TrajectoryBoxInfo>& trajectory_boxes = stitcher_->getTrajectoryBoxes();
        // 获取最新的GNSS消息
        std::lock_guard<std::mutex> lock2(latest_gnss_cache_mutex_);
        // 发布拼接图像（同时传入检测框和轨迹框和GNSS消息）
        publishStitchedImage(stitched_image, trajectory_boxes, latest_gnss_cache_.front());
    }

    // 定时更新拼缝线
    void update_stitch_line() {
        if(is_first_group_processed_ && !still_detecting && !stitch_thread.joinable()) {
            RCLCPP_INFO(this->get_logger(), "开始更新拼缝线: %s", thread_info().c_str());
            still_detecting = true;
            

            // 启动独立线程执行耗时操作
            stitch_thread = std::thread([this]() {
                // stitcher_->detectStitchLine(); // 内部包含 seam_finder->find
                try {
                    stitcher_->detectStitchLine();
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
            RCLCPP_WARN(this->get_logger(), "由于首次处理组尚未完成，跳过这一次拼缝线更新: %s", thread_info().c_str());
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
        if (stitched_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "拼接图像为空，无法发布");
            return;
        }
        // 拼接图像尺寸打印
        cout<<"stitched_image.size() = "<<stitched_image.size()<<endl;
        // 0. 转换为8位图像
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
        stitched_8u = stitched_8u(cv::Rect(0,0,stitched_image.cols,stitched_image.rows*6/7));
        // =================================================== DEBUG ===================================================

        // 1. 构建自定义消息对象
        JHjpgMsg jh_msg;

        // 2. 填充header1字段（自定义Header）
        mycustface::msg::Header custom_header;
        // timestamp用当前时间的纳秒数（转换为long类型）
        custom_header.timestamp = this->get_clock()->now().nanoseconds();
        // cout<<"custom_header.timestamp = "<<custom_header.timestamp<<endl;
        // rclcpp::Time t(custom_header.timestamp);
        // RCLCPP_INFO(this->get_logger(), "转换为rclcpp::Time: %u 秒, %u 纳秒", t.seconds(), t.nanoseconds());
        custom_header.id = "stitched_image_";  // 自定义ID
        jh_msg.mheader = custom_header;

        // 3. 填充index字段（递增计数器）
        jh_msg.index = 1; //msg_index_++;

        // 4. 填充message字段（描述信息）
        // jh_msg.message = "stitched image from 3 cameras";
        JHmessagetoJson(jh_msg.message, trajectory_boxes, gnss_msg);

        std::vector<int> params;
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(90); // 设置 JPEG 质量为 95%
        bool success = cv::imencode(".jpg", stitched_8u, jh_msg.picture, params);
        // cout<<"msg->picture.size() = "<<jh_msg.picture.size()<<endl;
            if (success){                
                jh_msg.size = jh_msg.picture.size();
                // jh_msg.index = inx++;
                // RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message1.index);
            }


            // 发布成功后，记录当前时间戳（纳秒）
        watchdog_lastpub_time = this->get_clock()->now().nanoseconds();

        // 6. 发布消息
        stitched_pub_->publish(jh_msg);
        RCLCPP_INFO(this->get_logger(), "发布JHjpg消息，大小: %u 字节，序号: %u\n", 
                jh_msg.size, jh_msg.index);
    }

    // 将轨迹框信息 和 此刻的GNSS消息 转换为JSON格式
    void JHmessagetoJson(std::string& message, const std::vector<TrajectoryBoxInfo>& trajectory_boxes, const GnssMsg& gnss_msg)
    {
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
                {"class_name", traj.class_name},
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

        // // 构建包含检测框和轨迹框的 JSON 对象
        // message = "{\"gnss\":{";  
        // message += "\"latitude\":" + std::to_string(gnss_msg.latitude) + ",";
        // message += "\"longitude\":" + std::to_string(gnss_msg.longitude) + ",";
        // message += "\"horizontal_orientation\":" + std::to_string(gnss_msg.horizontal_orientation) + ",";
        // message += "\"vertical_orientation\":" + std::to_string(gnss_msg.vertical_orientation) + ",";
        // message += "\"camera_height\":" + std::to_string(gnss_msg.camera_height) + "},";
        // message += "},";

        // // 2. 添加轨迹框数组（包含AIS信息）
        // message += "\"trajectories\":[";
        // if (!trajectory_boxes.empty()) {
        //     for (size_t i = 0; i < trajectory_boxes.size(); i++) {
        //         const auto& traj = trajectory_boxes[i];
        //         message += "{";
        //         message += "\"class_name\":\"" + traj.class_name + "\",";
        //         message += "\"x1\":" + std::to_string(traj.top_left.x) + ",";
        //         message += "\"y1\":" + std::to_string(traj.top_left.y) + ",";
        //         message += "\"x2\":" + std::to_string(traj.bottom_right.x) + ",";
        //         message += "\"y2\":" + std::to_string(traj.bottom_right.y) + ",";
        //         message += "\"mmsi\":" + std::to_string(traj.mmsi) + ",";
        //         message += "\"sog\":" + std::to_string(traj.sog) + ",";
        //         message += "\"cog\":" + std::to_string(traj.cog) + ",";
        //         message += "\"lat\":" + std::to_string(traj.lat) + ",";
        //         message += "\"lon\":" + std::to_string(traj.lon);
        //         message += "}";
        //         if (i != trajectory_boxes.size() - 1) {
        //             message += ",";
        //         }
        //     }
        // }
        // message += "]";
        
        // message += "}"; // 结束 JSON 对象
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

    // 获取相机参数服务回调
    void getCameraParamsCallback(
        const std::shared_ptr<detect_interfaces::srv::GetCameraParams::Request> request,
        std::shared_ptr<detect_interfaces::srv::GetCameraParams::Response> response)
    {
        // 查找相机索引
        auto cam_name = request->camera_name;
        int cam_idx = -1;
        if (stitcher_) {
            const auto& cam_map = stitcher_->getCamNameToIdx();
            if (cam_map.count(cam_name)) {
                cam_idx = cam_map.at(cam_name);
            }
        }
        if (cam_idx < 0 || !stitcher_) {
            response->success = false;
            return;
        }

        // 获取 TransformationData
        const auto& data = stitcher_->getTransformationData();
        if (cam_idx >= data.cameras.size()) {
            response->success = false;
            return;
        }
        const auto& cam = data.cameras[cam_idx];
        response->fov_hor = stitcher_->getFOVHor(); // 单位: degree
        response->fov_ver = stitcher_->getFOVVer(); // 单位: degree
        response->success = true;
        response->focal = cam.focal;
        response->aspect = cam.aspect;
        response->ppx = cam.ppx;
        response->ppy = cam.ppy;
        for (int i = 0; i < 9; ++i) response->rotate_matrix[i] = cam.R.at<float>(i / 3, i % 3);
        for (int i = 0; i < 3; ++i) response->transport_matrix[i] = cam.t.at<double>(i, 0);
        cv::Mat K = cam.K();
        for (int i = 0; i < 9; ++i) response->k_matrix[i] = K.at<double>(i / 3, i % 3);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JHRos2StitchNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
