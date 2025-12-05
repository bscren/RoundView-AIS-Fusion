#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

// YOLO检测消息头文件
#include "detect_interfaces/msg/multi_detections_cameras.hpp"
#include "detect_interfaces/msg/multi_detection_results.hpp"
#include "detect_interfaces/msg/detection_result.hpp"

// 自定义消息头文件
#include "mycustface/msg/j_hjpg.hpp"
#include "mycustface/msg/header.hpp"

// 服务头文件
#include "detect_interfaces/srv/get_camera_params.hpp"

// 船只跟踪消息体文件
#include "marnav_interfaces/msg/visiable_tra.hpp"
#include "marnav_interfaces/msg/visiable_tra_batch.hpp"

#include "JHstitcher.hpp"
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <atomic>

using namespace std;
using namespace sensor_msgs::msg;
using namespace message_filters;

// 消息类型别名和服务类型别名
using MultiDetectionsCamerasMsg = detect_interfaces::msg::MultiDetectionsCameras;
using MultiDetectionResultsMsg = detect_interfaces::msg::MultiDetectionResults;
using JHjpgMsg = mycustface::msg::JHjpg;
using GetCameraParamsSrv = detect_interfaces::srv::GetCameraParams;
using VisiableTraMsg = marnav_interfaces::msg::VisiableTra;
using VisiableTraBatchMsg = marnav_interfaces::msg::VisiableTraBatch;

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
            this->declare_parameter("detect_confidence", 0.3f),
            this->declare_parameter("iou_threshold", 0.5f),
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
        
        // 初始化订阅器
        img1_sub_ = std::make_shared<Subscriber>(this, "/rtsp_image_0");
        img2_sub_ = std::make_shared<Subscriber>(this, "/rtsp_image_1");
        img3_sub_ = std::make_shared<Subscriber>(this, "/rtsp_image_2");

        // 初始化YOLO检测结果订阅器
        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_;
        yolo_sub_ = this->create_subscription<MultiDetectionsCamerasMsg>(
            "/yolo/detection_results",
            50,
            std::bind(&JHRos2StitchNode::yolo_detection_callback, this, std::placeholders::_1),
            options);

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
            latest_visiable_tra_cache_[i] = std::queue<VisiableTraMsg>();
        }
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
    rclcpp::Subscription<MultiDetectionsCamerasMsg>::SharedPtr yolo_sub_;
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

    // 检测结果存储（按相机名称分类）的：锁类和对应保护对象
    std::mutex detections_mutex_;  // 保护检测结果的线程安全
    // DetectionResult 是一个结构体（或者类），用于存储单个目标检测的结果。
    // 一般该结构体包含如下字段：
    // - std::string class_name;       // 检测到的类别名称
    // - float confidence;             // 置信度
    // - float x1, y1, x2, y2;         // 检测框的左上角与右下角的坐标
    // 实际定义可能在 detection_result.h 或类似头文件里。
    std::unordered_map<std::string, std::vector<DetectionResult>> latest_detections_;

    std::thread stitch_thread; // 拼縫檢測的类成员变量，而非局部变量


     // 存储筛选后的检测框（私有成员，仅内部修改）
    std::vector<BoxInfo> filtered_boxes;

    // 存储船只跟踪消息的缓存队列,分为 cam_name_to_idx 个队列，每个队列对应一个相机的船只跟踪消息
    std::vector<std::queue<VisiableTraMsg>> latest_visiable_tra_cache_;
    std::mutex latest_visiable_tra_cache_mutex_;
    

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
        std::lock_guard<std::mutex> lock1(detections_mutex_); // 确保读取latest_detections_时线程安全
        std::lock_guard<std::mutex> lock2(latest_visiable_tra_cache_mutex_); // 确保读取visiable_tra_cache_时线程安全
        cv::Mat stitched_image = stitcher_->processSubsequentGroupImpl(images,latest_detections_);
        if (stitched_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "后续处理失败，无法生成拼接图像");
            return;
        }

        // 获取筛选后的检测框（通过拼接器的getter接口）
        const std::vector<BoxInfo>& filtered_boxes = stitcher_->getFilteredBoxes();
        

        // 发布拼接图像（同时传入检测框）
        publishStitchedImage(stitched_image, filtered_boxes);
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

    // YOLO检测结果回调
    void yolo_detection_callback(const MultiDetectionsCamerasMsg::SharedPtr msg) {
        if (!msg) {
            RCLCPP_ERROR(this->get_logger(), "收到空的YOLO检测结果消息");
            return;
        }
        else {
            // RCLCPP_INFO(this->get_logger(), "yolo_detection_callback收到YOLO检测结果消息，包含 %zu 个相机的检测结果", msg->multicamdetections.size());
        }
        std::lock_guard<std::mutex> lock(detections_mutex_);
        latest_detections_.clear();

        // 转换检测结果格式
        for (const auto& cam_detections : msg->multicamdetections) {
            std::vector<DetectionResult> results;
            for (const auto& det : cam_detections.results) {
                DetectionResult dr;
                dr.class_name = det.class_name;
                dr.confidence = det.confidence;
                dr.x1 = det.x1;
                dr.y1 = det.y1;
                dr.x2 = det.x2;
                dr.y2 = det.y2;
                results.push_back(dr);
            }
            latest_detections_[cam_detections.camera_name] = results;
        }
    }

    // 船只跟踪回调
    // 将获得的跟踪消息保存到一个缓存队列中，在拼接图像发布时，将跟踪消息附带在jh_msg里也发布出去
    void visiable_tra_callback(const VisiableTraBatchMsg::SharedPtr msg_batch) {
        if (!msg_batch) {
            RCLCPP_ERROR(this->get_logger(), "收到空的船只跟踪消息");
            return;
        }
        else {
            // 按照相机名称分别保存到对应的缓存队列中
            std::lock_guard<std::mutex> lock(latest_visiable_tra_cache_mutex_);
            latest_visiable_tra_cache_.clear();

            // 遍历 msg_batch 中的每个 VisiableTraMsg，按照相机名称分别保存到对应的缓存队列中
            for (const auto& msg : msg_batch->visiable_tra_list) {
                // 容错处理：检查相机名称是否在映射中
                auto it = cam_name_to_idx_.find(msg.camera_name);
                if (it != cam_name_to_idx_.end()) {
                    latest_visiable_tra_cache_[it->second].push(msg);
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "收到未知相机名称的轨迹消息: %s", msg.camera_name.c_str());
                }
            }

        }
    }
    // 发布拼接图像
    void publishStitchedImage(const cv::Mat& stitched_image, const std::vector<BoxInfo>& filtered_boxes) {
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
        
// 只为了好看：

        stitched_8u = stitched_8u(cv::Rect(0,0,stitched_image.cols,stitched_image.rows*6/7));

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
        JHmessagetoJson(jh_msg.message,filtered_boxes);

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

    void JHmessagetoJson(std::string& message, const std::vector<BoxInfo>& boxes)
    {
        if (boxes.empty()) {
        message = "[]";
        // cout<<"这一帧JHmessagetoJson没找到"<<endl;
        return;
        }
        else {
        // cout<<"这一帧JHmessagetoJson找到了"<<endl;
        }
        // 构建 JSON 字符串
        message += "[";// 初始化 JSON 字符串，用 [ 开头，因为最终要构造 JSON 数组（多个检测结果）
        // std::string jsonStr = "["; 
        for (size_t i = 0; i < boxes.size(); i++) {
                const auto& box = boxes[i];
                //构建单个检测框的JSON对象
                message += "{";
                message += "\"class_name\":\"" + box.class_name + "\",";
                message += "\"confidence\":" + std::to_string(box.confidence) + ",";
                message += "\"x1\":" + std::to_string(box.top_left.x) + ",";
                message += "\"y1\":" + std::to_string(box.top_left.y) + ",";
                message += "\"x2\":" + std::to_string(box.bottom_right.x) + ",";
                message += "\"y2\":" + std::to_string(box.bottom_right.y) + ",";
                message += "\"random_id\":" + std::to_string(rand()%1000000); // 添加随机ID字段
                message += "}";          
                // 添加逗号分隔（最后一个元素不加）
                if (i != boxes.size() - 1) {
                    message += ",";
                }
                // 拼接成一个 JSON 对象 { "p1x":x, "p1y":y, ... }
        }
        message += "]"; // 循环多次后用 ] 收尾就会变成 JSON 数组
        // cout<<"完成了一次JHmessagetoJson"<<endl;
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
