// 适合作为调试程序，用于初步测试拼接算法和相机连接，核心原因是其代码结构完整覆盖了相机连接验证和拼接算法流程，且包含便于调试的中间输出和基础错误处理。
// 介绍：用于多相机的实时拼接并通过节点发布拼接后的图像
// 直接处理RTSP 视频流，通过cv::VideoCapture从 RTSP URL 读取原始视频帧，再通过自定义的FrameQueue和FrameSynchronizer类实现多相机帧的缓存与时间同步（基于本地时间戳）。
// 输入源是 “硬件 / 网络视频流”，不依赖外部 ROS 2 话题。
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <vector>
#include <memory>
#include <chrono>
#include <string>

// ROS 2 相关头文件
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/timelapsers.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include <opencv2/xfeatures2d.hpp>  


using namespace cv;
using namespace std;
using namespace cv::detail;

const int input_frame_rate = 20; // 相机设置的输入帧率
const int target_frame_rate = 20; // 调整后的目标帧率，考虑到拼接处理时间
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

// 定义一个结构体来保存变换数据
struct TransformationData {
    std::vector<cv::detail::CameraParams> cameras;
    cv::Ptr<cv::detail::RotationWarper> warper;
    cv::Ptr<cv::detail::ExposureCompensator> compensator;
    cv::Ptr<cv::detail::SeamFinder> seam_finder;
    std::vector<cv::Point> corners;
    std::vector<cv::Size> sizes;
    std::vector<cv::Mat> masks;
    cv::Rect roi; // 用于裁剪全景图像的区域
};

// 函数用于处理第一组图片并保存变换数据
TransformationData processFirstGroup(const std::vector<cv::Mat>& images) {
    int num_images = images.size(); //图片数量
    cout<<"处理第一组图片" << endl;
    //存储图像、尺寸、特征点
    std::vector<cv::detail::ImageFeatures> features(num_images);  //存储图像特征点
    std::vector<cv::Size> images_sizes(num_images); //存储图像的尺寸
    cv::Ptr<cv::AKAZE> featurefinder = cv::AKAZE::create();

    for (int i = 0; i < num_images; ++i)
    {
        images_sizes[i] = images[i].size();
        imwrite("images_" + std::to_string(i) + ".png", images[i]); // Convert i to string
        cv::detail::computeImageFeatures(featurefinder, images[i], features[i]);    //计算图像特征
        std::cout << "image #" << i + 1 << " 特征点为: " << features[i].keypoints.size() << " 个"<<"  "<< "尺寸为: " << images_sizes[i] << std::endl;
    }
    std::cout << std::endl;

    //图像特征点匹配
    std::vector<cv::detail::MatchesInfo> pairwise_matches; //表示特征匹配信息变量
    cv::Ptr<cv::detail::FeaturesMatcher> matcher = cv::makePtr<cv::detail::BestOf2NearestMatcher>(false, 0.3f, 6, 6); //定义特征匹配器，2NN方法
    (*matcher)(features, pairwise_matches);  //进行特征匹配

    // 显示每两幅图片之间特征点的匹配关系
    for (int i = 0; i < num_images; ++i) {
        for (int j = i + 1; j < num_images; ++j) {
            const MatchesInfo& match_info = pairwise_matches[i * num_images + j];
            if (match_info.matches.size() > 0) {
                Mat match_img;
                drawMatches(images[i], features[i].keypoints, images[j], features[j].keypoints,
                            match_info.matches, match_img);
                // 显示匹配结果
                imwrite("matches_" + std::to_string(i) + "_" + std::to_string(j) + ".png", match_img); // Convert i and j to string
            }
        }
    }

    //预估相机参数
    cv::Ptr<cv::detail::Estimator> estimator;
    estimator = cv::makePtr<cv::detail::HomographyBasedEstimator>();  //水平估计
    std::vector<cv::detail::CameraParams> cameras;  //相机参数素组
    (*estimator)(features, pairwise_matches, cameras);  //得到相机参数
    std::cout << "预估相机参数:" << std::endl;
    for (size_t i = 0; i < cameras.size(); ++i)
    {
        cv::Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras[i].R = R;
        std::cout << "camera #" << i + 1 << ":\n内参数矩阵K:\n" << cameras[i].K() << "\n旋转矩阵R:\n" << cameras[i].R << "\n焦距focal: " << cameras[i].focal << std::endl;
    }
    std::cout << std::endl;

    // 光束平差，精确相机参数
    cv::Ptr<cv::detail::BundleAdjusterBase> adjuster;
    adjuster = cv::makePtr<cv::detail::BundleAdjusterRay>();
    (*adjuster)(features, pairwise_matches, cameras);
    std::cout << "精确相机参数" << std::endl;
    for (size_t i = 0; i < cameras.size(); ++i)
    {
        cv::Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras[i].R = R;
        std::cout << "camera #" << i + 1 << ":\n内参数矩阵K:\n" << cameras[i].K() << "\n旋转矩阵R:\n" << cameras[i].R << "\n焦距focal: " << cameras[i].focal << std::endl;
    }
    std::cout << std::endl;

    // 波形矫正
    std::vector<cv::Mat> mat;
    for (size_t i = 0; i < cameras.size(); ++i)
        mat.push_back(cameras[i].R);
    cv::detail::waveCorrect(mat, cv::detail::WAVE_CORRECT_HORIZ); //水平校正
    for (size_t i = 0; i < cameras.size(); ++i)
        cameras[i].R = mat[i];
    std::cout << std::endl;
    
    // //创建mask图像
    std::vector<cv::Mat> masks(num_images);
    for (int i = 0; i < num_images; ++i)
    {
        masks[i].create(images[i].size(), CV_8U);
        masks[i].setTo(cv::Scalar::all(255));
    }

    //图像、掩码变换
    std::vector<cv::Mat> masks_warp(num_images);  //mask扭曲
    std::vector<cv::Mat> images_warp(num_images); //图像扭曲
    std::vector<cv::Point> corners(num_images);   //图像左角点
    std::vector<cv::Size> sizes(num_images);		 //图像尺寸
    cv::Ptr<cv::WarperCreator> warper_creator=cv::makePtr<cv::CylindricalWarper>();  //柱面投影
    cv::Ptr<cv::detail::RotationWarper> warper = warper_creator->create(static_cast<float>(cameras[0].focal));  //因为图像焦距都一样
    
    vector<Point2f> four_corners(4); //四个角点
    vector<vector<Point2f>> four_corners_warp(num_images); //扭曲图像左角点
    for (int i = 0; i < num_images; ++i)
    {
        cv::Mat K;
        cameras[i].K().convertTo(K, CV_32F);
        corners[i] = warper->warp(images[i], K, cameras[i].R, cv::INTER_LINEAR, cv::BORDER_REFLECT, images_warp[i]);  //扭曲图像images->images_warp
        sizes[i] = images_warp[i].size();
        warper->warp(masks[i], K, cameras[i].R, cv::INTER_NEAREST, cv::BORDER_CONSTANT, masks_warp[i]);  //扭曲masks->masks_warped
    
    
        four_corners[0] = Point(0, 0); //左上角
        four_corners[1] = Point(images[i].cols-1, 0); //右上角
        four_corners[2] = Point(images[i].cols-1, images[i].rows-1); //右下角
        four_corners[3] = Point(0, images[i].rows-1); //左下角

        // 计算投影后的掩码的四个角点
        four_corners_warp[i].resize(4);
        for (int j = 0; j < 4; ++j)
        {
        four_corners_warp[i][j] = warper->warpPoint(four_corners[j], K, cameras[i].R); //扭曲图像左角点
        }
    }
    for (int i = 0; i < num_images; ++i)
    {
        std::cout << "Image #" << i + 1 << "  corner: " << corners[i] << "  " << "size: " << sizes[i] << std::endl;
        cout << "Image #" << i + 1 << "  four_corners_warp: ";
            for (const auto& pt : four_corners_warp[i]) {
                cout << "(" << pt.x << "," << pt.y << ") ";
            }
            cout << endl;  
    }
     std::cout << std::endl;

    // 修正曝光补偿器的创建和使用
    cv::Ptr<cv::detail::ExposureCompensator> compensator = cv::makePtr<cv::detail::BlocksGainCompensator>(); // GAIN类型对应GainCompensator子类

    std::vector<cv::UMat> images_warp_umat(num_images);
    std::vector<cv::UMat> masks_warp_umat(num_images);
    for (int i = 0; i < num_images; ++i) {
        images_warp[i].copyTo(images_warp_umat[i]);
        masks_warp[i].copyTo(masks_warp_umat[i]);
    }

    compensator->feed(corners, images_warp_umat, masks_warp_umat);

    for(int i = 0; i < num_images; ++i) {
        cv::UMat img_umat, mask_umat;
        images_warp[i].copyTo(img_umat);
        masks_warp[i].copyTo(mask_umat);
        compensator->apply(i, corners[i], img_umat, mask_umat);
        img_umat.copyTo(images_warp[i]); // 写回原Mat
        mask_umat.copyTo(masks_warp[i]);
    }
    cout<< "曝光补偿完成" << endl;

    // 使用图割法寻找缝合线
    cv::Ptr<cv::detail::SeamFinder> seam_finder = cv::makePtr<cv::detail::GraphCutSeamFinder>(cv::detail::GraphCutSeamFinderBase::COST_COLOR);

    for (int i = 0; i < num_images; ++i) {
        images_warp[i].convertTo(images_warp_umat[i], CV_32F); // 确保图像数据类型为CV_32F，不然会导致图割法错误。
        masks_warp[i].convertTo(masks_warp_umat[i], CV_8U);   // 确保掩码数据类型为CV_8U，去掉 'U8'，掩码会更新错误。
    }

    try {
        seam_finder->find(images_warp_umat, corners, masks_warp_umat);
    } catch (const cv::Exception& e) {
        std::cerr << "Error in seam finding: " << e.what() << std::endl;
        return TransformationData();
    }

    for (int i = 0; i < num_images; ++i) {
        images_warp_umat[i].copyTo(images_warp[i]);
        masks_warp_umat[i].copyTo(masks_warp[i]);
    }
    cout<< "图割法寻找缝合线完成" << endl;

    //图像融合
    cv::Ptr<cv::detail::Blender> blender; //定义图像融合器
    blender = cv::detail::Blender::createDefault(cv::detail::Blender::NO, false); //简单融合方法
    blender->prepare(corners, sizes);  //生成全景图像区域
    for (int i = 0; i < num_images; ++i)
    {
        images_warp[i].convertTo(images_warp[i], CV_16S); 
        blender->feed(images_warp[i], masks_warp[i], corners[i]);  //处理图像 初始化数据
    }
    cv::Mat result, result_mask;
    blender->blend(result, result_mask);

    cout << "图像融合完成" << endl;


    // 裁剪图像result
    int top_left_y = std::max(
    std::abs(std::abs(corners.front().y) - std::abs(four_corners_warp.front()[0].y)),//要么左边
    std::abs(std::abs(corners.back().y) - std::abs(four_corners_warp.back()[1].y))//要么右边
    );
    int width = result.cols;
    int height = min(four_corners_warp.front()[3].y-four_corners_warp.front()[0].y,four_corners_warp.back()[2].y-four_corners_warp.back()[1].y);
    Point top_left(0, top_left_y); // 左上角点
    Point bottom_right(top_left.x + width, top_left.y + height); // 右下角点
    
    cv::Rect roi(top_left ,bottom_right);
    cv::Mat croppedImage = result(roi);
    cout << "裁剪后的图像尺寸: " << croppedImage.size() << endl;
    cout << "裁剪位置：左上: " << top_left << ", 右下: " << bottom_right << endl;
    
    TransformationData data;
    data.cameras = cameras;
    data.warper = warper;
    data.compensator = compensator;
    data.seam_finder = seam_finder;
    data.corners = corners;
    data.sizes = sizes;
    data.masks = masks_warp;
    data.roi = roi;
    return data;
}

// 函数用于使用保存的变换数据处理后续组图片
cv::Mat processSubsequentGroup(const std::vector<cv::Mat>& images, const TransformationData& data) {
    int num_images = images.size(); //图片数量

    //图像变换
    std::vector<cv::Mat> images_warp(num_images); //图像扭曲
    for (int i = 0; i < num_images; ++i)
    {
        cv::Mat K;
        data.cameras[i].K().convertTo(K, CV_32F);
        data.warper->warp(images[i], K, data.cameras[i].R, cv::INTER_LINEAR, cv::BORDER_REFLECT, images_warp[i]);  //扭曲图像images->images_warp
    }

    // 修正曝光补偿
    for (int i = 0; i < num_images; ++i) {
        data.compensator->apply(i, data.corners[i], images_warp[i], data.masks[i]);
    }

    //图像融合
    cv::Ptr<cv::detail::Blender> blender; //定义图像融合器
    blender = cv::detail::Blender::createDefault(cv::detail::Blender::NO, false); //简单融合方法
    blender->prepare(data.corners, data.sizes);  //生成全景图像区域
    for (int i = 0; i < num_images; ++i)
    {
        images_warp[i].convertTo(images_warp[i], CV_16S); 
        blender->feed(images_warp[i], data.masks[i], data.corners[i]);  //处理图像 初始化数据
    }

    cv::Mat pano_result, result_mask;
    blender->blend(pano_result, result_mask);  //blend( InputOutputArray dst, InputOutputArray dst_mask  )混合并返回最后的pano。
    cv::Mat croppedImage = pano_result(data.roi);

    return croppedImage;
}

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
    image_transport::ImageTransport it(node);
    auto stitch_pub = it.advertise("stitched_image", 1);

    // 解析命令行参数，生成RTSP URL列表
    std::vector<std::string> rtsp_urls;
    for (int i = 1; i < argc; ++i) {
        int camera_id = std::stoi(argv[i]);
        std::string url = "rtsp://admin:juhai00" + std::to_string(camera_id) + "@192.168.1." + std::to_string(000 + camera_id) + ":554/Streaming/Channels/2";
        rtsp_urls.push_back(url);
    }

    // 为每个摄像头创建发布者
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

    rclcpp::Rate loop_rate(target_frame_rate); // 降低帧率以提高同步质量
    bool first_group_processed = false;
    TransformationData transformation_data;

    while (rclcpp::ok()) {
        std::vector<TimestampedFrame> synchronized_frames;
        
        // 尝试获取同步帧
        if (frame_synchronizer.get_synchronized_frames(synchronized_frames)) {
            std::vector<cv::Mat> frames;
            for (const auto& tf : synchronized_frames) {
                frames.push_back(tf.frame);
            }

            cv::Mat stitched_image;
            if (!first_group_processed) {
                // 处理第一组图片
                auto start_first_group = std::chrono::high_resolution_clock::now();
                transformation_data = processFirstGroup(frames);
                auto end_first_group = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> duration_first_group = end_first_group - start_first_group;
                std::cout << "Processing first group took: " << duration_first_group.count() << " seconds." << std::endl;
                stitched_image = cv::imread("result_first.png");
                first_group_processed = true;
            } else {
                // 处理后续组图片
                auto start_subsequent_group = std::chrono::high_resolution_clock::now();
                stitched_image = processSubsequentGroup(frames, transformation_data);
                auto end_subsequent_group = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> duration_subsequent_group = end_subsequent_group - start_subsequent_group;
                std::cout << "Processing subsequent group took: " << duration_subsequent_group.count() << " seconds." << std::endl;
            }

            
            Mat stitched_8u; // 用于存储最终的8位图像
            // 根据实际类型执行不同的转换逻辑
            if (stitched_image.type() == CV_16SC3) { // 处理 CV_16SC3 类型
                std::cout << "执行CV_16SC3分支" << std::endl;
                
                // 1. 转换为有符号32位，避免计算时溢出
                cv::Mat stitched_32s;
                stitched_image.convertTo(stitched_32s, CV_32SC3);
                
                // 2. 将负值调整到正值范围（例如，将-32768~32767映射到0~65535）
                cv::Mat stitched_32u;
                cv::add(stitched_32s, cv::Scalar(32768, 32768, 32768), stitched_32u, cv::noArray(), CV_32SC3);
                
                // 3. 归一化到0-255范围
                cv::normalize(stitched_32u, stitched_8u, 0, 255, cv::NORM_MINMAX, CV_8UC3);
            } 

            // 发布拼接后的图像
            auto msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), 
                "bgr8", 
                stitched_8u
            ).toImageMsg();
            msg->header.stamp = node->get_clock()->now();
            stitch_pub.publish(msg);
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







