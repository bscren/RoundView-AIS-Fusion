#ifndef JHSTITCHER_HPP
#define JHSTITCHER_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/stitching/detail/autocalib.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <opencv2/stitching/warpers.hpp>

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <map>
#include <shared_mutex>
#include <cstdlib>

using namespace std;
using namespace cv;
using namespace cv::detail;

// 定义时间戳帧结构
struct TimestampedFrame {
    cv::Mat frame;
    std::chrono::high_resolution_clock::time_point timestamp;
};

// 定义变换数据结构
struct TransformationData {
    std::vector<cv::detail::CameraParams> cameras;
    cv::Ptr<cv::detail::CylindricalWarperGpu> warper;
    cv::Ptr<cv::detail::ExposureCompensator> compensator;
    cv::Ptr<cv::detail::SeamFinder> seam_finder;
    std::vector<cv::Point> corners;
    std::vector<cv::Size> sizes;
    std::vector<cv::Mat> masks; // 带拼缝的掩码
    std::vector<cv::UMat> raw_masks_8u; // 原始的矩形掩码
    std::vector<cv::Point> dxdy;   //各相机在最终pano中需要位移的dx-dy
    cv::Rect crop_roi;  //裁剪的roi
};

// 定义yolo检测结果数据结构
struct DetectionResult {
    std::string class_name;
    float confidence;
    uint32_t x1;
    uint32_t y1;
    uint32_t x2;
    uint32_t y2;
};

// 定义检测框信息结构体
struct BoxInfo {
    cv::Point2i top_left;
    cv::Point2i bottom_right;
    std::string class_name;
    float confidence;
};

// 定义跟踪框消息类型
struct VisiableTra {
    std::string camera_name;
    int64_t timestamp_ns;  // 时间戳（纳秒），使用标准C++类型，避免依赖ROS
    int ais_or_not;
    int mmsi;
    double sog;
    double cog;
    double latitude;
    double longitude;
    double box_x1;
    double box_y1;
    double box_x2;
    double box_y2;
};

// 
struct TrajectoryBoxInfo {
    cv::Point2i top_left;
    cv::Point2i bottom_right;
    std::string class_name;
    int mmsi;
    double sog;
    double cog;
    double lat;
    double lon;
};


class JHStitcher {
public:
    // 构造函数，接收拼接相关参数
    JHStitcher(
        std::unordered_map<std::string, int>& cam_name_to_idx,
        int refresh_time = 2,
        size_t min_keypoints = 50,
        double min_confidence = 0.4,
        int min_inliers = 50,
        double max_focal_variance = 50000.0,
        double y_tolerance = 200,
        float roi_threshold = 0.95f,
        float detect_confidence = 0.3f,
        float iou_threshold = 0.5f,
        double scale = 0.75,
        bool cropornot = true,
        bool drawboxornot = true,
        bool save_CameraParams = false,
        string save_CameraParams_path = "/home/tl/RV/src/image_stitching_pkg/config/CameraParams.yaml",
        bool use_saved_CameraParams = false,
        double FOV_hor = 105.0,
        double FOV_ver = 57.0
    );

    // 首次处理图像组具体，生成变换数据
    bool processFirstGroupImpl(const std::vector<cv::Mat>& images);

    // 后续处理图像组具体，使用现有变换数据
    cv::Mat processSubsequentGroupImpl(const std::vector<cv::Mat>& images,\
        const std::unordered_map<std::string, std::vector<DetectionResult>>& latest_detections_, \
        const std::vector<std::queue<VisiableTra>>& latest_visiable_tra_cache_);

    // 检测并更新拼缝线
    void detectStitchLine();

    // 绘制检测框
    void filtBoxes(
        cv::Mat& pano, 
        const TransformationData& data,
        const std::unordered_map<std::string, std::vector<DetectionResult>>& latest_detections_
    );

    // 投影跟踪框到拼接图上
    void CalibTrajInPano(
        cv::Mat& pano,
        const TransformationData& data,
        const std::vector<std::queue<VisiableTra>>& latest_visiable_tra_cache_
    );  

    // 获取cam_name_to_idx_映射（getter接口）
    const std::unordered_map<std::string, int>& getCamNameToIdx() const { return cam_name_to_idx_; }
    
    // 获取最新的变形图像
    const std::vector<cv::UMat>& getLatestWarpImages() const { return latest_warp_images_32F; }

    // 获取筛选后的检测框（getter接口）
    const std::vector<BoxInfo>& getFilteredBoxes() const { return filtered_boxes_;}

    // 获取投影后的轨迹框（getter接口）
    const std::vector<TrajectoryBoxInfo>& getTrajectoryBoxes() const { return trajectory_boxes_;}

    // 获取变换数据（getter接口）
    const TransformationData& getTransformationData() const { return transformation_data_; }

    // 获取视场角参数
    double getFOVHor() const { return FOV_hor_; }
    double getFOVVer() const { return FOV_ver_; }
    
    // 更新拼缝线
    void updatestitchlineImpl();

private:
    // 初始化时的拼接相关参数
    const std::unordered_map<std::string, int> cam_name_to_idx_;
    int refresh_time_;
    size_t min_keypoints_;
    double min_confidence_;
    int min_inliers_;
    double max_focal_variance_;
    double y_tolerance_;
    float roi_threshold_;
    float detect_confidence_;
    float iou_threshold_;
    double scale_;
    bool cropornot_;
    bool drawboxornot_;
    bool save_CameraParams_;
    string save_CameraParams_path_;
    bool use_saved_CameraParams_;
//  视场角参数，也是用于共享的
    double FOV_hor_;
    double FOV_ver_;



// 共享数据和锁:
    // 图像缓存及线程安全控制
    std::vector<cv::UMat> latest_warp_images_32F;
    mutable std::shared_mutex rw_mutex_;
    
    // 拼接图像变换数据及线程安全控制
    TransformationData transformation_data_;
    std::mutex transformation_mutex_;

    // yolo检测框筛选结果
    std::vector<BoxInfo> filtered_boxes_;  // 公有化的检测框容器

    // 轨迹框筛选结果（包含AIS信息）
    std::vector<TrajectoryBoxInfo> trajectory_boxes_;  // 轨迹框容器

    // 检测结果存储（按相机名称分类）由ros节点处理

    

    // 辅助函数
    std::vector<cv::detail::CameraParams> readCameraParamters(const std::string& filename);
    void saveCameraParamters(const std::string& filename, const std::vector<CameraParams>& cameras);
    std::vector<cv::Point> prepare_pxpy(const std::vector<cv::Point>& corners, const std::vector<cv::Size>& sizes);
    float getWhiteRatio(const cv::Mat& mask, int row);
    int findFirstValidRow(const cv::Mat& mask, float threshold);
    int findLastValidRow(const cv::Mat& mask, float threshold);
    cv::Rect getValidRows(const cv::Mat& result_mask, float threshold);
    bool checkAndFilterMatches(std::vector<cv::detail::MatchesInfo>& pairwise_matches);
    bool checkCameraLegitimacy(const std::vector<cv::detail::CameraParams>& cameras);
    bool checkBALegitimacy(std::vector<cv::detail::CameraParams>& cameras);
    bool checkTopLeftPointsLegitimacy(const std::vector<std::vector<cv::Point2f>>& four_corners_warp);
    float calculateIoU(const cv::Rect& a, const cv::Rect& b);
};

#endif // JHSTITCHER_HPP