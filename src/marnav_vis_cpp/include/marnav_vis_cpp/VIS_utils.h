#ifndef MARNAV_VIS_CPP_VIS_UTILS_H
#define MARNAV_VIS_CPP_VIS_UTILS_H

#include <vector>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include "yolo_onnx.h"
#include "../deep_sort/tracker.h"

namespace marnav_vis_cpp {
namespace vis_utils {

// ============================================================
// 数据结构定义（替代Python的pandas DataFrame）
// ============================================================

/**
 * @brief 视觉轨迹数据结构（对应Python的DataFrame一行）
 * 包含ID、边界框坐标、中心点、速度、时间戳等信息
 */
struct VisTrajectory {
    int id;              // 跟踪ID
    int x1, y1, x2, y2;  // 边界框坐标
    int x, y;            // 中心点坐标
    std::string speed;   // 速度向量（字符串格式"[x_speed, y_speed]"）
    int64_t timestamp;   // 时间戳（秒）
    
    VisTrajectory() 
        : id(-1), x1(0), y1(0), x2(0), y2(0), x(0), y(0), 
          speed("[0, 0]"), timestamp(0) {}
    
    VisTrajectory(int id_, int x1_, int y1_, int x2_, int y2_, 
                  int x_, int y_, int64_t timestamp_)
        : id(id_), x1(x1_), y1(y1_), x2(x2_), y2(y2_), 
          x(x_), y(y_), speed("[0, 0]"), timestamp(timestamp_) {}
};

/**
 * @brief 检测框数据结构（对应Python的检测结果tuple）
 * 格式: (x1, y1, x2, y2, class_name, confidence)
 */
struct DetectionBox {
    float x1, y1, x2, y2;
    std::string class_name;
    float confidence;
    
    DetectionBox() : x1(0), y1(0), x2(0), y2(0), class_name(""), confidence(0) {}
    DetectionBox(float x1_, float y1_, float x2_, float y2_, 
                 const std::string& class_, float conf_)
        : x1(x1_), y1(y1_), x2(x2_), y2(y2_), 
          class_name(class_), confidence(conf_) {}
};

/**
 * @brief AIS可视化数据结构（对应Python的AIS_vis DataFrame一行）
 */
struct AISVisData {
    uint32_t mmsi;
    double lon, lat;
    double speed, course, heading;
    uint8_t type;
    int x, y;           // 图像坐标
    int64_t timestamp;  // 秒级时间戳
    
    AISVisData() : mmsi(0), lon(0), lat(0), speed(0), course(0), 
                   heading(0), type(0), x(0), y(0), timestamp(0) {}
};

/**
 * @brief 绑定信息数据结构（对应Python的bind_inf DataFrame一行）
 */
struct BindInfo {
    int id;         // 视觉跟踪ID
    uint32_t mmsi;  // AIS MMSI
    int64_t timestamp;
    
    BindInfo() : id(-1), mmsi(0), timestamp(0) {}
    BindInfo(int id_, uint32_t mmsi_, int64_t ts_)
        : id(id_), mmsi(mmsi_), timestamp(ts_) {}
};

/**
 * @brief DeepSORT跟踪输出结构（对应Python的outputs列表元素）
 * 格式: [x1, y1, x2, y2, lines, track_id]
 */
struct TrackOutput {
    float x1, y1, x2, y2;
    std::vector<cv::Point> lines;  // 轨迹线
    int track_id;
    
    TrackOutput() : x1(0), y1(0), x2(0), y2(0), track_id(-1) {}
    TrackOutput(float x1_, float y1_, float x2_, float y2_, int id_)
        : x1(x1_), y1(y1_), x2(x2_), y2(y2_), track_id(id_) {}
};

// ============================================================
// 工具函数声明（对应Python的独立函数）
// ============================================================

/**
 * @brief 判断边界框中心点是否在指定区域内
 * @param bounding_box 边界框 [x1, y1, x2, y2]
 * @param area 区域 [x1, y1, x2, y2]
 * @return 1表示在区域内，0表示不在
 * 
 * 对应Python: box_whether_in_area(bounding_box, Area)
 */
int box_whether_in_area(const std::vector<float>& bounding_box, 
                        const std::vector<float>& area);

/**
 * @brief 提取两个轨迹点之间的速度
 * @param last_traj 过去时刻的轨迹
 * @param now_traj 当前时刻的轨迹
 * @return 速度向量 [x_speed, y_speed]
 * 
 * 对应Python: speed_extract(last_traj, now_traj)
 */
std::vector<float> speed_extract(const VisTrajectory& last_traj, 
                                 const VisTrajectory& now_traj);

/**
 * @brief 判断点是否在边界框内
 * @param point 点坐标 [x, y]
 * @param bbox 边界框 [id, x1, y1, x2, y2]
 * @return 1表示在内，0表示不在
 * 
 * 对应Python: whether_in_area(point, bbox)
 */
int whether_in_area(const std::vector<float>& point, 
                    const std::vector<float>& bbox);

/**
 * @brief 判断两个矩形是否重叠，并计算重叠比例
 * @param box1 矩形1 [x1, y1, x2, y2]
 * @param box2 矩形2 [x1, y1, x2, y2]
 * @param val 阈值，重叠比例超过此值返回1
 * @return 1表示重叠超过阈值，0表示未超过
 * 
 * 对应Python: overlap(box1, box2, val)
 */
int overlap(const std::vector<float>& box1, 
            const std::vector<float>& box2, 
            float val);

/**
 * @brief 判断目标是否被遮挡
 * @param bbox 当前边界框 [id, x1, y1, x2, y2]
 * @param cur_bbox_list 当前所有边界框列表
 * @param val 遮挡阈值
 * @param occlusion_bbox_list 输出：遮挡区域列表
 * @param occlusion_id_list 输出：遮挡ID列表
 * 
 * 对应Python: whether_occlusion(bbox, cur_bbox_list, val)
 */
void whether_occlusion(const std::vector<float>& bbox,
                       const std::vector<std::vector<float>>& cur_bbox_list,
                       float val,
                       std::vector<std::vector<float>>& occlusion_bbox_list,
                       std::vector<int>& occlusion_id_list);

/**
 * @brief 判断点是否在任意遮挡区域内
 * @param point 点坐标 [x, y]
 * @param oar_list 遮挡区域列表
 * @return 1表示在遮挡区域内，0表示不在
 * 
 * 对应Python: whether_in_OAR(point, OAR_list)
 */
int whether_in_OAR(const std::vector<float>& point,
                   const std::vector<std::vector<float>>& oar_list);

/**
 * @brief 提取遮挡区域（Occlusion Area Region）
 * @param his_traj_list 历史轨迹数据列表（最多5秒）
 * @param val 遮挡阈值
 * @param oar_list 输出：遮挡区域列表
 * @param oar_id_list 输出：遮挡目标ID列表
 * 
 * 对应Python: OAR_extractor(his_traj_dataframe_list, val)
 */
void OAR_extractor(const std::vector<std::vector<VisTrajectory>>& his_traj_list,
                   float val,
                   std::vector<std::vector<float>>& oar_list,
                   std::vector<int>& oar_id_list);

/**
 * @brief 提取运动特征（速度）
 * @param his_traj_list 历史轨迹数据列表
 * @param vis_tra_cur 当前轨迹数据
 * @return 带速度信息的当前轨迹数据
 * 
 * 对应Python: motion_features_extraction(his_traj_dataframe_list, VIS_tra_cur)
 */
std::vector<VisTrajectory> motion_features_extraction(
    const std::vector<std::vector<VisTrajectory>>& his_traj_list,
    const std::vector<VisTrajectory>& vis_tra_cur);

/**
 * @brief 判断ID是否在过去5秒的轨迹中稳定存在
 * @param id 目标ID
 * @param last_5_trajs 过去5秒的轨迹列表
 * @return true表示稳定存在，false表示不稳定
 * 
 * 对应Python: id_whether_stable(id, last_5_trajs)
 */
bool id_whether_stable(int id, 
                       const std::vector<std::vector<VisTrajectory>>& last_5_trajs);

// ============================================================
// 前向声明（DeepSort接口，待C++实现完成后补充）
// ============================================================

/**
 * @brief DeepSORT跟踪器接口类（预留接口）
 * 
 * TODO: 待DeepSort C++实现完成后，替换为实际的DeepSort类
 * 参考Python版本: deep_sort.deep_sort.DeepSort
 */
class DeepSortInterface {
public:
    virtual ~DeepSortInterface() = default;
    
    /**
     * @brief DeepSort更新函数（核心接口）
     * @param bbox_xywh 检测框（中心点+宽高格式）[N, 4]
     * @param confidences 置信度 [N]
     * @param ori_img 原始图像
     * @param bbox_xywh_anti_occ 抗遮挡预测框 [M, 4]
     * @param confidences_anti_occ 抗遮挡置信度 [M]
     * @param id_list 抗遮挡对应的ID列表 [M]
     * @param timestamp 时间戳（秒）
     * @return 跟踪结果列表 [[x1,y1,x2,y2,lines,track_id], ...]
     * 
     * 对应Python: deepsort.update(xywhs, confss, image, xywhs_anti_occ, 
     *                             confss_anti_occ, id_list, timestamp)
     */
    virtual std::vector<TrackOutput> update(
        const std::vector<std::vector<float>>& bbox_xywh,
        const std::vector<float>& confidences,
        const cv::Mat& ori_img,
        const std::vector<std::vector<float>>& bbox_xywh_anti_occ = {},
        const std::vector<float>& confidences_anti_occ = {},
        const std::vector<int>& id_list = {},
        int64_t timestamp = 0) = 0;
};

// ============================================================
// VISPRO类（视觉处理核心类）
// ============================================================

/**
 * @brief 视觉处理类（对应Python的VISPRO类）
 * 
 * 功能包括：
 * 1. 目标检测（detection）
 * 2. 目标跟踪（track，基于DeepSort）
 * 3. 轨迹更新（update_tra）
 * 4. 抗遮挡预测（anti_occ）
 * 5. 主处理流程（feedCap）
 * 
 * 数据结构说明：
 * - 使用std::vector<VisTrajectory>替代Python的pandas.DataFrame
 * - last5_vis_tra_list存储过去5秒的轨迹数据（每秒一个vector）
 * - Vis_tra存储所有历史轨迹（限制2分钟）
 */
class VISPRO {
private:
    /**
     * @brief 构造函数
     * @param anti 是否启用抗遮挡功能
     * @param val 遮挡阈值
     * @param t 时间间隔（毫秒）
     * @param logger ROS2日志接口
     * @param model_path 目标检测模型路径
     * @param classes_path 目标检测类别文件路径
     * @param cfg_path 目标跟踪配置文件路径
     * 
     * 对应Python: def __init__(self, anti, val, t)
     */
    VISPRO(bool anti, float val, int t, rclcpp::Logger logger,
        const std::string& model_path, const std::string& classes_path,
        const std::string& cfg_path);
    
    ~VISPRO() = default;
    
    std::unique_ptr<YOLO_ONNX> yolo_detector_;
    // std::unique_ptr<DeepSort> deepsort_;

    /**
    // YOLO检测器
     * @brief 目标检测函数
     * @param image 输入图像（BGR格式）
     * @return 检测框列表 [(x1,y1,x2,y2,class,conf), ...]
     * 
     * 对应Python: def detection(self, image)
     * 
     * TODO: 需要集成YOLO检测模型（可使用yolo_onnx.cpp）
     */
    std::vector<DetectionBox> detection(const cv::Mat& image);
    
    /**
     * @brief 目标跟踪函数（调用DeepSort）
     * @param image 输入图像
     * @param bboxes 检测框列表（不含遮挡区域）
     * @param bboxes_anti_occ 遮挡区域的抗遮挡预测框
     * @param id_list 抗遮挡预测框对应的ID列表
     * @param timestamp 时间戳（毫秒）
     * 
     * 对应Python: def track(self, image, bboxes, bboxes_anti_occ, id_list, timestamp)
     * 
     * TODO: 待DeepSort C++实现完成后补充实际调用
     * 当前预留接口，输出存储到Vis_tra_cur_3
     */
    void track(const cv::Mat& image,
               const std::vector<DetectionBox>& bboxes,
               const std::vector<DetectionBox>& bboxes_anti_occ,
               const std::vector<int>& id_list,
               int64_t timestamp);
    
    /**
     * @brief 轨迹数据更新函数（每秒调用一次）
     * @param timestamp 当前时间戳（毫秒）
     * @return 当前秒的轨迹数据（带速度信息）
     * 
     * 核心逻辑：
     * 1. 对Vis_tra_cur_3中同一ID的多帧数据求均值，得到Vis_tra_cur
     * 2. 提取运动特征（速度），得到Vis_tra_cur_withfeature
     * 3. 更新last5_vis_tra_list（保持最多5个元素）
     * 4. 删除2分钟前的历史数据
     * 
     * 对应Python: def update_tra(self, Vis_tra, timestamp)
     */
    std::vector<VisTrajectory> update_tra(int64_t timestamp);
    
    /**
     * @brief 基于视觉速度的轨迹预测
     * @param last_traj 若干秒前的轨迹
     * @param timestamp 预测到的目标时间戳（秒）
     * @param speed 速度向量 [x_speed, y_speed]
     * @return 预测的轨迹
     * 
     * 对应Python: def traj_prediction_via_visual(self, last_traj, timestamp, speed)
     */
    VisTrajectory traj_prediction_via_visual(const VisTrajectory& last_traj,
                                             int64_t timestamp,
                                             const std::vector<float>& speed);
    
    /**
     * @brief 抗遮挡处理函数
     * @param last5_vis_tra_list 过去5秒的视觉轨迹列表
     * @param bboxes 检测框列表（会被修改，删除遮挡区域内的检测）
     * @param ais_vis AIS可视化数据
     * @param bind_inf 绑定信息（ID-MMSI映射）
     * @param timestamp 时间戳（秒）
     * @return 抗遮挡预测框列表
     * 
     * 核心逻辑：
     * 1. 删除处在遮挡区域（OAR）内的检测结果
     * 2. 对遮挡ID，根据MMSI使用AIS预测位置，或使用视觉速度预测
     * 3. 生成抗遮挡检测框（用于DeepSort的bbox_xywh_anti_occ参数）
     * 
     * 对应Python: def anti_occ(self, last5_vis_tra_list, bboxes, AIS_vis, bind_inf, timestamp)
     */
    std::vector<DetectionBox> anti_occ(
        const std::vector<std::vector<VisTrajectory>>& last5_vis_tra_list,
        std::vector<DetectionBox>& bboxes,
        const std::vector<AISVisData>& ais_vis,
        const std::vector<BindInfo>& bind_inf,
        int64_t timestamp);
    
    /**
     * @brief 主处理流程（每帧调用）
     * @param image 输入图像
     * @param ais_vis AIS可视化数据
     * @param bind_inf 绑定信息
     * @param timestamp 时间戳（毫秒）
     * @return 当前轨迹数据（pair: <所有历史轨迹, 当前秒轨迹>）
     * 
     * 核心流程：
     * 1. 目标检测（detection）
     * 2. 抗遮挡处理（anti_occ）
     * 3. DeepSort跟踪（track）
     * 4. 轨迹更新（update_tra，每秒调用一次）
     * 5. 提取遮挡区域（OAR_extractor）
     * 
     * 对应Python: def feedCap(self, image, AIS_vis, bind_inf, timestamp)
     * 
     * 调用接口说明（供DeepSORVF_ros.cpp使用）：
     * ```cpp
     * // 1. 创建VISPRO实例（在worker_thread中）
     * VISPRO vispro(true, 0.5, 1000, node->get_logger());
     * 
     * // 2. 在每帧图像回调中调用feedCap
     * auto result = vispro.feedCap(cv_image, ais_vis_data, bind_info, timestamp_ms);
     * std::vector<VisTrajectory> vis_tra = result.first;   // 所有历史轨迹
     * std::vector<VisTrajectory> vis_cur = result.second;  // 当前秒轨迹
     * ```
     */
    std::pair<std::vector<VisTrajectory>, std::vector<VisTrajectory>> feedCap(
        const cv::Mat& image,
        const std::vector<AISVisData>& ais_vis,
        const std::vector<BindInfo>& bind_inf,
        int64_t timestamp);
    
    // 获取内部状态（用于调试或外部访问）
    const std::vector<VisTrajectory>& get_vis_tra() const { return vis_tra_; }
    const std::vector<VisTrajectory>& get_vis_tra_cur() const { return vis_tra_cur_; }
    const std::vector<std::vector<float>>& get_oar_list() const { return oar_list_; }
    const std::vector<int>& get_oar_ids_list() const { return oar_ids_list_; }

private:
    // 配置参数
    bool anti_;           // 是否启用抗遮挡
    float val_;           // 遮挡阈值
    int t_;               // 时间间隔（毫秒）
    rclcpp::Logger logger_;
    
    // 数据缓存（对应Python的成员变量）
    std::vector<std::vector<VisTrajectory>> last5_vis_tra_list_;  // 过去5秒轨迹列表
    std::vector<VisTrajectory> vis_tra_cur_3_;   // 当前秒内所有帧的轨迹（累积）
    std::vector<VisTrajectory> vis_tra_cur_;     // 当前秒的轨迹（均值）
    std::vector<VisTrajectory> vis_tra_;         // 所有历史轨迹（2分钟限制）
    std::vector<VisTrajectory> vis_tra_last_;    // 上一秒的轨迹（带速度）
    
    // 遮挡相关
    std::vector<std::vector<float>> oar_list_;        // 遮挡区域列表 [[x1,y1,x2,y2], ...]
    std::vector<int> oar_ids_list_;                   // 遮挡ID列表
    std::vector<std::pair<int, uint32_t>> oar_mmsi_list_; // 遮挡ID-MMSI映射 [[id,mmsi], ...]
    std::vector<VisTrajectory> anti_occlusion_traj_;  // 被遮挡目标的轨迹数据
    
    // DeepSort接口（预留）
    // TODO: 待DeepSort C++实现完成后，替换为实际的DeepSort实例
    // std::shared_ptr<DeepSortInterface> deepsort_;
};

} // namespace vis_utils
} // namespace marnav_vis_cpp

#endif // MARNAV_VIS_CPP_VIS_UTILS_H

