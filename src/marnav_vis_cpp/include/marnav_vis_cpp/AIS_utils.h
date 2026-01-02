#ifndef MARNAV_VIS_CPP_AIS_UTILS_H
#define MARNAV_VIS_CPP_AIS_UTILS_H

#include <vector>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>

// AIS数据结构
struct AISData {
    uint32_t mmsi;
    double lon;
    double lat;
    double speed;
    double course;
    double heading;
    uint8_t type;
    int64_t timestamp_ms;  // 毫秒时间戳
};

// AIS可视化数据结构（包含图像坐标）
struct AISVisData {
    uint32_t mmsi;
    double lon;
    double lat;
    double speed;
    double course;
    double heading;
    uint8_t type;
    int x;  // 图像x坐标
    int y;  // 图像y坐标
    int64_t timestamp_ms;  // 毫秒时间戳（秒级，即除以1000）
};

// 相机位置参数
struct CameraPosPara {
    double longitude;
    double latitude;
    double horizontal_orientation;
    double vertical_orientation;
    double camera_height;
    double fov_hor;
    double fov_ver;
    double fx;
    double fy;
    double u0;
    double v0;
    double k1 = 0.0;
    double k2 = 0.0;
    double k3 = 0.0;
    double k4 = 0.0;
};

// 工具函数：计算两点间距离（使用GeographicLib）
double count_distance(double lat1, double lon1, double lat2, double lon2, const std::string& type = "m");

// 工具函数：计算方位角
double getDegree(double latA, double lonA, double latB, double lonB);

// 工具函数：将经纬度坐标转换为图像坐标
std::pair<int, int> visual_transform(double lon_v, double lat_v, const CameraPosPara& camera_pos_para, 
                                     const std::pair<int, int>& shape, const std::string& camera_type);

// 数据筛选标志
enum class DataFilterFlag {
    TRANSFORM,      // 可以进行坐标转换
    VIS_TRAJ_DEL,   // 删除视觉轨迹
    AIS_DEL         // 删除AIS数据
};

// 工具函数：数据筛选
DataFilterFlag data_filter(const AISData& ais, const CameraPosPara& camera_pos_para);

// 工具函数：对船舶AIS数据进行推算
AISData data_pre(const AISData& ais, int64_t timestamp_sec);

// AIS处理类
class AISPRO {
public:
    AISPRO(const std::pair<int, int>& im_shape, int t_ms);
    
    // 初始化（返回新的空数据，保存旧数据）
    void initialization(std::vector<AISData>& ais_cur_out,
                       std::vector<AISData>& ais_las_out,
                       std::vector<AISVisData>& ais_vis_out);
    
    // 数据坐标转换
    void data_tran(const std::vector<AISData>& ais_cur, 
                   std::vector<AISVisData>& ais_vis, 
                   const CameraPosPara& camera_pos_para, 
                   int64_t timestamp_ms, 
                   const std::string& camera_type);
    
    // AIS数据处理主流程
    void ais_pro(std::vector<AISData>& ais_cur,
                 const std::vector<AISData>& ais_las,
                 std::vector<AISVisData>& ais_vis,
                 const CameraPosPara& camera_pos_para,
                 const std::vector<AISData>& aisbatch_cache,
                 int64_t timestamp_ms,
                 const std::string& camera_type);
    
    // 外部接口：处理AIS数据
    void process(const std::vector<AISData>& aisbatch_cache,
                 const CameraPosPara& camera_pos_para,
                 int64_t timestamp_ms,
                 const std::string& camera_type,
                 std::vector<AISVisData>& ais_vis_out,
                 std::vector<AISData>& ais_cur_out);

private:
    // 数据粗处理（清洗、筛选）
    void data_coarse_process(std::vector<AISData>& ais_current,
                             const std::vector<AISData>& ais_last,
                             const CameraPosPara& camera_pos_para);
    
    // 数据推算
    void data_pred(std::vector<AISData>& ais_cur,
                   const std::vector<AISData>& ais_read,
                   const std::vector<AISData>& ais_las,
                   int64_t timestamp_ms);
    
    // 坐标转换函数（内部使用）
    void transform(const std::vector<AISData>& ais_current,
                   std::vector<AISVisData>& ais_vis,
                   const CameraPosPara& camera_pos_para,
                   const std::string& camera_type,
                   std::vector<AISVisData>& ais_vis_current);

    std::pair<int, int> im_shape_;  // 图像尺寸 (width, height)
    int max_dis_;                   // 最大探测距离（米）
    int t_ms_;                      // 每帧图像显示时间（毫秒）
    int time_lim_;                  // 数据保存时长（秒）
    
    // 内部状态数据
    std::vector<AISData> ais_cur_;
    std::vector<AISVisData> ais_vis_;
};

#endif // MARNAV_VIS_CPP_AIS_UTILS_H

