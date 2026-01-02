#include "marnav_vis_cpp/AIS_utils.h"
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <algorithm>
#include <limits>
#include <cstring>

using namespace GeographicLib;

// 计算两点间距离（使用GeographicLib）
double count_distance(double lat1, double lon1, double lat2, double lon2, const std::string& type) {
    const Geodesic& geod = Geodesic::WGS84();
    double s12;  // 距离（米）
    double a1, a2;  // 方位角（未使用）
    geod.Inverse(lat1, lon1, lat2, lon2, s12, a1, a2);
    
    if (type == "nm") {
        // 转换为海里
        s12 = s12 * 0.00054;
    }
    return s12;
}

// 计算方位角
double getDegree(double latA, double lonA, double latB, double lonB) {
    const Geodesic& geod = Geodesic::WGS84();
    double s12, a1, a2;
    geod.Inverse(latA, lonA, latB, lonB, s12, a1, a2);
    // a1是从A到B的方位角（度）
    return a1;
}

// 将经纬度坐标转换为图像坐标
std::pair<int, int> visual_transform(double lon_v, double lat_v, const CameraPosPara& camera_pos_para, 
                                     const std::pair<int, int>& shape, const std::string& camera_type) {
    double lon_cam = camera_pos_para.longitude;
    double lat_cam = camera_pos_para.latitude;
    double shoot_hdir = camera_pos_para.horizontal_orientation;
    double shoot_vdir = camera_pos_para.vertical_orientation;
    double height_cam = camera_pos_para.camera_height;
    
    double f_x = camera_pos_para.fx;
    double f_y = camera_pos_para.fy;
    double u0 = camera_pos_para.u0;
    double v0 = camera_pos_para.v0;
    
    // 1. 计算距离
    double D_abs = count_distance(lat_cam, lon_cam, lat_v, lon_v);
    
    // 2. 计算水平夹角
    double relative_angle = getDegree(lat_cam, lon_cam, lat_v, lon_v);
    double Angle_hor = relative_angle - shoot_hdir;
    if (Angle_hor < -180) {
        Angle_hor += 360;
    } else if (Angle_hor > 180) {
        Angle_hor -= 360;
    }
    
    double hor_rad = Angle_hor * M_PI / 180.0;
    double shv_rad = (-shoot_vdir) * M_PI / 180.0;
    
    // 绕x轴旋转shv_rad度，纠正相机俯仰角为理想的0，和对应的船只坐标(X,Y,Z)
    double Z_w = D_abs * std::cos(hor_rad);
    double X_w = D_abs * std::sin(hor_rad);
    double Y_w = height_cam;
    double Z = Z_w / std::cos(shv_rad) + (Y_w - Z_w * std::tan(shv_rad)) * std::sin(shv_rad);
    double X = X_w;
    double Y = (Y_w - Z_w * std::tan(shv_rad)) * std::cos(shv_rad);
    
    if (camera_type == "normal") {
        // 内参K_matrix计算得到目标图像坐标
        int target_x = static_cast<int>(f_x * X / Z + u0);
        int target_y = static_cast<int>(f_y * Y / Z + v0);
        return {target_x, target_y};
    } else if (camera_type == "fisheye") {
        // 使用鱼眼相机投影公式计算得到目标图像坐标
        cv::Mat K = (cv::Mat_<double>(3, 3) << 
                     f_x, 0, u0,
                     0, f_y, v0,
                     0, 0, 1);
        cv::Mat D = (cv::Mat_<double>(4, 1) << 
                     camera_pos_para.k1, 
                     camera_pos_para.k2, 
                     camera_pos_para.k3, 
                     camera_pos_para.k4);
        
        std::vector<cv::Point3d> object_points = {cv::Point3d(X, Y, Z)};
        std::vector<cv::Point2d> image_points;
        
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
        
        cv::fisheye::projectPoints(object_points, image_points, rvec, tvec, K, D);
        
        int target_x = static_cast<int>(image_points[0].x);
        int target_y = static_cast<int>(image_points[0].y);
        return {target_x, target_y};
    }
    
    return {0, 0};
}

// 数据筛选
DataFilterFlag data_filter(const AISData& ais, const CameraPosPara& camera_pos_para) {
    double lon_cam = camera_pos_para.longitude;
    double lat_cam = camera_pos_para.latitude;
    double shoot_hdir = camera_pos_para.horizontal_orientation;
    double shoot_vdir = camera_pos_para.vertical_orientation;
    double height_cam = camera_pos_para.camera_height;
    double FOV_hor = camera_pos_para.fov_hor;
    double FOV_ver = camera_pos_para.fov_ver;
    
    double lon = ais.lon;
    double lat = ais.lat;
    double D_abs = count_distance(lat_cam, lon_cam, lat, lon);
    double angle = getDegree(lat_cam, lon_cam, lat, lon);
    
    double diff = std::abs(shoot_hdir - angle);
    double in_angle = (diff < 180) ? diff : (360 - diff);
    
    // 先进行相机垂直方向可视范围的判断
    double vertical_angle_deg = std::atan(D_abs / height_cam) * 180.0 / M_PI;
    if (90 + shoot_vdir - FOV_ver / 2 < vertical_angle_deg) {
        // 相机水平方向可视范围判断
        if (in_angle <= (FOV_hor / 2 + 8)) {
            return DataFilterFlag::TRANSFORM;
        } else if (in_angle > (FOV_hor / 2 + 8)) {
            return DataFilterFlag::VIS_TRAJ_DEL;
        }
        
        // 如果当前AIS数据在更大范围的扇形范围内，则删除
        if (in_angle > (FOV_hor / 2 + 12)) {
            return DataFilterFlag::AIS_DEL;
        }
    }
    
    return DataFilterFlag::AIS_DEL;
}

// 对船舶AIS数据进行推算
AISData data_pre(const AISData& ais, int64_t timestamp_sec) {
    AISData result = ais;
    
    // 若船舶速度为0，则仅修改其时间
    if (result.speed == 0) {
        result.timestamp_ms = timestamp_sec * 1000;
        return result;
    }
    
    // 否则对其进行推算
    const Geodesic& geod = Geodesic::WGS84();
    double distance = result.speed * ((timestamp_sec - result.timestamp_ms / 1000) / 3600.0) * 1852.0;  // 米
    
    double lat_out, lon_out;
    double azi1, azi2;
    geod.Direct(result.lat, result.lon, result.course, distance, lat_out, lon_out, azi2);
    
    result.lat = lat_out;
    result.lon = lon_out;
    result.timestamp_ms = timestamp_sec * 1000;
    
    return result;
}

// AISPRO类实现
AISPRO::AISPRO(const std::pair<int, int>& im_shape, int t_ms)
    : im_shape_(im_shape), t_ms_(t_ms) {
    max_dis_ = 2 * 1852;  // 2海里，单位米
    time_lim_ = 2;  // 2分钟
}

void AISPRO::initialization(std::vector<AISData>& ais_cur_out,
                            std::vector<AISData>& ais_las_out,
                            std::vector<AISVisData>& ais_vis_out) {
    // 保存当前数据为上一时刻数据
    ais_las_out = ais_cur_;
    ais_vis_out = ais_vis_;
    // 返回新的空数据
    ais_cur_out.clear();
}

void AISPRO::data_coarse_process(std::vector<AISData>& ais_current,
                                  const std::vector<AISData>& ais_last,
                                  const CameraPosPara& camera_pos_para) {
    double lat_cam = camera_pos_para.latitude;
    double lon_cam = camera_pos_para.longitude;
    
    auto it = ais_current.begin();
    while (it != ais_current.end()) {
        bool should_remove = false;
        
        // 1. 清洗异常数据
        if (it->mmsi / 100000000 < 1 || it->mmsi / 100000000 >= 10 ||
            it->lon == -1 || it->lat == -1 || it->speed == -1 ||
            it->course == -1 || it->course == 360 || it->heading == -1 ||
            it->lon > 180 || it->lon < 0 || it->lat > 90 || it->lat < 0 ||
            it->speed <= 0.3) {
            should_remove = true;
        }
        
        // 2. 清洗经纬度速度发生较大变化的数据
        if (!should_remove) {
            for (const auto& last_ais : ais_last) {
                if (last_ais.mmsi == it->mmsi) {
                    if (std::abs(it->lon - last_ais.lon) >= 1 ||
                        std::abs(it->lat - last_ais.lat) >= 1 ||
                        std::abs(it->speed - last_ais.speed) >= 7) {
                        should_remove = true;
                        break;
                    }
                }
            }
        }
        
        // 3. 清洗距离过远或特定区域以外的数据
        if (!should_remove) {
            double dis = count_distance(lat_cam, lon_cam, it->lat, it->lon);
            if (dis > max_dis_ || data_filter(*it, camera_pos_para) == DataFilterFlag::AIS_DEL) {
                should_remove = true;
            }
        }
        
        if (should_remove) {
            it = ais_current.erase(it);
        } else {
            ++it;
        }
    }
}

void AISPRO::data_pred(std::vector<AISData>& ais_cur,
                       const std::vector<AISData>& ais_read,
                       const std::vector<AISData>& ais_las,
                       int64_t timestamp_ms) {
    int64_t timestamp_sec = timestamp_ms / 1000;
    
    // 处理ais_read中的数据
    for (const auto& ais : ais_read) {
        int64_t ais_timestamp_sec = ais.timestamp_ms / 1000;
        
        if (ais_timestamp_sec == timestamp_sec) {
            // 当前时刻存在，不推算
            ais_cur.push_back(ais);
        } else {
            // 当前时刻不存在，推算
            ais_cur.push_back(data_pre(ais, timestamp_sec));
        }
    }
    
    // 处理ais_las中未出现的船舶数据
    for (const auto& ais : ais_las) {
        bool found = false;
        for (const auto& cur : ais_cur) {
            if (cur.mmsi == ais.mmsi) {
                found = true;
                break;
            }
        }
        if (!found) {
            ais_cur.push_back(data_pre(ais, timestamp_sec));
        }
    }
}

void AISPRO::transform(const std::vector<AISData>& ais_current,
                       std::vector<AISVisData>& ais_vis,
                       const CameraPosPara& camera_pos_para,
                       const std::string& camera_type,
                       std::vector<AISVisData>& ais_vis_current) {
    ais_vis_current.clear();
    
    for (const auto& ais : ais_current) {
        DataFilterFlag flag = data_filter(ais, camera_pos_para);
        
        if (flag == DataFilterFlag::TRANSFORM) {
            // 坐标转换
            auto [x, y] = visual_transform(ais.lon, ais.lat, camera_pos_para, im_shape_, camera_type);
            
            AISVisData vis_data;
            vis_data.mmsi = ais.mmsi;
            vis_data.lon = ais.lon;
            vis_data.lat = ais.lat;
            vis_data.speed = ais.speed;
            vis_data.course = ais.course;
            vis_data.heading = ais.heading;
            vis_data.type = ais.type;
            vis_data.x = x;
            vis_data.y = y;
            vis_data.timestamp_ms = ais.timestamp_ms / 1000;  // 转换为秒
            
            ais_vis_current.push_back(vis_data);
        } else if (flag == DataFilterFlag::VIS_TRAJ_DEL || flag == DataFilterFlag::AIS_DEL) {
            // 删除对应的视觉轨迹数据
            ais_vis.erase(
                std::remove_if(ais_vis.begin(), ais_vis.end(),
                    [&ais](const AISVisData& vis) { return vis.mmsi == ais.mmsi; }),
                ais_vis.end());
        }
    }
}

void AISPRO::data_tran(const std::vector<AISData>& ais_cur,
                       std::vector<AISVisData>& ais_vis,
                       const CameraPosPara& camera_pos_para,
                       int64_t timestamp_ms,
                       const std::string& camera_type) {
    std::vector<AISVisData> ais_vis_cur;
    transform(ais_cur, ais_vis, camera_pos_para, camera_type, ais_vis_cur);
    
    // 添加到ais_vis
    ais_vis.insert(ais_vis.end(), ais_vis_cur.begin(), ais_vis_cur.end());
    
    // 删除时间过长的AIS数据（时间以2分钟为限）
    int64_t timestamp_sec = timestamp_ms / 1000;
    int64_t time_limit = timestamp_sec - time_lim_ * 60;
    
    ais_vis.erase(
        std::remove_if(ais_vis.begin(), ais_vis.end(),
            [time_limit](const AISVisData& vis) { return vis.timestamp_ms < time_limit; }),
        ais_vis.end());
}

void AISPRO::ais_pro(std::vector<AISData>& ais_cur,
                     const std::vector<AISData>& ais_las,
                     std::vector<AISVisData>& ais_vis,
                     const CameraPosPara& camera_pos_para,
                     const std::vector<AISData>& aisbatch_cache,
                     int64_t timestamp_ms,
                     const std::string& camera_type) {
    // 1. 数据粗清洗
    std::vector<AISData> ais_read = aisbatch_cache;
    data_coarse_process(ais_read, ais_las, camera_pos_para);
    
    // 2. 对未出现的AIS数据推算
    data_pred(ais_cur, ais_read, ais_las, timestamp_ms);
    
    // 3. 坐标转换
    data_tran(ais_cur, ais_vis, camera_pos_para, timestamp_ms, camera_type);
}

void AISPRO::process(const std::vector<AISData>& aisbatch_cache,
                     const CameraPosPara& camera_pos_para,
                     int64_t timestamp_ms,
                     const std::string& camera_type,
                     std::vector<AISVisData>& ais_vis_out,
                     std::vector<AISData>& ais_cur_out) {
    // 1. 参数初始化
    std::vector<AISData> ais_cur, ais_las;
    std::vector<AISVisData> ais_vis;
    initialization(ais_cur, ais_las, ais_vis);
    
    // 2. 数据生成
    ais_pro(ais_cur, ais_las, ais_vis, camera_pos_para, aisbatch_cache, timestamp_ms, camera_type);
    
    // 3. 更新内部状态
    ais_cur_ = ais_cur;
    ais_vis_ = ais_vis;
    
    // 4. 输出结果
    ais_vis_out = ais_vis;
    ais_cur_out = ais_cur;
}

