#include "marnav_vis_cpp/VIS_utils.h"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <limits>

namespace marnav_vis_cpp {
namespace vis_utils {

// ============================================================
// 工具函数实现
// ============================================================

// 判断边界框中心点是否在区域内
int box_whether_in_area(const std::vector<float>& bounding_box, 
                        const std::vector<float>& area) {
    // 计算边界框的中心点
    float x_center = (bounding_box[0] + bounding_box[2]) / 2.0f;
    float y_center = (bounding_box[1] + bounding_box[3]) / 2.0f;
    
    // 构造包含虚拟ID的区域向量（为了复用whether_in_area函数）
    std::vector<float> area_with_id = {1.0f, area[0], area[1], area[2], area[3]};
    std::vector<float> point = {x_center, y_center};
    
    return whether_in_area(point, area_with_id);
}

// 提取两个轨迹点之间的速度
std::vector<float> speed_extract(const VisTrajectory& last_traj, 
                                 const VisTrajectory& now_traj) {
    // 计算时间差（秒）
    int64_t time_diff = now_traj.timestamp - last_traj.timestamp;
    if (time_diff == 0) {
        return {0.0f, 0.0f};
    }
    
    // 计算位移
    float x_displacement = static_cast<float>(now_traj.x - last_traj.x);
    float y_displacement = static_cast<float>(now_traj.y - last_traj.y);
    
    // 计算速度（像素/秒）
    float x_speed = x_displacement / static_cast<float>(time_diff);
    float y_speed = y_displacement / static_cast<float>(time_diff);
    
    return {x_speed, y_speed};
}

// 判断点是否在边界框内
int whether_in_area(const std::vector<float>& point, 
                    const std::vector<float>& bbox) {
    // bbox格式: [id, x1, y1, x2, y2]
    if (bbox.size() < 5) return 0;
    
    float x = point[0];
    float y = point[1];
    float x1 = bbox[1];
    float y1 = bbox[2];
    float x2 = bbox[3];
    float y2 = bbox[4];
    
    // 判断点是否在矩形内
    if (x >= x1 && x <= x2 && y >= y1 && y <= y2) {
        return 1;
    }
    return 0;
}

// 判断两个矩形是否重叠，并计算重叠比例
int overlap(const std::vector<float>& box1, 
            const std::vector<float>& box2, 
            float val) {
    // box格式: [x1, y1, x2, y2]
    float minx1 = box1[0], miny1 = box1[1], maxx1 = box1[2], maxy1 = box1[3];
    float minx2 = box2[0], miny2 = box2[1], maxx2 = box2[2], maxy2 = box2[3];
    
    // 计算交集区域
    float minx = std::max(minx1, minx2);
    float miny = std::max(miny1, miny2);
    float maxx = std::min(maxx1, maxx2);
    float maxy = std::min(maxy1, maxy2);
    
    // 如果没有交集
    if (minx > maxx || miny > maxy) {
        return 0;
    }
    
    // 计算交集面积
    float cross_area = (maxx - minx) * (maxy - miny);
    
    // 计算两个box的面积
    float box1_area = (maxx1 - minx1) * (maxy1 - miny1);
    float box2_area = (maxx2 - minx2) * (maxy2 - miny2);
    
    // 判断交集面积占比是否超过阈值
    if (box1_area > 0 && (cross_area / box1_area > val)) {
        return 1;
    }
    if (box2_area > 0 && (cross_area / box2_area > val)) {
        return 1;
    }
    
    return 0;
}

// 判断目标是否被遮挡
void whether_occlusion(const std::vector<float>& bbox,
                       const std::vector<std::vector<float>>& cur_bbox_list,
                       float val,
                       std::vector<std::vector<float>>& occlusion_bbox_list,
                       std::vector<int>& occlusion_id_list) {
    // bbox格式: [id, x1, y1, x2, y2]
    // cur_bbox_list格式: [[id, x1, y1, x2, y2], ...]
    
    occlusion_bbox_list.clear();
    occlusion_id_list.clear();
    
    int current_id = static_cast<int>(bbox[0]);
    std::vector<float> current_box = {bbox[1], bbox[2], bbox[3], bbox[4]};
    
    for (const auto& other_bbox : cur_bbox_list) {
        int other_id = static_cast<int>(other_bbox[0]);
        std::vector<float> other_box = {other_bbox[1], other_bbox[2], other_bbox[3], other_bbox[4]};
        
        // 检查是否重叠
        if (overlap(current_box, other_box, val)) {
            // 第一次发现遮挡时，添加当前box
            if (occlusion_id_list.empty()) {
                occlusion_id_list.push_back(current_id);
                occlusion_bbox_list.push_back(current_box);
            }
            // 添加遮挡的box
            occlusion_bbox_list.push_back(other_box);
            occlusion_id_list.push_back(other_id);
            break;  // 只检查第一个遮挡
        }
    }
}

// 判断点是否在遮挡区域内
int whether_in_OAR(const std::vector<float>& point,
                   const std::vector<std::vector<float>>& oar_list) {

    for (const auto& oar : oar_list) {
        // oar格式: [x1, y1, x2, y2]
        // 构造包含虚拟ID的格式
        std::vector<float> oar_with_id = {0.0f, oar[0], oar[1], oar[2], oar[3]};
        if (whether_in_area(point, oar_with_id)) {
            return 1;
        }
    }
    return 0;
}

// 提取遮挡区域
void OAR_extractor(const std::vector<std::vector<VisTrajectory>>& his_traj_list,
                   float val,
                   std::vector<std::vector<float>>& oar_list,
                   std::vector<int>& oar_id_list) {
    oar_list.clear();
    oar_id_list.clear();
    
    // 如果没有历史数据，直接返回
    if (his_traj_list.empty()) {
        return;
    }
    
    // 获取上一时刻的轨迹数据
    const auto& last_traj_list = his_traj_list.back();
    
    // 构建边界框列表
    std::vector<std::vector<float>> his_bbox_list;
    std::vector<int> his_id_list;
    
    for (const auto& traj : last_traj_list) {
        his_id_list.push_back(traj.id);
        his_bbox_list.push_back({
            static_cast<float>(traj.id),
            static_cast<float>(traj.x1),
            static_cast<float>(traj.y1),
            static_cast<float>(traj.x2),
            static_cast<float>(traj.y2)
        });
    }
    
    // 检查每个bbox是否与其他bbox有遮挡
    for (size_t i = 0; i < his_bbox_list.size(); ++i) {
        if (i < his_bbox_list.size() - 1) {
            std::vector<std::vector<float>> remaining_bboxes(
                his_bbox_list.begin() + i + 1, his_bbox_list.end());
            
            std::vector<std::vector<float>> occlusion_boxes;
            std::vector<int> occlusion_ids;
            
            whether_occlusion(his_bbox_list[i], remaining_bboxes, val,
                            occlusion_boxes, occlusion_ids);
            
            // 添加遮挡区域和ID（避免重复）
            for (size_t j = 0; j < occlusion_ids.size(); ++j) {
                int id = occlusion_ids[j];
                // 检查ID是否已存在
                if (std::find(oar_id_list.begin(), oar_id_list.end(), id) == oar_id_list.end() &&
                    std::find(his_id_list.begin(), his_id_list.end(), id) != his_id_list.end()) {
                    oar_list.push_back(occlusion_boxes[j]);
                    oar_id_list.push_back(id);
                }
            }
        }
    }
}

// 提取运动特征（速度）
std::vector<VisTrajectory> motion_features_extraction(
    const std::vector<std::vector<VisTrajectory>>& his_traj_list,
    const std::vector<VisTrajectory>& vis_tra_cur) {

    std::vector<VisTrajectory> result = vis_tra_cur;
    
    // 如果没有历史数据，速度全部设为"[0, 0]"
    if (his_traj_list.empty()) {
        return result;
    }
    
    // 为每个当前轨迹计算速度
    for (auto& cur_traj : result) {
        int cur_id = cur_traj.id;
        bool found = false;
        
        // 从最近的历史数据开始查找
        for (const auto& his_traj_vec : his_traj_list) {
            // 在这个时刻的轨迹列表中查找匹配的ID
            for (const auto& his_traj : his_traj_vec) {
                if (his_traj.id == cur_id) {
                    // 计算速度
                    std::vector<float> speed = speed_extract(his_traj, cur_traj);
                    
                    // 转换为字符串格式 "[x_speed, y_speed]"
                    std::ostringstream oss;
                    oss << "[" << speed[0] << ", " << speed[1] << "]";
                    cur_traj.speed = oss.str();
                    
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        
        // 如果没找到历史数据，保持默认值"[0, 0]"
    }
    
    return result;
}

// 判断某一ID是否在过去五秒内的视觉轨迹内存在
bool id_whether_stable(int id, 
                       const std::vector<std::vector<VisTrajectory>>& last_5_trajs) {
    // 检查ID是否在所有5个时刻都存在
    for (const auto& traj_vec : last_5_trajs) {
        bool found = false;
        for (const auto& traj : traj_vec) {
            if (traj.id == id) {
                found = true;
                break;
            }
        }
        if (!found) {
            return false;
        }
    }
    return true;
}

// ============================================================
// VISPRO类实现
// ============================================================

VISPRO::VISPRO(bool anti, float val, int t, rclcpp::Logger logger, 
        const std::string& model_path, const std::string& classes_path,
        const std::string& cfg_path)
    : anti_(anti), val_(val), t_(t), logger_(logger) {
    // 初始化数据容器
    last5_vis_tra_list_.clear();
    vis_tra_cur_3_.clear();
    vis_tra_cur_.clear();
    vis_tra_.clear();
    vis_tra_last_.clear();
    oar_list_.clear();
    oar_ids_list_.clear();
    oar_mmsi_list_.clear();
    anti_occlusion_traj_.clear();
    
    RCLCPP_INFO(logger_, "VISPRO初始化完成 - 抗遮挡: %s, 阈值: %.2f, 时间间隔: %dms",
                anti_ ? "启用" : "禁用", val_, t_);

    // 初始化目标检测
    yolo_detector_ = std::make_unique<YOLO_ONNX>(model_path, classes_path, 
        640, 640,         // input_shape
        0.1f,             // confidence
        0.5f,             // nms_iou
        true,             // letterbox_image
        true,             // use_cuda
        YOLOType::YOLOv11); // model_type

    // 初始化跟踪模型
    // cfg_ = get_config();
    // cfg_.merge_from_file("/home/tl/RV/src/marnav_vis/deep_sort/configs/deep_sort.yaml");
    // deepsort_ = DeepSort(cfg_.DEEPSORT.REID_CKPT,
    //                     max_dist=cfg_.DEEPSORT.MAX_DIST, min_confidence=cfg_.DEEPSORT.MIN_CONFIDENCE,
    //                     nms_max_overlap=cfg_.DEEPSORT.NMS_MAX_OVERLAP, max_iou_distance=cfg_.DEEPSORT.MAX_IOU_DISTANCE,
    //                     max_age=cfg_.DEEPSORT.MAX_AGE, n_init=cfg_.DEEPSORT.N_INIT, nn_budget=cfg_.DEEPSORT.NN_BUDGET,
    //                     use_cuda=True);
}

std::vector<DetectionBox> VISPRO::detection(const cv::Mat& image) {
    // TODO: 集成YOLO检测模型
    // 参考Python版本使用的YOLO类，可以使用yolo_onnx.cpp中的实现
    std::vector<DetectionBox> bboxes;
    auto detections = yolo_detector_->detect_image(image);
    for (const auto& detection : detections) {
        DetectionBox box;
        box.x1 = detection.x1;
        box.y1 = detection.y1;
        box.x2 = detection.x2;
        box.y2 = detection.y2;
        box.class_name = detection.class_name;
        box.confidence = detection.score;
        bboxes.push_back(box);
    }

    return bboxes;
}

void VISPRO::track(const cv::Mat& image [[maybe_unused]],
                   const std::vector<DetectionBox>& bboxes,
                   const std::vector<DetectionBox>& bboxes_anti_occ,
                   const std::vector<int>& id_list [[maybe_unused]],
                   int64_t timestamp) {
    // TODO: 待DeepSort C++实现完成后补充
    // 
    // 接口说明：
    // 1. 输入参数：
    //    - image: 原始图像（BGR格式）
    //    - bboxes: 正常检测框列表（不含遮挡区域）
    //    - bboxes_anti_occ: 遮挡区域的抗遮挡预测框
    //    - id_list: 抗遮挡预测框对应的ID列表
    //    - timestamp: 时间戳（毫秒，需转换为秒）
    // 
    // 2. 处理流程：
    //    - 将检测框转换为xywh格式（中心点+宽高）
    //    - 提取置信度
    //    - 调用DeepSort的update方法
    //    - 解析跟踪结果，存储到vis_tra_cur_3_
    // 
    // 3. DeepSort调用示例：
    //    std::vector<std::vector<float>> bbox_xywh, bbox_xywh_anti_occ;
    //    std::vector<float> confidences, confidences_anti_occ;
    //    
    //    // 转换检测框格式
    //    for (const auto& box : bboxes) {
    //        float cx = (box.x1 + box.x2) / 2.0f;
    //        float cy = (box.y1 + box.y2) / 2.0f;
    //        float w = box.x2 - box.x1;
    //        float h = box.y2 - box.y1;
    //        bbox_xywh.push_back({cx, cy, w, h});
    //        confidences.push_back(box.confidence);
    //    }
    //    
    //    // 同样处理抗遮挡框
    //    for (const auto& box : bboxes_anti_occ) {
    //        float cx = (box.x1 + box.x2) / 2.0f;
    //        float cy = (box.y1 + box.y2) / 2.0f;
    //        float w = box.x2 - box.x1;
    //        float h = box.y2 - box.y1;
    //        bbox_xywh_anti_occ.push_back({cx, cy, w, h});
    //        confidences_anti_occ.push_back(box.confidence);
    //    }
    //    
    //    // 调用DeepSort
    //    auto outputs = deepsort_->update(bbox_xywh, confidences, image,
    //                                     bbox_xywh_anti_occ, confidences_anti_occ,
    //                                     id_list, timestamp / 1000);
    //    
    //    // 解析输出，存储到vis_tra_cur_3_
    //    for (const auto& output : outputs) {
    //        VisTrajectory traj;
    //        traj.id = output.track_id;
    //        traj.x1 = static_cast<int>(output.x1);
    //        traj.y1 = static_cast<int>(output.y1);
    //        traj.x2 = static_cast<int>(output.x2);
    //        traj.y2 = static_cast<int>(output.y2);
    //        traj.x = (traj.x1 + traj.x2) / 2;
    //        traj.y = (traj.y1 + traj.y2) / 2;
    //        traj.timestamp = timestamp / 1000;  // 转换为秒
    //        vis_tra_cur_3_.push_back(traj);
    //    }
    
    RCLCPP_DEBUG(logger_, "[VIS DEBUG] track函数调用 - 检测框: %zu, 抗遮挡框: %zu, timestamp: %ld",
                 bboxes.size(), bboxes_anti_occ.size(), timestamp);
    
    // 预留接口：当前仅记录调用，不实际处理
    // 实际的跟踪结果会存储到vis_tra_cur_3_，待DeepSort实现后补充
}

std::vector<VisTrajectory> VISPRO::update_tra(int64_t timestamp) {
    // 核心逻辑："求均值"
    // 1. 对vis_tra_cur_3_中同一ID的多帧数据求均值
    // 2. 提取运动特征（速度）
    // 3. 更新历史轨迹列表
    
    vis_tra_cur_.clear();
    
    // 提取所有唯一的ID
    std::vector<int> unique_ids;
    for (const auto& traj : vis_tra_cur_3_) {
        if (std::find(unique_ids.begin(), unique_ids.end(), traj.id) == unique_ids.end()) {
            unique_ids.push_back(traj.id);
        }
    }
    
    int64_t timestamp_sec = timestamp / 1000;  // 转换为秒
    
    // 对每个ID计算均值
    for (int id : unique_ids) {
        int count = 0;
        int sum_x1 = 0, sum_y1 = 0, sum_x2 = 0, sum_y2 = 0;
        int sum_x = 0, sum_y = 0;
        
        for (const auto& traj : vis_tra_cur_3_) {
            if (traj.id == id) {
                sum_x1 += traj.x1;
                sum_y1 += traj.y1;
                sum_x2 += traj.x2;
                sum_y2 += traj.y2;
                sum_x += traj.x;
                sum_y += traj.y;
                count++;
            }
        }
        
        if (count > 0) {
            VisTrajectory avg_traj;
            avg_traj.id = id;
            avg_traj.x1 = sum_x1 / count;
            avg_traj.y1 = sum_y1 / count;
            avg_traj.x2 = sum_x2 / count;
            avg_traj.y2 = sum_y2 / count;
            avg_traj.x = sum_x / count;
            avg_traj.y = sum_y / count;
            avg_traj.timestamp = timestamp_sec;
            avg_traj.speed = "[0, 0]";  // 初始速度
            
            vis_tra_cur_.push_back(avg_traj);
        }
    }
    
    // 清空当前秒的累积数据
    vis_tra_cur_3_.clear();
    
    // 提取运动特征（速度）
    std::vector<VisTrajectory> vis_tra_cur_withfeature = 
        motion_features_extraction(last5_vis_tra_list_, vis_tra_cur_);
    
    // 更新历史轨迹
    for (const auto& traj : vis_tra_cur_withfeature) {
        vis_tra_.push_back(traj);
    }
    
    // 更新过去5秒的轨迹列表
    if (last5_vis_tra_list_.size() > 4) {
        last5_vis_tra_list_.erase(last5_vis_tra_list_.begin());
    }
    last5_vis_tra_list_.push_back(vis_tra_cur_withfeature);
    
    // 删除2分钟前的数据
    int time_limit = 2;  // 分钟
    int64_t cutoff_time = timestamp_sec - time_limit * 60;
    
    vis_tra_.erase(
        std::remove_if(vis_tra_.begin(), vis_tra_.end(),
                      [cutoff_time](const VisTrajectory& traj) {
                          return traj.timestamp < cutoff_time;
                      }),
        vis_tra_.end()
    );
    
    RCLCPP_DEBUG(logger_, "[VIS DEBUG] update_tra完成 - vis_tra size: %zu, vis_tra_cur size: %zu",
                 vis_tra_.size(), vis_tra_cur_withfeature.size());
    
    if (!vis_tra_.empty()) {
        // 统计每个ID的轨迹点数
        std::map<int, int> id_counts;
        for (const auto& traj : vis_tra_) {
            id_counts[traj.id]++;
        }
        
        std::ostringstream oss;
        oss << "vis_tra ID分布: ";
        for (const auto& pair : id_counts) {
            oss << "ID" << pair.first << ":" << pair.second << "点 ";
        }
        RCLCPP_DEBUG(logger_, "[VIS DEBUG] %s", oss.str().c_str());
    }
    
    if (vis_tra_cur_withfeature.empty()) {
        RCLCPP_DEBUG(logger_, "[VIS DEBUG] ⚠️ vis_tra_cur为空！");
    }
    
    return vis_tra_cur_withfeature;
}

// 预测轨迹通过视觉速度预测
VisTrajectory VISPRO::traj_prediction_via_visual(const VisTrajectory& last_traj,
                                                 int64_t timestamp,
                                                 const std::vector<float>& speed) {
    VisTrajectory predicted = last_traj;
    
    // 计算时间差
    int64_t time_diff = timestamp - last_traj.timestamp;
    
    // 计算位移
    int x_move = static_cast<int>(time_diff * speed[0]);
    int y_move = static_cast<int>(time_diff * speed[1]);
    
    // 更新所有坐标
    predicted.x += x_move;
    predicted.x1 += x_move;
    predicted.x2 += x_move;
    predicted.y += y_move;
    predicted.y1 += y_move;
    predicted.y2 += y_move;
    predicted.timestamp = timestamp;
    
    return predicted;
}

// 抗遮挡处理函数
std::vector<DetectionBox> VISPRO::anti_occ(
    const std::vector<std::vector<VisTrajectory>>& last5_vis_tra_list,
    std::vector<DetectionBox>& bboxes,
    const std::vector<AISVisData>& ais_vis,
    const std::vector<BindInfo>& bind_inf,
    int64_t timestamp) {
    
    std::vector<DetectionBox> bboxes_anti_occ;
    
    if (oar_list_.empty()) {
        return bboxes_anti_occ;
    }
    
    // 1. 删除处在OAR内的检测结果
    std::vector<size_t> indices_to_remove;
    for (size_t i = 0; i < bboxes.size(); ++i) {
        std::vector<float> bbox_vec = {bboxes[i].x1, bboxes[i].y1, 
                                       bboxes[i].x2, bboxes[i].y2};
        for (const auto& oar : oar_list_) {
            if (box_whether_in_area(bbox_vec, oar)) {
                indices_to_remove.push_back(i);
                break;
            }
        }
    }
    
    // 从后向前删除（避免索引问题）
    for (auto it = indices_to_remove.rbegin(); it != indices_to_remove.rend(); ++it) {
        bboxes.erase(bboxes.begin() + *it);
    }
    
    // 2. 提取所有遮挡ID的MMSI
    oar_mmsi_list_.clear();
    for (int oar_id : oar_ids_list_) {
        bool found = false;
        for (const auto& bind : bind_inf) {
            if (bind.id == oar_id) {
                oar_mmsi_list_.push_back({oar_id, bind.mmsi});
                found = true;
                break;
            }
        }
        if (!found) {
            oar_mmsi_list_.push_back({oar_id, 0});  // 没有MMSI
        }
    }
    
    // 3. 预测bbox位置
    std::vector<size_t> indices_to_remove_from_oar;
    
    for (size_t k = 0; k < oar_mmsi_list_.size(); ++k) {
        int oar_id = oar_mmsi_list_[k].first;
        uint32_t mmsi = oar_mmsi_list_[k].second;
        
        bool final_find_flag = false;
        bool second_final_find_flag = false;
        std::vector<float> final_pos;
        std::vector<float> second_final_pos;
        
        // 3.1 若存在MMSI，使用AIS预测
        if (mmsi != 0) {
            // 从AIS数据中查找对应的MMSI
            for (int i = static_cast<int>(ais_vis.size()) - 1; i >= 0; --i) {
                if (ais_vis[i].mmsi == mmsi && ais_vis[i].timestamp == timestamp - 1) {
                    final_find_flag = true;
                    final_pos = {static_cast<float>(ais_vis[i].x), 
                                static_cast<float>(ais_vis[i].y)};
                }
                else if (ais_vis[i].mmsi == mmsi && ais_vis[i].timestamp == timestamp - 2) {
                    second_final_find_flag = true;
                    second_final_pos = {static_cast<float>(ais_vis[i].x), 
                                       static_cast<float>(ais_vis[i].y)};
                }
                
                if (final_find_flag && second_final_find_flag) {
                    // 计算位移
                    float x_motion = final_pos[0] - second_final_pos[0];
                    float y_motion = final_pos[1] - second_final_pos[1];
                    
                    // 应用到抗遮挡轨迹上
                    if (k < anti_occlusion_traj_.size()) {
                        DetectionBox pred_box;
                        pred_box.x1 = anti_occlusion_traj_[k].x1 + x_motion;
                        pred_box.y1 = anti_occlusion_traj_[k].y1 + y_motion;
                        pred_box.x2 = anti_occlusion_traj_[k].x2 + x_motion;
                        pred_box.y2 = anti_occlusion_traj_[k].y2 + y_motion;
                        pred_box.class_name = "vessel";
                        pred_box.confidence = 1.0f;
                        bboxes_anti_occ.push_back(pred_box);
                    }
                    break;
                }
            }
        }
        // 3.2 若不存在MMSI，使用视觉预测
        else {
            // 检查ID是否稳定
            if (!id_whether_stable(oar_id, last5_vis_tra_list)) {
                indices_to_remove_from_oar.push_back(k);
                continue;
            }
            
            // 从5秒前的轨迹中查找
            if (!last5_vis_tra_list.empty()) {
                const auto& first_traj_list = last5_vis_tra_list[0];
                
                for (const auto& traj : first_traj_list) {
                    if (traj.id == oar_id) {
                        // 解析速度字符串
                        std::string speed_str = traj.speed;
                        // 去掉括号，解析数字
                        speed_str.erase(std::remove(speed_str.begin(), speed_str.end(), '['), speed_str.end());
                        speed_str.erase(std::remove(speed_str.begin(), speed_str.end(), ']'), speed_str.end());
                        
                        std::istringstream iss(speed_str);
                        std::string token;
                        std::vector<float> speed;
                        while (std::getline(iss, token, ',')) {
                            speed.push_back(std::stof(token));
                        }
                        
                        if (speed.size() == 2) {
                            // 预测轨迹
                            VisTrajectory predicted = traj_prediction_via_visual(traj, timestamp, speed);
                            
                            // 生成抗遮挡检测框
                            DetectionBox pred_box;
                            pred_box.x1 = predicted.x1;
                            pred_box.y1 = predicted.y1;
                            pred_box.x2 = predicted.x2;
                            pred_box.y2 = predicted.y2;
                            pred_box.class_name = "vessel";
                            pred_box.confidence = 1.0f;
                            bboxes_anti_occ.push_back(pred_box);
                        }
                        
                        break;
                    }
                }
            } else {
                indices_to_remove_from_oar.push_back(k);
            }
        }
    }
    
    // 4. 删除既没有5秒历史又没有AIS的目标
    for (auto it = indices_to_remove_from_oar.rbegin(); 
         it != indices_to_remove_from_oar.rend(); ++it) {
        size_t idx = *it;
        if (idx < oar_mmsi_list_.size()) {
            oar_mmsi_list_.erase(oar_mmsi_list_.begin() + idx);
        }
        if (idx < oar_ids_list_.size()) {
            oar_ids_list_.erase(oar_ids_list_.begin() + idx);
        }
        if (idx < oar_list_.size()) {
            oar_list_.erase(oar_list_.begin() + idx);
        }
    }
    
    // 检查一致性
    if (oar_ids_list_.size() != bboxes_anti_occ.size()) {
        RCLCPP_WARN(logger_, "[VIS DEBUG] OAR ID列表和抗遮挡框数量不一致: %zu vs %zu",
                    oar_ids_list_.size(), bboxes_anti_occ.size());
    }
    
    return bboxes_anti_occ;
}

std::pair<std::vector<VisTrajectory>, std::vector<VisTrajectory>> 
VISPRO::feedCap(const cv::Mat& image,
                const std::vector<AISVisData>& ais_vis,
                const std::vector<BindInfo>& bind_inf,
                int64_t timestamp) {
    // 主处理流程
    // 对应Python: def feedCap(self, image, AIS_vis, bind_inf, timestamp)
    
    int64_t timestamp_sec = timestamp / 1000;  // 转换为秒
    
    // 1. 目标检测
    std::vector<DetectionBox> bboxes = detection(image);
    RCLCPP_DEBUG(logger_, "[VIS DEBUG] timestamp=%ld, 检测到 %zu 个目标",
                 timestamp, bboxes.size());
    
    // 2. 抗遮挡处理
    std::vector<DetectionBox> bboxes_anti_occ = anti_occ(
        last5_vis_tra_list_, bboxes, ais_vis, bind_inf, timestamp_sec);
    
    // 3. DeepSORT跟踪（每帧调用）
    track(image, bboxes, bboxes_anti_occ, oar_ids_list_, timestamp);
    
    RCLCPP_DEBUG(logger_, "[VIS DEBUG] track后 vis_tra_cur_3 size: %zu",
                 vis_tra_cur_3_.size());
    
    // 4. 轨迹数据更新（每秒调用一次）
    std::vector<VisTrajectory> vis_tra_cur;
    // 判断是否到了新的一秒（通过时间戳的秒级变化）
    // Python版本: if timestamp % 1000 < self.t
    if (static_cast<int64_t>(timestamp % 1000) < static_cast<int64_t>(t_)) {
        RCLCPP_DEBUG(logger_, "[VIS DEBUG] update_tra前 vis_tra_cur_3 size: %zu",
                     vis_tra_cur_3_.size());
        
        vis_tra_cur = update_tra(timestamp);
        
        RCLCPP_DEBUG(logger_, "[VIS DEBUG] update_tra后 vis_tra size: %zu, vis_tra_cur size: %zu",
                     vis_tra_.size(), vis_tra_cur.size());
    } else {
        vis_tra_cur = vis_tra_cur_;  // 使用上次的结果
    }
    
    // 5. 提取遮挡区域（如果启用抗遮挡）
    if (anti_) {
        OAR_extractor(last5_vis_tra_list_, val_, oar_list_, oar_ids_list_);
        
        if (!oar_ids_list_.empty()) {
            std::ostringstream oss;
            oss << "OAR_id_list: ";
            for (int id : oar_ids_list_) {
                oss << id << " ";
            }
            RCLCPP_DEBUG(logger_, "[VIS DEBUG] %s", oss.str().c_str());
        }
    }
    
    // 6. 更新VIS_tra_last（用于下次的抗遮挡处理）
    vis_tra_last_ = vis_tra_cur;
    
    // 7. 更新被遮挡ID对应的轨迹数据
    anti_occlusion_traj_.clear();
    for (int oar_id : oar_ids_list_) {
        for (const auto& traj : vis_tra_last_) {
            if (traj.id == oar_id) {
                anti_occlusion_traj_.push_back(traj);
                break;
            }
        }
    }
    
    // 返回：<所有历史轨迹, 当前秒轨迹>
    return {vis_tra_, vis_tra_cur};
}

} // namespace vis_utils
} // namespace marnav_vis_cpp

