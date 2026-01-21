// 羽化去拼缝线的版本

#include "JH_FeatherStitcher.hpp"
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <shared_mutex>
#include <rclcpp/rclcpp.hpp>
// #include "opencv2/xfeatures2d/nonfree.hpp" // 引入非自由特征检测模块（如SIFT、SURF等）

using namespace std;
using namespace cv;
using namespace cv::detail;

// 读取鱼眼相机标定参数的辅助函数
namespace {
std::string trimAscii(const std::string& input)
{
    size_t first = input.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    size_t last = input.find_last_not_of(" \t\r\n");
    return input.substr(first, last - first + 1);
}

bool parseFisheyeYaml(const std::string& filename, cv::Mat& K, cv::Mat& D)
{
    std::ifstream in(filename);
    if (!in.is_open()) {
        return false;
    }

    enum class Section { None, Camera, Distortion };
    Section section = Section::None;
    bool in_data = false;
    std::vector<double> k_vals;
    std::vector<double> d_vals;

    std::string line;
    while (std::getline(in, line)) {
        std::string s = trimAscii(line);
        if (s.empty()) {
            continue;
        }
        if (s.rfind("camera_matrix:", 0) == 0) {
            section = Section::Camera;
            in_data = false;
            continue;
        }
        if (s.rfind("distortion_coefficients:", 0) == 0) {
            section = Section::Distortion;
            in_data = false;
            continue;
        }
        if (s.rfind("data:", 0) == 0) {
            in_data = true;
            continue;
        }
        if (!in_data) {
            continue;
        }

        size_t dash = s.find('-');
        if (dash == std::string::npos) {
            if (s.find(':') != std::string::npos) {
                in_data = false;
            }
            continue;
        }
        std::string num = trimAscii(s.substr(dash + 1));
        if (num.empty()) {
            continue;
        }
        try {
            double value = std::stod(num);
            if (section == Section::Camera) {
                if (k_vals.size() < 9) {
                    k_vals.push_back(value);
                }
                if (k_vals.size() == 9) {
                    in_data = false;
                }
            } else if (section == Section::Distortion) {
                if (d_vals.size() < 4) {
                    d_vals.push_back(value);
                }
                if (d_vals.size() == 4) {
                    in_data = false;
                }
            }
        } catch (const std::exception&) {
            return false;
        }
    }

    if (k_vals.size() != 9 || d_vals.size() < 4) {
        return false;
    }

    K = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        K.at<double>(i / 3, i % 3) = k_vals[i];
    }
    D = cv::Mat(4, 1, CV_64F);
    for (int i = 0; i < 4; ++i) {
        D.at<double>(i, 0) = d_vals[i];
    }
    return true;
}

bool readFisheyeCalibration(const std::string& filename, cv::Mat& K, cv::Mat& D)
{
    std::cerr << "Reading fisheye calibration from: " << filename << std::endl;
    return parseFisheyeYaml(filename, K, D);
}

const char kRemapDir[] = "/home/tl/RV/src/image_stitching_pkg/config/remap";
const char kCacheFile[] = "/home/tl/RV/src/image_stitching_pkg/config/remap/stitch_cache.yml";

std::string mapPath(const char* base, int idx)
{
    return std::string(kRemapDir) + "/" + base + "_" + std::to_string(idx) + ".exr";
}

std::string featherPath(int idx)
{
    return std::string(kRemapDir) + "/feather_w_" + std::to_string(idx) + ".exr";
}

bool ensureRemapDir()
{
    std::error_code ec;
    if (std::filesystem::exists(kRemapDir, ec)) {
        return true;
    }
    return std::filesystem::create_directories(kRemapDir, ec);
}

bool saveRemapMaps(const TransformationData& data, int num_images)
{
    if (!ensureRemapDir()) {
        return false;
    }
    if (data.mapIU_x.size() != static_cast<size_t>(num_images) ||
        data.mapIU_y.size() != static_cast<size_t>(num_images) ||
        data.mapPU_x.size() != static_cast<size_t>(num_images) ||
        data.mapPU_y.size() != static_cast<size_t>(num_images)) {
        return false;
    }

    for (int i = 0; i < num_images; ++i) {
        if (data.mapIU_x[i].empty() || data.mapIU_y[i].empty() ||
            data.mapPU_x[i].empty() || data.mapPU_y[i].empty()) {
            return false;
        }
        if (!cv::imwrite(mapPath("mapIU_x", i), data.mapIU_x[i])) {
            return false;
        }
        if (!cv::imwrite(mapPath("mapIU_y", i), data.mapIU_y[i])) {
            return false;
        }
        if (!cv::imwrite(mapPath("mapPU_x", i), data.mapPU_x[i])) {
            return false;
        }
        if (!cv::imwrite(mapPath("mapPU_y", i), data.mapPU_y[i])) {
            return false;
        }
    }
    return true;
}

bool saveFeatherWeights(const TransformationData& data, int num_images)
{
    if (!ensureRemapDir()) {
        return false;
    }
    if (data.feather_weights.size() != static_cast<size_t>(num_images)) {
        return false;
    }
    for (int i = 0; i < num_images; ++i) {
        if (data.feather_weights[i].empty()) {
            return false;
        }
        if (!cv::imwrite(featherPath(i), data.feather_weights[i])) {
            return false;
        }
    }
    return true;
}

bool loadRemapMaps(int num_images, TransformationData& data)
{
    data.mapIU_x.clear();
    data.mapIU_y.clear();
    data.mapPU_x.clear();
    data.mapPU_y.clear();
    data.mapIU_x.resize(num_images);
    data.mapIU_y.resize(num_images);
    data.mapPU_x.resize(num_images);
    data.mapPU_y.resize(num_images);

    for (int i = 0; i < num_images; ++i) {
        data.mapIU_x[i] = cv::imread(mapPath("mapIU_x", i), cv::IMREAD_UNCHANGED);
        data.mapIU_y[i] = cv::imread(mapPath("mapIU_y", i), cv::IMREAD_UNCHANGED);
        data.mapPU_x[i] = cv::imread(mapPath("mapPU_x", i), cv::IMREAD_UNCHANGED);
        data.mapPU_y[i] = cv::imread(mapPath("mapPU_y", i), cv::IMREAD_UNCHANGED);

        if (data.mapIU_x[i].empty() || data.mapIU_y[i].empty() ||
            data.mapPU_x[i].empty() || data.mapPU_y[i].empty()) {
            return false;
        }

        if (data.mapIU_x[i].type() != CV_32FC1) {
            data.mapIU_x[i].convertTo(data.mapIU_x[i], CV_32FC1);
        }
        if (data.mapIU_y[i].type() != CV_32FC1) {
            data.mapIU_y[i].convertTo(data.mapIU_y[i], CV_32FC1);
        }
        if (data.mapPU_x[i].type() != CV_32FC1) {
            data.mapPU_x[i].convertTo(data.mapPU_x[i], CV_32FC1);
        }
        if (data.mapPU_y[i].type() != CV_32FC1) {
            data.mapPU_y[i].convertTo(data.mapPU_y[i], CV_32FC1);
        }
    }
    return true;
}

bool loadFeatherWeights(int num_images, TransformationData& data)
{
    data.feather_weights.clear();
    data.feather_weights.resize(num_images);
    for (int i = 0; i < num_images; ++i) {
        data.feather_weights[i] = cv::imread(featherPath(i), cv::IMREAD_UNCHANGED);
        if (data.feather_weights[i].empty()) {
            return false;
        }
        if (data.feather_weights[i].type() != CV_32FC1) {
            data.feather_weights[i].convertTo(data.feather_weights[i], CV_32FC1);
        }
    }
    return true;
}

bool saveStitchCache(const TransformationData& data)
{
    if (!ensureRemapDir()) {
        return false;
    }
    cv::FileStorage fs(kCacheFile, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        return false;
    }

    fs << "num_cameras" << static_cast<int>(data.cameras.size());
    fs << "cameras" << "[";
    for (const auto& cam : data.cameras) {
        fs << "{";
        fs << "focal" << cam.focal;
        fs << "aspect" << cam.aspect;
        fs << "ppx" << cam.ppx;
        fs << "ppy" << cam.ppy;
        fs << "R" << cam.R;
        fs << "t" << cam.t;
        fs << "}";
    }
    fs << "]";
    fs << "corners" << data.corners;
    fs << "sizes" << data.sizes;
    fs << "dxdy" << data.dxdy;
    fs << "dst_roi_x" << data.dst_roi.x;
    fs << "dst_roi_y" << data.dst_roi.y;
    fs << "dst_roi_w" << data.dst_roi.width;
    fs << "dst_roi_h" << data.dst_roi.height;
    fs << "crop_roi_x" << data.crop_roi.x;
    fs << "crop_roi_y" << data.crop_roi.y;
    fs << "crop_roi_w" << data.crop_roi.width;
    fs << "crop_roi_h" << data.crop_roi.height;

    fs << "masks" << "[";
    for (const auto& mask : data.masks) {
        fs << mask;
    }
    fs << "]";

    fs << "raw_masks_8u" << "[";
    for (const auto& mask : data.raw_masks_8u) {
        cv::Mat mask_mat = mask.getMat(cv::ACCESS_READ);
        fs << mask_mat;
    }
    fs << "]";

    fs.release();
    return true;
}

bool loadStitchCache(TransformationData& data)
{
    cv::FileStorage fs(kCacheFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }

    int num_cameras = 0;
    fs["num_cameras"] >> num_cameras;
    if (num_cameras <= 0) {
        return false;
    }

    data.cameras.clear();
    cv::FileNode cameras_node = fs["cameras"];
    if (cameras_node.type() != cv::FileNode::SEQ) {
        return false;
    }
    for (auto it = cameras_node.begin(); it != cameras_node.end(); ++it) {
        cv::detail::CameraParams cam;
        cv::FileNode cam_node = *it;
        cam_node["focal"] >> cam.focal;
        cam_node["aspect"] >> cam.aspect;
        cam_node["ppx"] >> cam.ppx;
        cam_node["ppy"] >> cam.ppy;
        cam_node["R"] >> cam.R;
        cam_node["t"] >> cam.t;
        data.cameras.push_back(cam);
    }
    if (static_cast<int>(data.cameras.size()) != num_cameras) {
        return false;
    }

    fs["corners"] >> data.corners;
    fs["sizes"] >> data.sizes;
    fs["dxdy"] >> data.dxdy;

    int dst_x = 0;
    int dst_y = 0;
    int dst_w = 0;
    int dst_h = 0;
    fs["dst_roi_x"] >> dst_x;
    fs["dst_roi_y"] >> dst_y;
    fs["dst_roi_w"] >> dst_w;
    fs["dst_roi_h"] >> dst_h;
    data.dst_roi = cv::Rect(dst_x, dst_y, dst_w, dst_h);

    int crop_x = 0;
    int crop_y = 0;
    int crop_w = 0;
    int crop_h = 0;
    fs["crop_roi_x"] >> crop_x;
    fs["crop_roi_y"] >> crop_y;
    fs["crop_roi_w"] >> crop_w;
    fs["crop_roi_h"] >> crop_h;
    data.crop_roi = cv::Rect(crop_x, crop_y, crop_w, crop_h);

    data.masks.clear();
    cv::FileNode masks_node = fs["masks"];
    if (masks_node.type() == cv::FileNode::SEQ) {
        for (auto it = masks_node.begin(); it != masks_node.end(); ++it) {
            cv::Mat mask;
            (*it) >> mask;
            data.masks.push_back(mask);
        }
    }

    data.raw_masks_8u.clear();
    cv::FileNode raw_masks_node = fs["raw_masks_8u"];
    if (raw_masks_node.type() == cv::FileNode::SEQ) {
        for (auto it = raw_masks_node.begin(); it != raw_masks_node.end(); ++it) {
            cv::Mat mask;
            (*it) >> mask;
            if (!mask.empty()) {
                data.raw_masks_8u.push_back(mask.getUMat(cv::ACCESS_READ));
            }
        }
    }

    float avg_focal = 0.0f;
    for (const auto& cam : data.cameras) {
        avg_focal += static_cast<float>(cam.focal);
    }
    avg_focal /= data.cameras.size();
    data.warper = makePtr<cv::detail::CylindricalWarperGpu>(avg_focal);

    fs.release();
    return true;
}

std::vector<cv::Mat> computeFeatherWeights(const std::vector<cv::Mat>& masks)
{
        // 可视化masks图（调试用）
    for (size_t i = 0; i < masks.size(); ++i) {
        cv::Mat vis;
        masks[i].convertTo(vis, CV_8U, 255.0);
        cv::imwrite("feather_mask_" + std::to_string(i) + ".png", vis);
    }
    std::vector<cv::Mat> weights;
    weights.resize(masks.size());
    for (size_t i = 0; i < masks.size(); ++i) {
        if (masks[i].empty()) {
            continue;
        }
        cv::Mat mask_8u;
        if (masks[i].type() != CV_8U) {
            masks[i].convertTo(mask_8u, CV_8U);
        } else {
            mask_8u = masks[i];
        }
        cv::Mat mask_bin;
        cv::threshold(mask_8u, mask_bin, 0, 255, cv::THRESH_BINARY);

        cv::Mat dist;
        cv::distanceTransform(mask_bin, dist, cv::DIST_L2, 3);
        double max_val = 0.0;
        cv::minMaxLoc(dist, nullptr, &max_val);
        if (max_val <= 0.0) {
            cv::Mat fallback;
            mask_bin.convertTo(fallback, CV_32F, 1.0 / 255.0);
            weights[i] = fallback;
            continue;
        }
        dist.convertTo(dist, CV_32F, 1.0 / max_val);

        cv::Mat mask_f;
        mask_bin.convertTo(mask_f, CV_32F, 1.0 / 255.0);
        cv::multiply(dist, mask_f, weights[i]);
    }

    // 可视化权重图（调试用）
    for (size_t i = 0; i < weights.size(); ++i) {
        cv::Mat vis;
        weights[i].convertTo(vis, CV_8U, 255.0);
        cv::imwrite("feather_weight_" + std::to_string(i) + ".png", vis);
    }    
    return weights;
}

std::vector<cv::Mat> computeFeatherWeightsFromRawMasks(const std::vector<cv::UMat>& raw_masks)
{
    std::vector<cv::Mat> masks;
    masks.reserve(raw_masks.size());
    for (const auto& mask : raw_masks) {
        masks.push_back(mask.getMat(cv::ACCESS_READ));
    }
    return computeFeatherWeights(masks);
}
} // namespace

JHStitcher::JHStitcher(
    std::unordered_map<std::string, int>& cam_name_to_idx,
    int refresh_time,
    size_t min_keypoints,
    double min_confidence,
    int min_inliers,
    double max_focal_variance,
    double y_tolerance,
    float roi_threshold,
    double scale,
    bool cropornot,
    bool drawboxornot,
    bool save_CameraParams,
    string save_CameraParams_path,
    bool use_saved_CameraParams,
    double FOV_hor,
    double FOV_ver
):  
    cam_name_to_idx_(cam_name_to_idx),
    refresh_time_(refresh_time),
    min_keypoints_(min_keypoints),
    min_confidence_(min_confidence),
    min_inliers_(min_inliers),
    max_focal_variance_(max_focal_variance),
    y_tolerance_(y_tolerance),
    roi_threshold_(roi_threshold),
    scale_(scale),
    cropornot_(cropornot),
    drawboxornot_(drawboxornot),
    save_CameraParams_(save_CameraParams),
    save_CameraParams_path_(save_CameraParams_path),
    use_saved_CameraParams_(use_saved_CameraParams),
    FOV_hor_(FOV_hor),
    FOV_ver_(FOV_ver)

    {
    latest_warp_images_32F.resize(3);
    
    // 此处可临时添加打印，验证初始化是否正确
    // std::cout << "cam_name_to_idx_ 初始化相机映射：" << std::endl;
    // for (const auto& [name, idx] : cam_name_to_idx_) {
    //     std::cout << "名称: " << name << ", 索引: " << idx << std::endl;
    // }
}

bool JHStitcher::processFirstGroupImpl(std::vector<cv::Mat>& images) {
    // =================== 调试 代码：保存输入图像 ===================
    // bool JHStitcher::processFirstGroupImpl(const std::vector<cv::Mat>& images) {
    // 保存输入图片，以便后续调试和检查
    // for (size_t i = 0; i < images.size(); ++i) {
    //     cv::imwrite("input_image_" + std::to_string(i) + ".png", images[i]);
    //     cout << "已保存输入图像: input_image_" << i << ".png" << endl;
    // }
    // ===== 调试: 设置images为现有图片 ==================
    // for(int i = 0; i < images.size(); ++i) {
    //     images[i] = cv::imread("/home/tl/RV/input_image_" + std::to_string(i) + ".png");
    // }
    // =======================================================
    int num_images = images.size();
    cout<<"处理第一组图片，相机数量:"<< num_images << endl;

    TransformationData data;
    if (use_saved_CameraParams_) {
        TransformationData cached;
        if (loadStitchCache(cached) && loadRemapMaps(num_images, cached)) {
            bool size_ok = (static_cast<int>(cached.cameras.size()) == num_images);
            for (int i = 0; i < num_images && size_ok; ++i) {
                if (cached.mapIU_x[i].size() != images[i].size() ||
                    cached.mapIU_y[i].size() != images[i].size()) {
                    size_ok = false;
                }
            }
            if (size_ok) {
                if (!loadFeatherWeights(num_images, cached)) {
                    if (!cached.raw_masks_8u.empty()) {
                        cached.feather_weights = computeFeatherWeightsFromRawMasks(cached.raw_masks_8u);
                    } else {
                        cached.feather_weights = computeFeatherWeights(cached.masks);
                    }
                }
                std::lock_guard<std::mutex> lock(transformation_mutex_);
                transformation_data_ = cached;
                return true;
            }
        }
    }

    // 先用相机标定参数去畸变（fisheye）
    std::vector<cv::Mat> undistorted_images(num_images);
    data.mapIU_x.resize(num_images);
    data.mapIU_y.resize(num_images);
    for (int i = 0; i < num_images; ++i) {
        std::string calib_path = "/home/tl/RV/src/marnav_vis/config/calibration_result_" + std::to_string(i) + ".yaml";
        cv::Mat K, D;
        if (!readFisheyeCalibration(calib_path, K, D)) {
            cout << "无法读取相机标定参数: " << calib_path << endl;
            return false;
        }
        cv::Mat K_new;
        double balance = 1.0; // 0:更裁剪，1:尽量保留视场
        double fov_scale = 1.0;

        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
            K, D, images[i].size(), cv::Mat::eye(3, 3, CV_64F), K_new, balance, images[i].size(), fov_scale
        );

        cv::fisheye::initUndistortRectifyMap(
            K, D, cv::Mat::eye(3, 3, CV_64F), K_new, images[i].size(), CV_32FC1,
            data.mapIU_x[i], data.mapIU_y[i]);

        cv::remap(images[i], undistorted_images[i], data.mapIU_x[i], data.mapIU_y[i],
                  cv::INTER_LINEAR, cv::BORDER_CONSTANT);


        // 保存去畸变后的图像以供检查
        cv::imwrite("undistorted_image_" + std::to_string(i) + ".png", undistorted_images[i]);
        cout << "已保存去畸变后的图像: undistorted_image_" << i << ".png" << endl;


    }

        // 相机参数估计
        std::vector<CameraParams> cameras;
        if(use_saved_CameraParams_ && !save_CameraParams_path_.empty())
        {
            cout<<"使用已保存的相机参数代替相机参数估计" << endl;
            cameras = readCameraParamters(save_CameraParams_path_);
            if(cameras.size() != num_images)
            {
                cout<<"已保存的相机参数数量与当前图像数量不匹配" << endl;
                return false;
            }
        }
        else
        {
// ------------------------------------------------------------------------------------------------------------------------------------------
        // 特征点检测
        std::vector<ImageFeatures> features(num_images);
        std::vector<Size> images_sizes(num_images);
        // 使用SURF特征检测器
        // Ptr<cv::xfeatures2d::SURF> featurefinder = cv::xfeatures2d::SURF::create();
        // 使用AKAZE特征检测器
        Ptr<AKAZE> featurefinder = AKAZE::create();


        for (int i = 0; i < num_images; ++i) {
            images_sizes[i] = undistorted_images[i].size();
            computeImageFeatures(featurefinder, undistorted_images[i], features[i]);
            // 绘制特征点并保存
            // Mat img_with_keypoints;
            // drawKeypoints(undistorted_images[i], features[i].keypoints, img_with_keypoints, 
            //             Scalar::all(-1), DrawMatchesFlags::DEFAULT);
            // 注意：这里修改了内部循环变量名，避免与外部循环冲突
            // imwrite("keypoints_image_" + std::to_string(i) + ".png", img_with_keypoints);


            if (features[i].keypoints.size() < min_keypoints_) {
                cout<<"图像"<<i<<"特征点不足,只有："<<features[i].keypoints.size()<<endl;
                return false;
            }
                cout<<"图像"<<i<<"特征点充足,有："<<features[i].keypoints.size()<<endl;
        }

        // 特征匹配
        // cout<< "开始特征匹配" << endl;
        std::vector<MatchesInfo> pairwise_matches;
        Ptr<FeaturesMatcher> matcher = makePtr<BestOf2NearestRangeMatcher>(1, true, 0.3f, 6, 6);
        // Ptr<FeaturesMatcher> matcher = makePtr<BestOf2NearestRangeMatcher>(1, true, 0.3f, 6, 6);
        (*matcher)(features, pairwise_matches);
            // 执行筛选和置信度检查
        if (!checkAndFilterMatches(pairwise_matches)) {
            // 若检查失败，输出错误信息并退出函数
            // cout<<"特征匹配检查失败"<<endl;
            return false; // 表示失败
        }

            // 显示每两幅图片之间特征点的匹配关系
        for (int i = 0; i < num_images; ++i) {
            for (int j = i + 1; j < num_images; ++j) {
                const MatchesInfo& match_info = pairwise_matches[i * num_images + j];
                if (match_info.matches.size() > 0) {
                    Mat match_img;
                    drawMatches(undistorted_images[i], features[i].keypoints, undistorted_images[j], features[j].keypoints,
                                match_info.matches, match_img);
                    // imwrite("matches_" + std::to_string(i) + "_" + std::to_string(j) + ".png", match_img); // Convert i and j to string
                }
            }
        }
// ------------------------------------------------------------------------------------------------------------------------------------------
            cout<<"开始相机参数估计" << endl;
            Ptr<Estimator> estimator = makePtr<HomographyBasedEstimator>();
            if (!(*estimator)(features, pairwise_matches, cameras)) {
                cout<<"相机参数估计失败"<<endl;
                return false;
            }
            // 检查粗估计的cam合法性
            if (!checkCameraLegitimacy(cameras)) {
            // 若检查失败，输出错误信息并退出函数
                return false;  // 返回空数据表示失败
            }
        
            // 关键修复：在光束平差前转换旋转矩阵为CV_32F
            for (auto& cam : cameras) {
                if (cam.R.type() != CV_32F) {
                    cam.R.convertTo(cam.R, CV_32F);  // 强制转换为32位浮点
                }
            }

            // 光束平差优化
            cout<<"开始光束平差" << endl;
            Ptr<BundleAdjusterBase> adjuster = makePtr<BundleAdjusterRay>();
            try {
                (*adjuster)(features, pairwise_matches, cameras);
            } 
            catch (const cv::Exception& e) {
                cout<<"光束平差执行失败"<<endl;
                return false;
            }

            // 检查BA法的焦距方差合法性
            if (!checkBALegitimacy(cameras)) {
                // 若检查失败，输出错误信息并退出函数
                return false;  // 返回空数据表示失败
            }
            for (size_t i = 0; i < cameras.size(); ++i)
            {
                cv::Mat R;
                cameras[i].R.convertTo(R, CV_32F);
                cameras[i].R = R;
            }


            // 波形矫正
            // cout<<"开始波形矫正" << endl;
            std::vector<Mat> rotations;
            for (size_t i = 0; i < cameras.size(); ++i) {
                rotations.push_back(cameras[i].R);
            }
            waveCorrect(rotations, WAVE_CORRECT_HORIZ);
            for (size_t i = 0; i < cameras.size(); ++i) {
                cameras[i].R = rotations[i];
            }

        }

        // 保存相机参数
        if(save_CameraParams_ && !save_CameraParams_path_.empty())
        {
            saveCameraParamters(save_CameraParams_path_, cameras );
        }

        // 创建掩码（按相机位置生成方向性梯度）
        std::vector<Mat> masks(num_images);
        int border_width = 30; // Standard gradient width (adjustable later)
        for (int i = 0; i < num_images; ++i) {
            masks[i].create(undistorted_images[i].size(), CV_8U);

            Mat mask = Mat::ones(undistorted_images[i].size(), CV_8U) * 255;

            if (i == 0) {
                rectangle(mask,
                          Point(undistorted_images[i].cols - border_width, 0),
                          Point(undistorted_images[i].cols, undistorted_images[i].rows),
                          Scalar(0), -1);
            } else if (i == num_images - 1) {
                rectangle(mask,
                          Point(0, 0),
                          Point(border_width, undistorted_images[i].rows),
                          Scalar(0), -1);
            } else {
                rectangle(mask,
                          Point(0, 0),
                          Point(border_width, undistorted_images[i].rows),
                          Scalar(0), -1);
                rectangle(mask,
                          Point(undistorted_images[i].cols - border_width, 0),
                          Point(undistorted_images[i].cols, undistorted_images[i].rows),
                          Scalar(0), -1);
            }

            Mat dist;
            distanceTransform(mask, dist, DIST_L2, 3);

            double max_val = 0;
            minMaxLoc(dist, nullptr, &max_val);
            if (max_val > 0) {
                dist.convertTo(dist, CV_32F, 1.0 / max_val);
            }

            dist.convertTo(masks[i], CV_8U, 255.0);
        }

        // 图像扭曲
        // cout<<"开始图像扭曲" << endl;
        std::vector<Mat> masks_warp(num_images);
        std::vector<Mat> images_warp(num_images);
        std::vector<Point> corners(num_images);
        std::vector<Size> sizes(num_images);
        data.mapPU_x.resize(num_images);
        data.mapPU_y.resize(num_images);

        vector<Point2f> four_corners(4); //四个角点
        vector<vector<Point2f>> four_corners_warp(num_images); //扭曲图像左角点


        float avg_focal = 0.0f;
        for (const auto& cam : cameras) {
            avg_focal += static_cast<float>(cam.focal);
        }
        avg_focal /= cameras.size();
        Ptr< cv::detail::CylindricalWarperGpu> warper = makePtr<cv::detail::CylindricalWarperGpu>(avg_focal);

        for (int i = 0; i < num_images; ++i) {
            Mat K;
            cameras[i].K().convertTo(K, CV_32F);
            cv::Rect warp_roi = warper->buildMaps(undistorted_images[i].size(), K, cameras[i].R,
                                                  data.mapPU_x[i], data.mapPU_y[i]);
            corners[i] = warp_roi.tl();
            cv::remap(undistorted_images[i], images_warp[i], data.mapPU_x[i], data.mapPU_y[i],
                      INTER_LINEAR, BORDER_REFLECT);
            cv::remap(masks[i], masks_warp[i], data.mapPU_x[i], data.mapPU_y[i],
                      INTER_NEAREST, BORDER_CONSTANT);
            if (images_warp[i].empty() || masks_warp[i].empty()) {
                cout << "变形后图像或掩码为空: idx=" << i << endl;
                return false;
            }
            if (masks_warp[i].size() != images_warp[i].size()) {
                cv::resize(masks_warp[i], masks_warp[i], images_warp[i].size(), 0, 0, cv::INTER_NEAREST);
            }
            sizes[i] = images_warp[i].size();
            // 计算原始图像的四个角点
            four_corners[0] = Point(0, 0); //左上角
            four_corners[1] = Point(undistorted_images[i].cols-1, 0); //右上角
            four_corners[2] = Point(undistorted_images[i].cols-1, undistorted_images[i].rows-1); //右下角
            four_corners[3] = Point(0, undistorted_images[i].rows-1); //左下角

            // 计算投影后的掩码的四个角点
            four_corners_warp[i].resize(4);
            for (int j = 0; j < 4; ++j)
            {
                four_corners_warp[i][j] = warper->warpPoint(four_corners[j], K, cameras[i].R); //扭曲图像左角点
            }
        }
        
        // cout<<"检测左上角合法性" << endl;
        if(!checkTopLeftPointsLegitimacy(four_corners_warp)) {
            // 若检查失败，输出错误信息并退出函数
            std::cerr << "Error: 左上角点位置不合法，可能是图像尺寸过小或变换参数异常。" << std::endl;
            return false;  // 返回空数据表示失败
        }
        cout<<"检测结束,左上角合法性通过" << endl;

        // 曝光补偿
        // cout<<"开始曝光补偿" << endl;
        std::vector<cv::UMat> images_warp_umat(num_images), masks_warp_umat(num_images);
        for (int i = 0; i < num_images; ++i) {
            images_warp[i].copyTo(images_warp_umat[i]);
            masks_warp[i].copyTo(masks_warp_umat[i]);
        }

        // GainCompensator-0.0442372s
        // ChannelsCompensator-0.0487535s
        // BlocksGainCompensator-2.30637s
        // BlocksChannelsCompensator-3.14044s
        Ptr<ExposureCompensator> compensator = makePtr<GainCompensator>();
        compensator->feed(corners, images_warp_umat, masks_warp_umat);

        for (int i = 0; i < num_images; ++i) {
            // Save and Debug:
            imwrite("before_compensate_image_" + std::to_string(i) + ".png", images_warp[i]);
            imwrite("before_compensate_mask_" + std::to_string(i) + ".png", masks_warp[i]);
            // ==========================================================================
            cv::UMat img_umat = images_warp[i].getUMat(ACCESS_RW);
            cv::UMat mask_umat = masks_warp[i].getUMat(ACCESS_RW);
            compensator->apply(i, corners[i], img_umat, mask_umat);
            img_umat.copyTo(images_warp[i]);
            mask_umat.copyTo(masks_warp[i]);
            // Save and Debug:
            imwrite("after_compensate_image_" + std::to_string(i) + ".png", images_warp[i]);
            imwrite("after_compensate_mask_" + std::to_string(i) + ".png", masks_warp[i]);
            // ==========================================================================
        }

        // 缝合线查找
    // 图割法、动态规划缝合线、Voronoi图缝合线查找 GraphCutSeamFinderBase DpSeamFinder VoronoiSeamFinder
        cout<<"开始缝合线查找" << endl;
        // 计算耗时
        auto start_time_match = chrono::high_resolution_clock::now();
        Ptr<SeamFinder> seam_finder = makePtr<DpSeamFinder>(DpSeamFinder::COLOR_GRAD);
        // Ptr<SeamFinder> seam_finder = makePtr<GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
        
        // 确保图像和掩码的类型正确
        data.raw_masks_8u.resize(num_images);
        for (int i = 0; i < num_images; ++i) {
            images_warp[i].convertTo(images_warp_umat[i], CV_32F); // 确保图像数据类型为CV_32F，不然会导致图割法错误。
            masks_warp[i].convertTo(masks_warp_umat[i], CV_8U);   // 确保掩码数据类型为CV_8U，去掉 'U8'，掩码会更新错误。
            data.raw_masks_8u[i] = masks_warp_umat[i].clone(); 
    }

    // 检测图像和掩码的类型是否分别为CV_32F和CV_8U
        try {
            seam_finder->find(images_warp_umat, corners, masks_warp_umat);
        } catch (const Exception& e) {
            cout<< "缝合线查找失败: " << e.what() << endl;
            return false;
        }
        for (int i = 0; i < num_images; ++i) {
            masks_warp_umat[i].copyTo(masks_warp[i]);
        }
        auto end_time_match = chrono::high_resolution_clock::now();
        chrono::duration<double> duration_time_match = end_time_match - start_time_match;
        cout << "缝合线查找耗时: " << duration_time_match.count() << " seconds." << endl;

        // 图像融合（固定 seam + feather 权重）
        if (!data.raw_masks_8u.empty()) {
            data.feather_weights = computeFeatherWeightsFromRawMasks(data.raw_masks_8u);
        } else {
            data.feather_weights = computeFeatherWeights(masks_warp);
        }

        Rect dst_roi = Rect(corners[0], sizes[0]);
        for (size_t i = 1; i < corners.size(); ++i) {
            dst_roi |= Rect(corners[i], sizes[i]);
        }
        data.dst_roi = dst_roi;

        Mat accum = Mat::zeros(dst_roi.size(), CV_32FC3); //在融合过程中，accum 会累积加权后的像素值
        Mat weight_sum = Mat::zeros(dst_roi.size(), CV_32FC1); //累积每个像素点的权重和，用于后续归一化（accum / weight_sum）
        Mat result_mask = Mat::zeros(dst_roi.size(), CV_8U); // 标记哪些区域已被图像覆盖（1=已覆盖，0=未覆盖）, 用于处理重叠区域（避免重复覆盖）

        for (int i = 0; i < num_images; ++i) {
            if (images_warp[i].empty() || masks_warp[i].empty()) {
                cout << "融合前图像或掩码为空: idx=" << i << endl;
                return false;
            }
            if (masks_warp[i].size() != images_warp[i].size()) { // 再次检查掩码和图像尺寸一致性
                cv::resize(masks_warp[i], masks_warp[i], images_warp[i].size(), 0, 0, cv::INTER_NEAREST);
            }
            if (masks_warp[i].type() != CV_8U) { // 确保掩码类型为CV_8U
                masks_warp[i].convertTo(masks_warp[i], CV_8U);
            }
            
            // 获取羽化权重图
            Mat weight = (data.feather_weights.size() == static_cast<size_t>(num_images) &&
                          !data.feather_weights[i].empty())
                             ? data.feather_weights[i] // 条件成立时使用的值 → data.feather_weights[i]
                             : Mat(); // 条件不成立时使用的值 → 空Mat
            if (weight.empty() || weight.size() != images_warp[i].size()) {
                masks_warp[i].convertTo(weight, CV_32F, 1.0 / 255.0);
            }
            // Save and Debug:
            imwrite("feather_weight_used_" + std::to_string(i) + ".png" , weight*255.0);
            // ==========================================================================

            Rect roi_rect(corners[i] - dst_roi.tl(), sizes[i]); // 将当前图像的坐标转换为相对于全景图裁剪区域的坐标
            Rect pano_rect(0, 0, accum.cols, accum.rows); // 全景图的有效区域
            Rect clipped = roi_rect & pano_rect; // 计算裁剪后的有效区域
            if (clipped.empty()) { // 如果没有交集，跳过该图像
                continue;
            }

            Rect src_rect(clipped.x - roi_rect.x, clipped.y - roi_rect.y, clipped.width, clipped.height); // 重叠区域在全景图裁剪区域中的坐标
            Mat accum_roi = accum(clipped); // 全景图裁剪区域的重叠部分
            Mat weight_sum_roi = weight_sum(clipped); // 权重和的重叠部分
            Mat mask_roi = masks_warp[i](src_rect); // 当前图像掩码的重叠部分
            Mat weight_roi = weight(src_rect); // 当前图像权重的重叠部分

            Mat img_roi = images_warp[i](src_rect); // 当前图像的重叠部分
            Mat img_f;
            img_roi.convertTo(img_f, CV_32FC3); // 转换为浮点型以进行加权累加
            Mat weight_3;
            cv::cvtColor(weight_roi, weight_3, cv::COLOR_GRAY2BGR); // 将单通道权重转换为三通道,以匹配图像通道数
            accum_roi += img_f.mul(weight_3); // 加权累加图像
            weight_sum_roi += weight_roi; // 加权累加权重

            // 标记当前图像覆盖的区域（避免后续图像覆盖已处理区域）
            Mat result_mask_roi = result_mask(clipped);
            cv::max(result_mask_roi, mask_roi, result_mask_roi);
        }

        Mat weight_safe; // 如果某个像素未被任何图像覆盖（权重和=0），直接除法会得到NaN（非数字），导致图像损坏。
        cv::max(weight_sum, 1e-6, weight_safe);
        Mat weight_safe_3; // 扩展为三通道
        cv::cvtColor(weight_safe, weight_safe_3, cv::COLOR_GRAY2BGR);
        Mat result_f; // 浮点型结果图
        cv::divide(accum, weight_safe_3, result_f); // result_f = accum / weight_safe_3, 逐元素除法
        Mat result;
        result_f.convertTo(result, images_warp[0].type()); // 将浮点结果转换为原始图像类型

        // 裁剪图像
        // cout<<"开始裁剪图像" << endl;
        Mat crop_mask;
        cv::threshold(result_mask, crop_mask, 0, 255, cv::THRESH_BINARY);
        Rect crop_roi = getValidRows(crop_mask, roi_threshold_);
        if (crop_roi.empty()) {
            cout << "图像裁剪失败: crop_mask empty or no valid rows, mask size="
                 << crop_mask.cols << "x" << crop_mask.rows
                 << ", threshold=" << roi_threshold_ << endl;
            return false;
        }

        data.cameras = cameras;
        data.warper = warper;
        data.compensator = compensator;
        data.seam_finder = seam_finder;
        data.corners = corners;
        data.sizes = sizes;
        data.dst_roi = dst_roi;
        data.masks = masks_warp;
        data.dxdy = prepare_pxpy(data.corners,data.sizes);
        data.crop_roi = Rect(static_cast<int>(crop_roi.x*scale_),static_cast<int>(crop_roi.y*scale_),static_cast<int>(crop_roi.width*scale_),static_cast<int>(crop_roi.height*scale_));

        if (!saveRemapMaps(data, num_images)) {
            cout << "保存remap映射表失败" << endl;
            return false;
        }
        if (!saveFeatherWeights(data, num_images)) {
            cout << "保存feather权重失败" << endl;
            return false;
        }
        if (!saveStitchCache(data)) {
            cout << "保存拼接缓存失败" << endl;
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(transformation_mutex_);
            transformation_data_ = data; //transformation_data_为全局参数
        }

        // 保留图像到本地
        imwrite("stitched_calib_feather_result.png", result(crop_roi));
        cout<<"拼接结果已保存为stitched_calib_result.png" << endl;
        // 关闭整个程序
        // exit(0);
        return true;
}

cv::Mat JHStitcher::processSubsequentGroupImpl
    (const std::vector<cv::Mat>& images,
     const std::vector<std::queue<VisiableTra>>& latest_visiable_tra_cache_)
    {
    // ========== 性能监控：总体计时开始 ==========
    auto total_start = chrono::high_resolution_clock::now();
    
    // ========== 步骤1：读取变换数据（含锁等待） ==========
    auto step1_start = chrono::high_resolution_clock::now();
    TransformationData data;
    {
        std::lock_guard<std::mutex> lock(transformation_mutex_);
        if (transformation_data_.cameras.empty()) {
            return cv::Mat();
        }
        data = transformation_data_;
    }
    auto step1_end = chrono::high_resolution_clock::now();
    auto step1_duration = chrono::duration_cast<chrono::milliseconds>(step1_end - step1_start).count();

    size_t num_images = images.size();
    if (num_images != data.cameras.size()) {
        return cv::Mat();
    }

    // ========== 步骤2：图像扭曲（CylindricalWarper） ==========
    auto step2_start = chrono::high_resolution_clock::now();
    std::vector<cv::Mat> images_warp(num_images);
    bool has_maps = data.mapIU_x.size() == num_images &&
                    data.mapIU_y.size() == num_images &&
                    data.mapPU_x.size() == num_images &&
                    data.mapPU_y.size() == num_images;
    for (size_t i = 0; i < num_images; ++i) {
        if (has_maps && !data.mapIU_x[i].empty() && !data.mapIU_y[i].empty() &&
            !data.mapPU_x[i].empty() && !data.mapPU_y[i].empty()) {
            cv::Mat undistorted;
            cv::remap(images[i], undistorted, data.mapIU_x[i], data.mapIU_y[i],
                      cv::INTER_LINEAR, cv::BORDER_CONSTANT);
            cv::remap(undistorted, images_warp[i], data.mapPU_x[i], data.mapPU_y[i],
                      cv::INTER_LINEAR, cv::BORDER_REFLECT);
        } else {
            cv::Mat K;
            data.cameras[i].K().convertTo(K, CV_32F);
            data.warper->warp(images[i], K, data.cameras[i].R, cv::INTER_LINEAR, cv::BORDER_REFLECT, images_warp[i]);
        }
    }
    auto step2_end = chrono::high_resolution_clock::now();
    auto step2_duration = chrono::duration_cast<chrono::milliseconds>(step2_end - step2_start).count();

    // ========== 步骤3：写入缓存（独占锁） ==========
    auto step3_start = chrono::high_resolution_clock::now();
    {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);
        for (size_t i = 0; i < images_warp.size(); ++i) {
            images_warp[i].copyTo(latest_warp_images_32F[i]);
        }
    }
    auto step3_end = chrono::high_resolution_clock::now();
    auto step3_duration = chrono::duration_cast<chrono::milliseconds>(step3_end - step3_start).count();

    // ========== 步骤4：ROI准备 ==========
    auto step4_start = chrono::high_resolution_clock::now();
    Rect dst_roi;
    if (!data.dst_roi.empty()) {
        dst_roi = data.dst_roi;
    } else {
        dst_roi = Rect(data.corners[0], data.sizes[0]);
        for (size_t i = 1; i < data.corners.size(); ++i) {
            dst_roi |= Rect(data.corners[i], data.sizes[i]);
        }
    }
    Mat accum = Mat::zeros(dst_roi.size(), CV_32FC3);
    Mat weight_sum = Mat::zeros(dst_roi.size(), CV_32FC1);
    auto step4_end = chrono::high_resolution_clock::now();
    auto step4_duration = chrono::duration_cast<chrono::milliseconds>(step4_end - step4_start).count();

    // ========== 步骤5：固定ROI掩码拷贝 ==========
    auto step5_start = chrono::high_resolution_clock::now();
    bool use_feather = data.feather_weights.size() == num_images;
    for (size_t i = 0; i < num_images; ++i) {
        if (images_warp[i].empty() || data.masks[i].empty()) {
            return cv::Mat();
        }
        if (data.masks[i].size() != images_warp[i].size()) {
            cv::resize(data.masks[i], data.masks[i], images_warp[i].size(), 0, 0, cv::INTER_NEAREST);
        }
        Rect roi_rect(data.corners[i] - dst_roi.tl(), data.sizes[i]);
        Rect pano_rect(0, 0, accum.cols, accum.rows);
        Rect clipped = roi_rect & pano_rect;
        if (clipped.empty()) {
            continue;
        }
        Rect src_rect(clipped.x - roi_rect.x, clipped.y - roi_rect.y, clipped.width, clipped.height);
        Mat weight_roi;
        if (use_feather && !data.feather_weights[i].empty() &&
            data.feather_weights[i].size() == images_warp[i].size()) {
            weight_roi = data.feather_weights[i](src_rect);
        } else {
            data.masks[i].convertTo(weight_roi, CV_32F, 1.0 / 255.0);
            weight_roi = weight_roi(src_rect);
        }
        Mat img_roi = images_warp[i](src_rect);
        Mat img_f;
        img_roi.convertTo(img_f, CV_32FC3);
        Mat weight_3;
        cv::cvtColor(weight_roi, weight_3, cv::COLOR_GRAY2BGR);
        Mat accum_roi = accum(clipped);
        Mat weight_sum_roi = weight_sum(clipped);
        accum_roi += img_f.mul(weight_3);
        weight_sum_roi += weight_roi;
    }
    auto step5_end = chrono::high_resolution_clock::now();
    auto step5_duration = chrono::duration_cast<chrono::milliseconds>(step5_end - step5_start).count();

    // ========== 步骤6：融合归一化 ==========
    auto step6_start = chrono::high_resolution_clock::now();
    Mat weight_safe;
    cv::max(weight_sum, 1e-6, weight_safe);
    Mat weight_safe_3;
    cv::cvtColor(weight_safe, weight_safe_3, cv::COLOR_GRAY2BGR);
    Mat pano_result_f;
    cv::divide(accum, weight_safe_3, pano_result_f);
    Mat pano_result;
    pano_result_f.convertTo(pano_result, images_warp[0].type());
    auto step6_end = chrono::high_resolution_clock::now();
    auto step6_duration = chrono::duration_cast<chrono::milliseconds>(step6_end - step6_start).count();

    // ========== 步骤7：图像缩放 ==========
    auto step7_start = chrono::high_resolution_clock::now();
    cv::Mat resized_pano_result = Mat();
    resize(pano_result, resized_pano_result, 
           cv::Size(static_cast<int>(pano_result.cols*scale_), static_cast<int>(pano_result.rows*scale_)),
           0, 0, cv::INTER_LINEAR);
    auto step7_end = chrono::high_resolution_clock::now();
    auto step7_duration = chrono::duration_cast<chrono::milliseconds>(step7_end - step7_start).count();

    // 判断裁剪区域有效性
    if (data.crop_roi.empty() || data.crop_roi.x < 0 || data.crop_roi.y < 0 || 
        data.crop_roi.x + data.crop_roi.width > resized_pano_result.cols || 
        data.crop_roi.y + data.crop_roi.height > resized_pano_result.rows) {
        cout<<"裁剪区域无效"<<endl;
        return Mat();
    }
    
    // ========== 步骤8：轨迹框投影 ==========
    auto step8_start = chrono::high_resolution_clock::now();
    CalibTrajInPano(resized_pano_result, data, latest_visiable_tra_cache_);
    auto step8_end = chrono::high_resolution_clock::now();
    auto step8_duration = chrono::duration_cast<chrono::milliseconds>(step8_end - step8_start).count();

    // ========== 步骤9：裁剪 ==========
    auto step9_start = chrono::high_resolution_clock::now();
    cv::Mat final_result;
    if(cropornot_){
        final_result = resized_pano_result(data.crop_roi);
    } else {
        final_result = resized_pano_result;
    }
    auto step9_end = chrono::high_resolution_clock::now();
    auto step9_duration = chrono::duration_cast<chrono::milliseconds>(step9_end - step9_start).count();

    // ========== 性能监控：总体计时结束 ==========
    auto total_end = chrono::high_resolution_clock::now();
    auto total_duration = chrono::duration_cast<chrono::milliseconds>(total_end - total_start).count();

    // ========== 性能报告（仅在总耗时>2ms时输出详细信息） ==========
    if (total_duration > 2) {
        std::cout << "⚠️ [性能分析] 总耗时: " << total_duration << "ms " << std::endl;
        std::cout << "  ├─ 步骤1-读取变换数据: " << step1_duration << "ms" << std::endl;
        std::cout << "  ├─ 步骤2-图像扭曲: " << step2_duration << "ms" << std::endl;
        std::cout << "  ├─ 步骤3-写入缓存(锁): " << step3_duration << "ms" << std::endl;
        std::cout << "  ├─ 步骤4-创建Blender: " << step4_duration << "ms" << std::endl;
        std::cout << "  ├─ 步骤5-类型转换&Feed: " << step5_duration << "ms" << std::endl;
        std::cout << "  ├─ 步骤6-Blend融合: " << step6_duration << "ms" << std::endl;
        std::cout << "  ├─ 步骤7-图像缩放: " << step7_duration << "ms" << std::endl;
        std::cout << "  ├─ 步骤8-轨迹框投影: " << step8_duration << "ms" << std::endl;
        std::cout << "  └─ 步骤9-裁剪: " << step9_duration << "ms" << std::endl;
    }

    return final_result;
}

void JHStitcher::detectStitchLine() 
{
    std::vector<cv::UMat> images;
    try {
        // 优化：缩短锁持有时间，仅在复制图像时加锁
        {
            std::shared_lock<std::shared_mutex> lock(rw_mutex_);
            //latest_warp_images_32F是一个全局变量，存储了最新的变形图像
            images.push_back(latest_warp_images_32F[0].clone());
            images.push_back(latest_warp_images_32F[1].clone());
            images.push_back(latest_warp_images_32F[2].clone());
        } // 锁在这里释放，后续处理不占用锁
    } catch (const std::exception& e) {
        // cout<<"读取错误: " << e.what()<<endl;
        return;
    }

    try {
        for (auto& img : images) {
            if (img.type() != CV_32FC3) {
                // cout<<"需要图像类型转换为CV_32FC3，开转！"<<endl;
                img.convertTo(img, CV_32FC3);
            }
        }
    } catch (const cv::Exception& e) {
        // cout<<"类型转换错误: %s"<<e.what()<<endl;
        return;
    }
    size_t num_images = images.size(); 
    Ptr<SeamFinder> seam_finder = makePtr<DpSeamFinder>(DpSeamFinder::COLOR_GRAD);
    // cv::Ptr<cv::detail::SeamFinder> seam_finder = cv::makePtr<cv::detail::GraphCutSeamFinder>(cv::detail::GraphCutSeamFinderBase::COST_COLOR);
    std::vector<cv::UMat> masks_warp_umat_8U(num_images);
    for (size_t i = 0; i < num_images; ++i) {
        masks_warp_umat_8U[i] = transformation_data_.raw_masks_8u[i].clone();
    }
    try {
        seam_finder->find(images, transformation_data_.corners, masks_warp_umat_8U);
    } catch (const cv::Exception& e) {
        // std::cerr << "缝合线查找失败: " << e.what() << std::endl;
        return;
    }
    for (size_t i = 0; i < num_images; ++i) {
        masks_warp_umat_8U[i].copyTo(transformation_data_.masks[i]);
    }
}

// 功能和 filtBoxes 类似，输入也类似
void JHStitcher::CalibTrajInPano(
    cv::Mat& pano,
    const TransformationData& data,
    const std::vector<std::queue<VisiableTra>>& latest_visiable_tra_cache_    
)
{
    // 0. 清空之前的轨迹框（避免累积旧数据）
    trajectory_boxes_.clear();
    
    // cout << "CalibTrajInPano 开始处理轨迹框" << endl;

    // 1. 计算全局ROI（与filtBoxes中相同的逻辑）
    Rect dst_roi = Rect(data.corners[0], data.sizes[0]);
    for (size_t i = 1; i < data.corners.size(); ++i) {
        dst_roi |= Rect(data.corners[i], data.sizes[i]);
    }

    // 2. 遍历每个相机的可见轨迹队列
    for (const auto& [cam_name, cam_idx] : cam_name_to_idx_) {
        // 检查索引是否有效
        if (cam_idx >= latest_visiable_tra_cache_.size()) {
            cout << "警告: 相机索引 " << cam_idx << " 超出范围" << endl;
            continue;
        }

        // 获取该相机的内参矩阵
        Mat K;
        data.cameras[cam_idx].K().convertTo(K, CV_32F);

        // 拷贝队列以便遍历（因为参数是const引用）
        std::queue<VisiableTra> single_cam_visiable_tra_queue = latest_visiable_tra_cache_[cam_idx];
        
        // cout << "相机 " << cam_name << " (索引 " << cam_idx << ") 的轨迹数量: " 
            //  << single_cam_visiable_tra_queue.size() << endl;

        // 3. 遍历该相机的所有轨迹消息
        while (!single_cam_visiable_tra_queue.empty()) {
            VisiableTra visiable_tra = single_cam_visiable_tra_queue.front();
            single_cam_visiable_tra_queue.pop();

            // 4. 轨迹框的两个对角点（左上角和右下角）
            std::vector<cv::Point2f> four_corners(2);
            four_corners[0] = cv::Point2f(visiable_tra.box_x1, visiable_tra.box_y1); // 左上角
            four_corners[1] = cv::Point2f(visiable_tra.box_x2, visiable_tra.box_y2); // 右下角

            // 5. 投影轨迹框的角点到拼接图坐标系
            std::vector<cv::Point2f> four_corners_warp(2);
            for (int j = 0; j < 2; ++j) {
                four_corners_warp[j] = data.warper->warpPoint(four_corners[j], K, data.cameras[cam_idx].R);
            }

            // 6. 坐标转换：基于投影后的角点，叠加偏移并缩放（与filtBoxes相同的逻辑）
            cv::Point2i top_left(
                cvRound((four_corners_warp[0].x - dst_roi.x) * scale_),
                cvRound((four_corners_warp[0].y - dst_roi.y) * scale_)
            );
            cv::Point2i bottom_right(
                cvRound((four_corners_warp[1].x - dst_roi.x) * scale_),
                cvRound((four_corners_warp[1].y - dst_roi.y) * scale_)
            );
            
            // cout << "top_left = " << top_left << ", bottom_right = " << bottom_right << endl;
            // cout << "four_corners_warp[0] = " << four_corners_warp[0] << ", four_corners_warp[1] = " << four_corners_warp[1] << endl;
            // 7. 确保检测框坐标非负且有效
            top_left.x = std::max(0, top_left.x);
            top_left.y = std::max(0, top_left.y);
            bottom_right.x = std::max(top_left.x + 1, bottom_right.x);
            bottom_right.y = std::max(top_left.y + 1, bottom_right.y);

            // 8. 构建TrajectoryBoxInfo对象并保存
            TrajectoryBoxInfo traj_box;
            traj_box.top_left = top_left; // 左上角坐标
            traj_box.bottom_right = bottom_right; // 右下角坐标
            traj_box.message_type = visiable_tra.ais_or_not ? "AIS" : "Visual";  // 消息类型，AIS或Visual
            traj_box.ship_type = visiable_tra.ship_type; // 船只类型
            traj_box.mmsi = visiable_tra.mmsi; // 船只编号
            traj_box.sog = visiable_tra.sog; // 船只速度
            traj_box.cog = visiable_tra.cog; // 船只航向
            traj_box.lat = visiable_tra.latitude; // 船只纬度
            traj_box.lon = visiable_tra.longitude; // 船只经度

            trajectory_boxes_.push_back(traj_box);

            // 9. 如果启用绘制，在拼接图上画框（可选）
            if (drawboxornot_) {
                // 根据是否包含AIS信息选择颜色
                cv::Scalar color = visiable_tra.ais_or_not ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 0, 0);  // AIS绿色，纯视觉蓝色
                cv::rectangle(pano, top_left, bottom_right, color, 2);
                
                // 文本绘制配置
                double font_scale = 0.5;
                int font_thickness = 2;
                int font_face = cv::FONT_HERSHEY_SIMPLEX;
                int baseline = 0;
                
                // 计算文本高度（用于行间距）
                cv::Size text_size = cv::getTextSize("Test", font_face, font_scale, font_thickness, &baseline);
                int line_height = text_size.height + baseline + 3;  // 文本高度 + 基线 + 间距
                
                // 从检测框左下角开始，依次往下绘制信息
                int text_x = top_left.x;
                int text_y = bottom_right.y + line_height;  // 从左下角开始，往下偏移一行
                
                // 绘制船只类型信息（如果有效）
                if (!visiable_tra.ship_type.empty()) {
                    std::string label = "Ship Type:" + visiable_tra.ship_type;
                    cv::putText(pano, label, cv::Point(text_x, text_y),
                               font_face, font_scale, color, font_thickness);
                    text_y += line_height;  // 下一行
                }
                
                // 绘制MMSI信息（如果有效）
                if (visiable_tra.mmsi > 0) {
                    std::string label = "MMSI:" + std::to_string(visiable_tra.mmsi);
                    cv::putText(pano, label, cv::Point(text_x, text_y),
                               font_face, font_scale, color, font_thickness);
                    text_y += line_height;  // 下一行
                }
                
                // 绘制SOG信息（如果有效）
                if (visiable_tra.sog > 0) {
                    // 保留2位小数
                    char buffer[32];
                    snprintf(buffer, sizeof(buffer), "SOG:%.2f", visiable_tra.sog);
                    cv::putText(pano, buffer, cv::Point(text_x, text_y),
                               font_face, font_scale, color, font_thickness);
                    text_y += line_height;  // 下一行
                }
                
                // 绘制COG信息（如果有效）
                if (visiable_tra.cog > 0) {
                    // 保留2位小数
                    char buffer[32];
                    snprintf(buffer, sizeof(buffer), "COG:%.2f", visiable_tra.cog);
                    cv::putText(pano, buffer, cv::Point(text_x, text_y),
                               font_face, font_scale, color, font_thickness);
                    text_y += line_height;  // 下一行
                }
                
                // 绘制LAT信息（如果有效）
                if (visiable_tra.latitude != 0) {  // 纬度可以为负，所以用 != 0
                    // 保留6位小数（GPS精度）
                    char buffer[32];
                    snprintf(buffer, sizeof(buffer), "LAT:%.6f", visiable_tra.latitude);
                    cv::putText(pano, buffer, cv::Point(text_x, text_y),
                               font_face, font_scale, color, font_thickness);
                    text_y += line_height;  // 下一行
                }
                
                // 绘制LON信息（如果有效）
                if (visiable_tra.longitude != 0) {  // 经度可以为负，所以用 != 0
                    // 保留6位小数（GPS精度）
                    char buffer[32];
                    snprintf(buffer, sizeof(buffer), "LON:%.6f", visiable_tra.longitude);
                    cv::putText(pano, buffer, cv::Point(text_x, text_y),
                               font_face, font_scale, color, font_thickness);
                    // text_y += line_height;  // 如果后续还有信息，继续递增
                }
            }
        }
    }

    // cout << "CalibTrajInPano 完成，共处理 " << trajectory_boxes_.size() << " 个轨迹框" << endl;
}
// 其他辅助函数的实现...

// 保存相机参数到YAML文件
void JHStitcher::saveCameraParamters(const std::string& filename, const std::vector<CameraParams>& cameras) {
    FileStorage fs(filename, FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "无法打开文件保存相机参数: " << filename << std::endl;
        return;
    }

    fs << "num_cameras" << static_cast<int>(cameras.size());
    fs << "cameras" << "[";
    for (const auto& cam : cameras) {
        fs << "{";
        fs << "FOV_hor" << FOV_hor_;
        fs << "FOV_ver" << FOV_ver_;
        fs << "focal" << cam.focal; 
        fs << "aspect" << cam.aspect;// 宽高比
        fs << "ppx" << cam.ppx;
        fs << "ppy" << cam.ppy;
        fs << "R" << cam.R;
        fs << "t" << cam.t;
        // 这里不保存K矩阵，因为它是由focal, aspect, ppx, ppy计算得出
        // 但为了方便起见，我们仍然保存K矩阵，便于直接查看
        fs << "K_matrix" << cam.K();  // 这里调用K()获取计算后的矩阵
        fs << "}";
    }
    
    fs << "]";
    fs.release();
    std::cout << "相机参数已保存到: " << filename << std::endl;
}

// 从YAML文件加载相机参数
std::vector<cv::detail::CameraParams> JHStitcher::readCameraParamters(const std::string& filename) {
    std::vector<cv::detail::CameraParams> cameras;
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "错误：无法打开文件 " << filename << " 进行读取！" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    int num_cameras;
    fs["num_cameras"] >> num_cameras;

    // 获取"cameras"序列节点（关键修正）
    cv::FileNode cameras_node = fs["cameras"];
    if (cameras_node.type() != cv::FileNode::SEQ) {  // 检查是否为序列类型
        std::cerr << "错误：YAML文件中'cameras'不是序列类型！" << std::endl;
        return cameras;
    }


    // 遍历序列中的每个相机参数（关键修正）
    cv::FileNodeIterator it = cameras_node.begin();
    cv::FileNodeIterator it_end = cameras_node.end();
    for (int i = 0; it != it_end && i < num_cameras; ++it, ++i) {
        cv::detail::CameraParams cam;
        cv::FileNode cam_node = *it;  // 当前相机的节点

        // 读取参数（直接从序列元素中读取，无需"camera_i"前缀）
        cam_node["focal"] >> cam.focal;
        cam_node["aspect"] >> cam.aspect;
        cam_node["ppx"] >> cam.ppx;
        cam_node["ppy"] >> cam.ppy;
        cam_node["R"] >> cam.R;
        cam_node["t"] >> cam.t;

        cameras.push_back(cam);
    }
    // 检查读取数量是否匹配
    if (cameras.size() != num_cameras) {
        std::cerr << "警告：读取的相机参数数量（" << cameras.size() 
                  << "）与记录的数量（" << num_cameras << "）不匹配！" << std::endl;
    }
    cout<<"读取相机参数成功，数量为: "<< cameras.size() << endl;

    fs.release();
    return cameras;
}


// 获得全景边框
std::vector<Point> JHStitcher::prepare_pxpy(const vector<Point> &corners, const vector<Size> &sizes)
{
    // 检查输入参数合法性
    if (corners.empty() || sizes.empty() || corners.size() != sizes.size()) {
        throw std::invalid_argument("Invalid input: corners and sizes must be non-empty and of the same size");
    }

    Rect dst_roi = Rect(corners[0], sizes[0]);
    std::vector<Point> dxdy;

    for (size_t i = 1; i < corners.size(); ++i)
        dst_roi |= Rect(corners[i], sizes[i]);
    // cout << "dst_roi in prepare_pxpy = " << dst_roi << endl;
    for(const auto corner : corners)
    {
        int dx = corner.x - dst_roi.x;  // X方向偏移量
        int dy = corner.y - dst_roi.y;  // Y方向偏移量
        // cout << "dx = " << dx << ", dy = " << dy << endl;
        dxdy.emplace_back(dx, dy);      // 添加到结果列表
    }
    // cout << "data.dxdy[0].x = " << to_string(dxdy[0].x) << "data.dxdy[0].y = " << to_string(dxdy[0].y) << endl;
    // cout << "data.dxdy[1].x = " << to_string(dxdy[1].x) << "data.dxdy[1].y = " << to_string(dxdy[1].y) << endl;
    // cout << "data.dxdy[2].x = " << to_string(dxdy[2].x) << "data.dxdy[2].y = " << to_string(dxdy[2].y) << endl;


    return dxdy;
}

// 计算指定行的白色像素比例
float JHStitcher::getWhiteRatio(const cv::Mat& mask, int row) {
    if (mask.empty() || row < 0 || row >= mask.rows) {
        return 0.0f;
    }
    
    // 确保是单通道图像
    CV_Assert(mask.channels() == 1);
    
    int whiteCount = 0;
    const uchar* rowPtr = mask.ptr<uchar>(row);
    
    // 统计白色像素(255)数量
    for (int col = 0; col < mask.cols; ++col) {
        if (rowPtr[col] == 255) {
            whiteCount++;
        }
    }
    
    // 返回白色像素比例
    return static_cast<float>(whiteCount) / mask.cols;
}

// 二分法查找第一个白色比例超过阈值的行
int JHStitcher::findFirstValidRow(const cv::Mat& mask, float threshold) {
    int left = 0;
    int right = mask.rows - 1;
    int result = -1;
    
    while (left <= right) {
        int mid = left + (right - left) / 2; // 避免溢出
        float ratio = getWhiteRatio(mask, mid);
        
        if (ratio > threshold) {
            result = mid;   // 记录可能的结果
            right = mid - 1; // 继续向左查找更早的符合条件的行
        } else {
            left = mid + 1; // 向右查找
        }
    }
    return result;
}

// 二分法查找最后一个白色比例超过阈值的行
int JHStitcher::findLastValidRow(const cv::Mat& mask, float threshold) {
    int left = 0;
    int right = mask.rows - 1;
    int result = -1;
    while (left <= right) {
        int mid = left + (right - left) / 2;
        float ratio = getWhiteRatio(mask, mid);
        
        if (ratio > threshold) {
            result = mid;   // 记录可能的结果
            left = mid + 1; // 继续向右查找更晚的符合条件的行
        } else {
            right = mid - 1; // 向左查找
        }
    }
    return result;
}

// 获取所有白色比例超过阈值的行的y坐标
Rect JHStitcher::getValidRows(const cv::Mat& result_mask, float threshold) {
    if (result_mask.empty()) {
        return Rect(); // 返回空矩形表示无效输入
    }
    // 查找有效行的范围
    int first = findFirstValidRow(result_mask, threshold);
    int last = findLastValidRow(result_mask, threshold);
    // cout << "First valid row: " << first << ", Last valid row: " << last << endl;
    // 如果没有找到有效行
    if (first == -1 || last == -1) {
        return Rect(); // 返回空矩形表示无效输入
    }
    return Rect(0, first, result_mask.cols, last - first + 1);
}

// 在processFirstGroupImpl函数中添加此检测筛选逻辑
bool JHStitcher::checkAndFilterMatches(std::vector<cv::detail::MatchesInfo>& pairwise_matches) {
    // 需要保留的图像对组合（允许双向匹配）
    std::set<std::pair<int, int>> valid_pairs = {
        {0, 1}, {1, 0},   // 0和1之间的双向匹配
        {1, 2}, {2, 1}    // 1和2之间的双向匹配
    };
    
    
    for (auto& match_info : pairwise_matches) {
        int src = match_info.src_img_idx;
        int dst = match_info.dst_img_idx;
        std::pair<int, int> current_pair = {src, dst};
        
        // 检查当前匹配对是否在有效列表中
        if (valid_pairs.find(current_pair) == valid_pairs.end()) {//if条件是指
            // 非有效匹配对：清除匹配信息
            match_info.matches.clear();
            match_info.inliers_mask.clear();
            match_info.num_inliers = 0;
            match_info.H = cv::Mat();
            match_info.confidence = 0.0;
        } else {
            // 有效匹配对：检查置信度、内点数是否达标
            if (match_info.confidence < min_confidence_) {
                // 置信度不足，返回失败
                std::cerr << "Error: 有效匹配对的最低置信度仅为:" + to_string(match_info.confidence) + "低于阈值"<< to_string(min_confidence_) <<",换一帧检测" << std::endl;
                return false;
            }
            
            if (match_info.num_inliers < min_inliers_) {  // 内点数量阈值
            std::cerr << "Error: 图像对 (" << src << "-" << dst 
                    << ") 内点数量不足（" << match_info.num_inliers << "），至少需要" + to_string(min_inliers_) + "个" << std::endl;
            return false;
        }
        }
    }
    
    // 所有有效匹配对均满足置信度要求
    return true;
}

// 检查相机参数的合法性
bool JHStitcher::checkCameraLegitimacy(const std::vector<CameraParams>& cameras) {
    bool valid = true;
    for (const auto& cam : cameras) {
        if (std::isnan(cam.focal) || cam.focal <= 0 || 
            std::isnan(cam.ppx) || std::isnan(cam.ppy) ||
            cam.R.empty() || cam.R.rows != 3 || cam.R.cols != 3) 
            {
                // cout<<"初始相机参数存在无效值"<<endl;
                valid = false;
            }
    }
    return valid;  // 返回是否有无效相机参数
}

// 检查各个相机焦距的方差是否在允许范围内
bool JHStitcher::checkBALegitimacy(std::vector<cv::detail::CameraParams>& cameras)
{
    // 检查相机焦距的方差是否在允许范围内
    // 这里可以根据实际情况实现具体的检查逻辑
    // 返回true表示焦距方差在允许范围内，false表示不在范围内
    if (cameras.size() >= 2) {  // 至少需要2个相机才能计算方差
        // 1. 计算焦距平均值
        double focal_sum = 0.0;
        for (const auto& cam : cameras) {
            if(std::isnan(cam.focal) || cam.focal< 0) {
                // std::cerr << "Error: 相机的焦距为：" << cam.focal << "，包含nan，or less than 0 计算异常！" << std::endl;
                return false;  // 返回空数据表示失败
            }
            focal_sum += cam.focal;
        }
        double focal_mean = focal_sum / cameras.size();

        // 2. 计算方差
        double variance = 0.0;
        for (const auto& cam : cameras) {
            variance += pow(cam.focal - focal_mean, 2);
        }
        variance /= cameras.size();  // 总体方差（样本量较小时可用此方法）

        // 3. 设置方差阈值（根据实际场景调整）
        if (variance > max_focal_variance_) {
            std::cerr << "Error: 光束平差后焦距方差过大（方差=" << variance 
                    << "，阈值=" << max_focal_variance_ << "）" << std::endl;
            // 输出各相机焦距便于调试
            for (size_t i = 0; i < cameras.size(); ++i) {
                std::cerr << "相机 " << i << " 焦距: " << cameras[i].focal << std::endl;
            }
            return false;  // 返回空数据表示失败
        }
    } else if (!cameras.empty()) {
        // std::cerr << "Warning: 仅单个相机，无法计算焦距方差" << std::endl;
        return false;
    } else {
        // std::cerr << "Error: 未获取到相机参数" << std::endl;
        return false;
    }

    // 同时也要检测相机变化矩阵的合法性
    for (const auto& cam : cameras) {
        if (cam.R.empty() || cam.t.empty()) {
            // std::cerr << "Error: 相机变换矩阵R或t为空" << std::endl;
            return false;  // 返回空数据表示失败
        }
    }

    // 在相机参数估计后（如光束平差后）添加检测
    for (size_t i = 0; i < cameras.size(); ++i) {
        // 检查旋转矩阵R是否包含nan
        bool has_nan = false;
        if (!cameras[i].R.empty() && cameras[i].R.rows == 3 && cameras[i].R.cols == 3) {
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    double val = cameras[i].R.at<double>(row, col);
                    if (std::isnan(val)) {  // 检测nan
                        has_nan = true;
                        break;
                    }
                }
                if (has_nan) break;
            }
        } else {
            // R矩阵尺寸异常（非3x3）
            // std::cerr << "相机 " << i << " 旋转矩阵R尺寸无效（非3x3）" << std::endl;
            return false;
        }

        if (has_nan) {
            // std::cerr << "相机 " << i << " 旋转矩阵R包含nan，计算异常！" << std::endl;
            // 输出关联的其他参数，辅助调试
            // std::cerr << "  焦距: " << cameras[i].focal << ", 主点: (" << cameras[i].ppx << ", " << cameras[i].ppy << ")" << std::endl;
            return false;  // 终止流程，避免后续错误
        }
    }
    return true;  // 焦距方差在允许范围内

}

// 检查各图片的左上角点位置是否合法
bool JHStitcher::checkTopLeftPointsLegitimacy(const vector<vector<Point2f>>& four_corners_warp) {
    // 检测左角点（左上角点）是否符合分布要求
    // 2. 检查x坐标是否递增分布
    for (size_t i = 1; i < four_corners_warp.size(); ++i) {
        if (four_corners_warp[i][0].x <= four_corners_warp[i-1][0].x) {
            std::cerr << "错误：图像 " << i-1 << " 和 " << i << " 的x坐标未递增（" 
                    << four_corners_warp[i][0].x << " <= " << four_corners_warp[i-1][0].x << "）" << std::endl;
            return false;
        }
    }

    // 3. 检查y坐标是否在允许范围内浮动
    double min_y = four_corners_warp[0][0].y;
    double max_y = four_corners_warp[0][0].y;
    for (const auto& pt : four_corners_warp) {
        if (pt[0].y < min_y) min_y = pt[0].y;
        if (pt[0].y > max_y) max_y = pt[0].y;
    }
    if (max_y - min_y > y_tolerance_) {
        std::cerr << "错误：y坐标浮动超出阈值（差值：" << max_y - min_y 
                << "，阈值：" << y_tolerance_ << "）" << std::endl;
        return false;
    }
    return true;

}
