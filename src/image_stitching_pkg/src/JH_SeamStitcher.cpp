// 只使用拼缝线的版本

#include "JH_SeamStitcher.hpp"
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

JHStitcher::JHStitcher(
    std::unordered_map<std::string, int>& cam_name_to_idx,
    int refresh_time,
    size_t min_key_points,
    double min_confidence,
    int min_inliers,
    double max_focal_variance,
    double y_tolerance,
    float roi_threshold,
    double scale,
    bool crop_or_not,
    int crop_top,
    int crop_bottom,
    int crop_left,
    int crop_right,
    bool draw_box_or_not,
    bool save_camera_params,
    string remap_dir,
    string cache_file,
    string calib_dir,
    bool use_saved_camera_params,
    double FOV_hor,
    double FOV_ver
):  
    cam_name_to_idx_(cam_name_to_idx),
    refresh_time_(refresh_time),
    min_key_points_(min_key_points),
    min_confidence_(min_confidence),
    min_inliers_(min_inliers),
    max_focal_variance_(max_focal_variance),
    y_tolerance_(y_tolerance),
    roi_threshold_(roi_threshold),
    scale_(scale),
    crop_or_not_(crop_or_not),
    crop_top_(crop_top),
    crop_bottom_(crop_bottom),
    crop_left_(crop_left),
    crop_right_(crop_right),
    draw_box_or_not_(draw_box_or_not),
    save_camera_params_(save_camera_params),
    remap_dir_(remap_dir),
    cache_file_(cache_file),
    calib_dir_(calib_dir),
    use_saved_camera_params_(use_saved_camera_params),
    FOV_hor_(FOV_hor),
    FOV_ver_(FOV_ver)

    {
    latest_warp_images_32F.resize(3);
    if (save_camera_params_file_.empty() || remap_dir_.empty() || cache_file_.empty() || calib_dir_.empty()) {
        std::string share_dir = ament_index_cpp::get_package_share_directory("image_stitching_pkg");
        if (calib_dir_.empty()) {
            calib_dir_ = share_dir + "/config";
        }
        if (save_camera_params_file_.empty()) {
            save_camera_params_file_ = calib_dir_ + "/CameraParams.yaml";
        }
        if (remap_dir_.empty()) {
            remap_dir_ = share_dir + "/config/remap";
        }
        if (cache_file_.empty()) {
            cache_file_ = remap_dir_ + "/stitch_cache.yaml";
        }
    }
    }
bool JHStitcher::processFirstGroupImpl(std::vector<cv::Mat>& images) {
    // =================== debug 代码：保存输入图像 ===================
    // bool JHStitcher::processFirstGroupImpl(const std::vector<cv::Mat>& images) {
    // 保存输入图片，以便后续调试和检查
    // for (size_t i = 0; i < images.size(); ++i) {
    //     cv::imwrite("input_image_" + std::to_string(i) + ".png", images[i]);
    //     cout << "已保存输入图像: input_image_" << i << ".png" << endl;
    // }
    //  设置images为现有图片 
    // for(int i = 0; i < images.size(); ++i) {
    //     images[i] = cv::imread("/home/tl/RV/input_image_" + std::to_string(i) + ".png");
    // }
    // =======================================================
    int num_images = images.size();
    cout<<"处理第一组图片，相机数量:"<< num_images << endl;

    TransformationData data;
    if (use_saved_camera_params_) { // 使用已保存的相机参数
        cout<<"使用已保存的相机参数" << endl;
        TransformationData cached;
        if (loadStitchCache(cache_file_, cached) && loadRemapMaps(remap_dir_, num_images, cached)) {
            bool size_ok = (static_cast<int>(cached.cameras.size()) == num_images);
            for (int i = 0; i < num_images && size_ok; ++i) {
                if (cached.mapIU_x[i].size() != images[i].size() ||
                    cached.mapIU_y[i].size() != images[i].size()) {
                    size_ok = false;
                }
            }
            if (size_ok) {
                if (cached.calib_K.empty() || cached.calib_D.empty()) {
                    cached.calib_K.resize(num_images);
                    cached.calib_D.resize(num_images);
                    cached.calib_Knew.resize(num_images);
                    for (int i = 0; i < num_images; ++i) {
                        std::string calib_path = calib_dir_ + "/calibration_result_" + std::to_string(i) + ".yaml";
                        cv::Mat K, D;
                        if (readFisheyeCalibration(calib_path, K, D)) {
                            cached.calib_K[i] = K;
                            cached.calib_D[i] = D;
                            cached.calib_Knew[i] = K;
                        }
                    }
                }
                std::lock_guard<std::mutex> lock(transformation_mutex_);
                transformation_data_ = cached;
                return true;
            }
        }
    }
    else{ // 不使用已保存的相机参数，按照当前图像重新计算相机参数
        cout<<"不使用已保存的相机参数" << endl;
        // 先用相机标定参数去畸变（fisheye）
        std::vector<cv::Mat> undistorted_images(num_images);
        data.mapIU_x.resize(num_images);
        data.mapIU_y.resize(num_images);
        data.calib_K.resize(num_images);
        data.calib_D.resize(num_images);
        data.calib_Knew.resize(num_images);
        for (int i = 0; i < num_images; ++i) {
            std::string calib_path = calib_dir_ + "/calibration_result_" + std::to_string(i) + ".yaml";
            cv::Mat K, D;
            if (!readFisheyeCalibration(calib_path, K, D)) {
                cout << "无法读取相机标定参数: " << calib_path << endl;
                return false;
            }
            cv::Mat K_new;
            double balance = 1.0; // 0:更裁剪，1:尽量保留视场
            double fov_scale = 1.0; // 视场缩放因子
            // 计算新的相机矩阵
            cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
                K, D, images[i].size(), cv::Mat::eye(3, 3, CV_64F), K_new, balance, images[i].size(), fov_scale
            );
            // 计算去畸变映射
            cv::fisheye::initUndistortRectifyMap(
                K, D, cv::Mat::eye(3, 3, CV_64F), K_new, images[i].size(), CV_32FC1,
                data.mapIU_x[i], data.mapIU_y[i]);
            // 应用去畸变映射
            cv::remap(images[i], undistorted_images[i], data.mapIU_x[i], data.mapIU_y[i],
                    cv::INTER_LINEAR, cv::BORDER_CONSTANT);
            data.calib_K[i] = K;
            data.calib_D[i] = D;
            data.calib_Knew[i] = K_new;
        }

        // 相机参数估计
        std::vector<CameraParams> cameras;
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
            if (features[i].keypoints.size() < min_key_points_) {
                cout<<"图像"<<i<<"特征点不足,只有："<<features[i].keypoints.size()<<endl;
                return false;
            }
                cout<<"图像"<<i<<"特征点充足,有："<<features[i].keypoints.size()<<endl;
        }

        // 特征匹配
        // cout<< "开始特征匹配" << endl;
        std::vector<MatchesInfo> pairwise_matches;
        Ptr<FeaturesMatcher> matcher = makePtr<BestOf2NearestRangeMatcher>(1, true, 0.3f, 6, 6);
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
                // 输出初始相机参数
            for (size_t i = 0; i < cameras.size(); ++i)
            {
                // cout<<"在光束平差前，相机参数：" << endl;
                // std::cout << "camera #" << i + 1 << ":\n内参数矩阵K:\n" << cameras[i].K() << "\n旋转矩阵R:\n" << cameras[i].R << "\n焦距focal: " << cameras[i].focal << std::endl;
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
        std::vector<Mat> rotations;
        for (size_t i = 0; i < cameras.size(); ++i) {
            rotations.push_back(cameras[i].R);
        }
        waveCorrect(rotations, WAVE_CORRECT_HORIZ);
        for (size_t i = 0; i < cameras.size(); ++i) {
            cameras[i].R = rotations[i];
        }

        // 创建掩码
        std::vector<Mat> masks(num_images);
        for (int i = 0; i < num_images; ++i) {
            masks[i].create(undistorted_images[i].size(), CV_8U);
            masks[i].setTo(Scalar::all(255));
        }

        // 图像扭曲
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
        
        if(!checkTopLeftPointsLegitimacy(four_corners_warp)) {
            // 若检查失败，输出错误信息并退出函数
            std::cerr << "Error: 左上角点位置不合法，可能是图像尺寸过小或变换参数异常。" << std::endl;
            return false;  // 返回空数据表示失败
        }

        // 曝光补偿
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
            cv::UMat img_umat = images_warp[i].getUMat(ACCESS_RW);
            cv::UMat mask_umat = masks_warp[i].getUMat(ACCESS_RW);
            compensator->apply(i, corners[i], img_umat, mask_umat);
            img_umat.copyTo(images_warp[i]);
            mask_umat.copyTo(masks_warp[i]);
        }

        //缝合线查找: 图割法、动态规划缝合线、Voronoi图缝合线查找 GraphCutSeamFinderBase DpSeamFinder VoronoiSeamFinder
        Ptr<SeamFinder> seam_finder = makePtr<DpSeamFinder>(DpSeamFinder::COLOR_GRAD);
        // Ptr<SeamFinder> seam_finder = makePtr<GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
        
        // 确保图像和掩码的类型正确
        data.raw_masks_8u.resize(num_images);
        for (int i = 0; i < num_images; ++i) {
            images_warp[i].convertTo(images_warp_umat[i], CV_32F); // 确保图像数据类型为CV_32F，不然会导致图割法错误。
            masks_warp[i].convertTo(masks_warp_umat[i], CV_8U);   // 确保掩码数据类型为CV_8U，去掉 'U8'，掩码会更新错误。
            data.raw_masks_8u[i] = masks_warp_umat[i].clone(); 
    }

        try {
            seam_finder->find(images_warp_umat, corners, masks_warp_umat);
        } catch (const Exception& e) {
            cout<< "缝合线查找失败"<<endl;
            return false;
        }
        for (int i = 0; i < num_images; ++i) {
            masks_warp_umat[i].copyTo(masks_warp[i]);
        }

        // 图像融合
        // cout<<"开始图像融合" << endl;
        Ptr<Blender> blender = Blender::createDefault(Blender::NO, false);
        blender->prepare(corners, sizes);
        for (int i = 0; i < num_images; ++i) {
            Mat img_warp_16s;
            if (images_warp[i].empty() || masks_warp[i].empty()) {
                cout << "融合前图像或掩码为空: idx=" << i << endl;
                return false;
            }
            if (masks_warp[i].size() != images_warp[i].size()) {
                cv::resize(masks_warp[i], masks_warp[i], images_warp[i].size(), 0, 0, cv::INTER_NEAREST);
            }
            if (masks_warp[i].type() != CV_8U) {
                masks_warp[i].convertTo(masks_warp[i], CV_8U);
            }
            images_warp[i].convertTo(img_warp_16s, CV_16S);
            blender->feed(img_warp_16s, masks_warp[i], corners[i]);
        }

        Mat result, result_mask;
        blender->blend(result, result_mask);

        // 裁剪图像
        // cout<<"开始裁剪图像" << endl;
        Mat crop_mask;
        cv::threshold(result_mask, crop_mask, 0, 255, cv::THRESH_BINARY);
        Rect crop_roi;
        if (crop_or_not_ && (crop_top_ > 0 || crop_bottom_ > 0 || crop_left_ > 0 || crop_right_ > 0)) {
            int left = std::max(0, crop_left_);
            int top = std::max(0, crop_top_);
            int right = std::max(0, crop_right_);
            int bottom = std::max(0, crop_bottom_);
            int width = result.cols - left - right;
            int height = result.rows - top - bottom;
            if (width <= 0 || height <= 0) {
                cout << "裁剪参数无效: left=" << left << ", right=" << right
                     << ", top=" << top << ", bottm=" << bottom << endl;
                return false;
            }
            crop_roi = Rect(left, top, width, height);
        } else {
            // crop_roi保持为原始图片大小
            crop_roi = Rect(0, 0, result.cols, result.rows);
            if (crop_roi.empty()) {
                cout<<"图像不裁剪"<<endl;
                return false;
            }
        }

        data.cameras = cameras;
        data.warper = warper;
        data.compensator = compensator;
        data.seam_finder = seam_finder;
        data.corners = corners;
        data.sizes = sizes;
        data.masks = masks_warp;
        data.dxdy = prepare_pxpy(data.corners,data.sizes);
        data.crop_roi = Rect(static_cast<int>(crop_roi.x*scale_),static_cast<int>(crop_roi.y*scale_),static_cast<int>(crop_roi.width*scale_),static_cast<int>(crop_roi.height*scale_));

        {
            std::lock_guard<std::mutex> lock(transformation_mutex_);
            transformation_data_ = data; //transformation_data_为全局参数
        }

        // 保留图像到本地
        imwrite("stitched_calib_result.png", result(crop_roi));
        cout<<"拼接结果已保存为stitched_calib_result.png" << endl;
        // 关闭整个程序
        if (!saveRemapMaps(remap_dir_, data, num_images)) {
            cout << "保存remap映射表失败" << endl;
            return false;
        }
        if (!saveStitchCache(cache_file_, data)) {
            cout << "保存拼接缓存失败" << endl;
            return false;
        }

        // exit(0);
        return true;
        }
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
    Rect dst_roi = Rect(data.corners[0], data.sizes[0]);
    for (size_t i = 1; i < data.corners.size(); ++i) {
        dst_roi |= Rect(data.corners[i], data.sizes[i]);
    }
    Mat pano_result = Mat::zeros(dst_roi.size(), images_warp[0].type());
    auto step4_end = chrono::high_resolution_clock::now();
    auto step4_duration = chrono::duration_cast<chrono::milliseconds>(step4_end - step4_start).count();

    // ========== 步骤5：固定ROI掩码拷贝 ==========
    auto step5_start = chrono::high_resolution_clock::now();
    for (size_t i = 0; i < num_images; ++i) {
        if (images_warp[i].empty() || data.masks[i].empty()) {
            return cv::Mat();
        }
        if (data.masks[i].size() != images_warp[i].size()) {
            cv::resize(data.masks[i], data.masks[i], images_warp[i].size(), 0, 0, cv::INTER_NEAREST);
        }
        Rect roi_rect(data.corners[i] - dst_roi.tl(), data.sizes[i]);
        Rect pano_rect(0, 0, pano_result.cols, pano_result.rows);
        Rect clipped = roi_rect & pano_rect;
        if (clipped.empty()) {
            continue;
        }
        Rect src_rect(clipped.x - roi_rect.x, clipped.y - roi_rect.y, clipped.width, clipped.height);
        Mat pano_roi = pano_result(clipped);
        Mat warp_roi = images_warp[i](src_rect);
        Mat mask_roi = data.masks[i](src_rect);
        warp_roi.copyTo(pano_roi, mask_roi);
    }
    auto step5_end = chrono::high_resolution_clock::now();
    auto step5_duration = chrono::duration_cast<chrono::milliseconds>(step5_end - step5_start).count();

    // ========== 步骤6：Blend融合 ==========
    auto step6_start = chrono::high_resolution_clock::now();
    Mat result_mask;
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
    
    // ========== 步骤8：裁剪 ==========
    auto step8_start = chrono::high_resolution_clock::now();
    cv::Mat final_result;
    if(crop_or_not_){
        final_result = resized_pano_result(data.crop_roi);
    } else {
        final_result = resized_pano_result;
    }
    auto step8_end = chrono::high_resolution_clock::now();
    auto step8_duration = chrono::duration_cast<chrono::milliseconds>(step8_end - step8_start).count();

    // ========== 步骤9：轨迹框投影 ==========
    auto step9_start = chrono::high_resolution_clock::now();
    CalibTrajInPano(final_result, data, latest_visiable_tra_cache_);
    auto step9_end = chrono::high_resolution_clock::now();
    auto step9_duration = chrono::duration_cast<chrono::milliseconds>(step9_end - step9_start).count();

    // ========== 性能监控：总体计时结束 ==========
    auto total_end = chrono::high_resolution_clock::now();
    auto total_duration = chrono::duration_cast<chrono::milliseconds>(total_end - total_start).count();

    // ========== 性能报告（仅在总耗时>2ms时输出详细信息） ==========
    // if (total_duration > 2) {
    //     std::cout << "⚠️ [性能分析] 总耗时: " << total_duration << "ms " << std::endl;
    //     std::cout << "  ├─ 步骤1-读取变换数据: " << step1_duration << "ms" << std::endl;
    //     std::cout << "  ├─ 步骤2-图像扭曲: " << step2_duration << "ms" << std::endl;
    //     std::cout << "  ├─ 步骤3-写入缓存(锁): " << step3_duration << "ms" << std::endl;
    //     std::cout << "  ├─ 步骤4-创建Blender: " << step4_duration << "ms" << std::endl;
    //     std::cout << "  ├─ 步骤5-类型转换&Feed: " << step5_duration << "ms" << std::endl;
    //     std::cout << "  ├─ 步骤6-Blend融合: " << step6_duration << "ms" << std::endl;
    //     std::cout << "  ├─ 步骤7-图像缩放: " << step7_duration << "ms" << std::endl;
    //     std::cout << "  ├─ 步骤8-轨迹框投影: " << step8_duration << "ms" << std::endl;
    //     std::cout << "  └─ 步骤9-裁剪: " << step9_duration << "ms" << std::endl;
    // }

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

        // 获取该相机的拼接内参矩阵
        Mat K_stitch;
        data.cameras[cam_idx].K().convertTo(K_stitch, CV_32F);

        // 拷贝队列以便遍历（因为参数是const引用）
        std::queue<VisiableTra> single_cam_visiable_tra_queue = latest_visiable_tra_cache_[cam_idx];

        // Debug时，手动添加原始轨迹框，以便观察轨迹框在经过畸变矫正-投影-缩放-裁剪后的效果
        // if (data.mapIU_x.size() > static_cast<size_t>(cam_idx) &&
        //     !data.mapIU_x[cam_idx].empty()) {
        //     VisiableTra debug_box{};
        //     debug_box.camera_name = "debug";
        //     debug_box.box_x1 = 1000.0f;
        //     debug_box.box_y1 = 300.0f;
        //     debug_box.box_x2 = 1100.0f;
        //     debug_box.box_y2 = 350.0f;
        //     single_cam_visiable_tra_queue.push(debug_box);
        // }


        
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
                cv::Point2f undistorted_pt = four_corners[j];
                if (data.calib_K.size() > static_cast<size_t>(cam_idx) &&
                    data.calib_D.size() > static_cast<size_t>(cam_idx) &&
                    !data.calib_K[cam_idx].empty() && !data.calib_D[cam_idx].empty()) {
                    std::vector<cv::Point2f> src_pts(1, four_corners[j]);
                    std::vector<cv::Point2f> undist_pts;
                    const cv::Mat& Kc = data.calib_K[cam_idx];
                    const cv::Mat& Dc = data.calib_D[cam_idx];
                    cv::Mat Knew = (data.calib_Knew.size() > static_cast<size_t>(cam_idx) &&
                                    !data.calib_Knew[cam_idx].empty())
                                       ? data.calib_Knew[cam_idx]
                                       : Kc;
                    cv::fisheye::undistortPoints(src_pts, undist_pts, Kc, Dc,
                                                 cv::Mat::eye(3, 3, CV_64F), Knew);
                    if (!undist_pts.empty()) {
                        undistorted_pt = undist_pts[0];
                    }
                }
                four_corners_warp[j] = data.warper->warpPoint(undistorted_pt, K_stitch, data.cameras[cam_idx].R);
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

            if (crop_or_not_ && !data.crop_roi.empty()) {
                top_left.x -= data.crop_roi.x;
                top_left.y -= data.crop_roi.y;
                bottom_right.x -= data.crop_roi.x;
                bottom_right.y -= data.crop_roi.y;
            }
            
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
            if (draw_box_or_not_) {
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



// ===================================================其他辅助函数的实现==================================================

// 读取鱼眼相机标定参数的辅助函数
std::string JHStitcher::trimAscii(const std::string& input)
{
    size_t first = input.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    size_t last = input.find_last_not_of(" \t\r\n");
    return input.substr(first, last - first + 1);
}

bool JHStitcher::parseFisheyeYaml(const std::string& filename, cv::Mat& K, cv::Mat& D)
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

bool JHStitcher::readFisheyeCalibration(const std::string& filename, cv::Mat& K, cv::Mat& D)
{
    std::cerr << "Reading fisheye calibration from: " << filename << std::endl;
    return parseFisheyeYaml(filename, K, D);
}


std::string JHStitcher::mapPath(const std::string& filename, const char* base, int idx)
{
    return std::string(filename) + "/" + base + "_" + std::to_string(idx) + ".exr";
}

bool JHStitcher::ensureDir(const std::string& filename )
{
    std::filesystem::path path(filename);
    std::filesystem::path dir = path.has_extension() ? path.parent_path() : path;
    if (dir.empty()) {
        return false;
    }
    std::error_code ec;
    if (std::filesystem::exists(dir, ec)) {
        return true;
    }
    return std::filesystem::create_directories(dir, ec);
}

bool JHStitcher::saveRemapMaps(const std::string& filename, const TransformationData& data, int num_images)
{
    if (!ensureDir(filename)) {
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
        if (!cv::imwrite(mapPath(filename, "mapIU_x", i), data.mapIU_x[i])) {
            return false;
        }
        if (!cv::imwrite(mapPath(filename, "mapIU_y", i), data.mapIU_y[i])) {
            return false;
        }
        if (!cv::imwrite(mapPath(filename, "mapPU_x", i), data.mapPU_x[i])) {
            return false;
        }
        if (!cv::imwrite(mapPath(filename, "mapPU_y", i), data.mapPU_y[i])) {
            return false;
        }
    }
    return true;
}

bool JHStitcher::loadRemapMaps(const std::string& filename, int num_images, TransformationData& data)
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
        data.mapIU_x[i] = cv::imread(mapPath(filename, "mapIU_x", i), cv::IMREAD_UNCHANGED);
        data.mapIU_y[i] = cv::imread(mapPath(filename, "mapIU_y", i), cv::IMREAD_UNCHANGED);
        data.mapPU_x[i] = cv::imread(mapPath(filename, "mapPU_x", i), cv::IMREAD_UNCHANGED);
        data.mapPU_y[i] = cv::imread(mapPath(filename, "mapPU_y", i), cv::IMREAD_UNCHANGED);

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

bool JHStitcher::saveStitchCache(const std::string& filename, const TransformationData& data)
{
    if (!ensureDir(filename)) {
        return false;
    }
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        return false;
    }

    fs << "num_cameras" << static_cast<int>(data.cameras.size());
    fs << "cameras" << "[";
    for (const auto& cam : data.cameras) {
        fs << "{";
        fs << "FOV_hor" << FOV_hor_;
        fs << "FOV_ver" << FOV_ver_;
        fs << "focal" << cam.focal;
        fs << "aspect" << cam.aspect;
        fs << "ppx" << cam.ppx;
        fs << "ppy" << cam.ppy;
        fs << "R" << cam.R;
        fs << "t" << cam.t;
        fs << "K_matrix" << cam.K();
        fs << "}";
    }
    fs << "]";
    fs << "corners" << data.corners;
    fs << "sizes" << data.sizes;
    fs << "dxdy" << data.dxdy;
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

    fs << "calib_K" << "[";
    for (const auto& K : data.calib_K) {
        fs << K;
    }
    fs << "]";
    fs << "calib_D" << "[";
    for (const auto& D : data.calib_D) {
        fs << D;
    }
    fs << "]";
    fs << "calib_Knew" << "[";
    for (const auto& Knew : data.calib_Knew) {
        fs << Knew;
    }
    fs << "]";

    fs.release();
    return true;
}

bool JHStitcher::loadStitchCache(const std::string& filename, TransformationData& data)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
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

    data.calib_K.clear();
    cv::FileNode calib_k_node = fs["calib_K"];
    if (calib_k_node.type() == cv::FileNode::SEQ) {
        for (auto it = calib_k_node.begin(); it != calib_k_node.end(); ++it) {
            cv::Mat K;
            (*it) >> K;
            data.calib_K.push_back(K);
        }
    }

    data.calib_D.clear();
    cv::FileNode calib_d_node = fs["calib_D"];
    if (calib_d_node.type() == cv::FileNode::SEQ) {
        for (auto it = calib_d_node.begin(); it != calib_d_node.end(); ++it) {
            cv::Mat D;
            (*it) >> D;
            data.calib_D.push_back(D);
        }
    }

    data.calib_Knew.clear();
    cv::FileNode calib_knew_node = fs["calib_Knew"];
    if (calib_knew_node.type() == cv::FileNode::SEQ) {
        for (auto it = calib_knew_node.begin(); it != calib_knew_node.end(); ++it) {
            cv::Mat Knew;
            (*it) >> Knew;
            data.calib_Knew.push_back(Knew);
        }
    }

    float avg_focal = 0.0f;
    for (const auto& cam : data.cameras) {
        avg_focal += static_cast<float>(cam.focal);
    }
    avg_focal /= data.cameras.size();
    data.warper = makePtr<cv::detail::CylindricalWarperGpu>(avg_focal);

    data.compensator = makePtr<GainCompensator>();

    data.seam_finder = makePtr<DpSeamFinder>(DpSeamFinder::COLOR_GRAD);

    fs.release();
    return true;
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

// 计算指定列的白色像素比例
float JHStitcher::getWhiteRatioCol(const cv::Mat& mask, int col) {
    if (mask.empty() || col < 0 || col >= mask.cols) {
        return 0.0f;
    }

    CV_Assert(mask.channels() == 1);

    int whiteCount = 0;
    for (int row = 0; row < mask.rows; ++row) {
        if (mask.at<uchar>(row, col) == 255) {
            whiteCount++;
        }
    }

    return static_cast<float>(whiteCount) / mask.rows;
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
