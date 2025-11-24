#include "JHstitcher.hpp"
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <shared_mutex>
#include <rclcpp/rclcpp.hpp>
#include "opencv2/xfeatures2d/nonfree.hpp" // 引入非自由特征检测模块（如SIFT、SURF等）

using namespace std;
using namespace cv;
using namespace cv::detail;

JHStitcher::JHStitcher(
    std::unordered_map<std::string, int>& cam_name_to_idx,
    int refresh_time,
    size_t min_keypoints,
    double min_confidence,
    int min_inliers,
    double max_focal_variance,
    double y_tolerance,
    float roi_threshold,
    float detect_confidence,
    float iou_threshold,
    double scale,
    bool cropornot,
    bool drawboxornot,
    bool save_CameraParams,
    string save_CameraParams_path,
    bool use_saved_CameraParams
):  
    cam_name_to_idx_(cam_name_to_idx),
    refresh_time_(refresh_time),
    min_keypoints_(min_keypoints),
    min_confidence_(min_confidence),
    min_inliers_(min_inliers),
    max_focal_variance_(max_focal_variance),
    y_tolerance_(y_tolerance),
    roi_threshold_(roi_threshold),
    detect_confidence_(detect_confidence),
    iou_threshold_(iou_threshold),
    scale_(scale),
    cropornot_(cropornot),
    drawboxornot_(drawboxornot),
    save_CameraParams_(save_CameraParams),
    save_CameraParams_path_(save_CameraParams_path),
    use_saved_CameraParams_(use_saved_CameraParams)

    {
    latest_warp_images_32F.resize(3);
    
    // 此处可临时添加打印，验证初始化是否正确
    // std::cout << "cam_name_to_idx_ 初始化相机映射：" << std::endl;
    // for (const auto& [name, idx] : cam_name_to_idx_) {
    //     std::cout << "名称: " << name << ", 索引: " << idx << std::endl;
    // }
}

bool JHStitcher::processFirstGroupImpl(const std::vector<cv::Mat>& images) {

    int num_images = images.size();
        // cout<<"处理第一组图片，相机数量:"<< num_images << endl;

        // create TransformationData对象
        TransformationData data;


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
        // 使用USRF特征检测器
        // Ptr<SURF> featurefinder = SURF::create();
        // cout<< "开始特征检测" << endl;
        // 使用AKAZE特征检测器
        Ptr<AKAZE> featurefinder = AKAZE::create();


        for (int i = 0; i < num_images; ++i) {
            images_sizes[i] = images[i].size();
            computeImageFeatures(featurefinder, images[i], features[i]);
            // 绘制特征点并保存
            // Mat img_with_keypoints;
            // drawKeypoints(images[i], features[i].keypoints, img_with_keypoints, 
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
                    drawMatches(images[i], features[i].keypoints, images[j], features[j].keypoints,
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
                // cout<<"在光束平差后，相机参数：" << endl;
                // std::cout << "camera #" << i + 1 << ":\n内参数矩阵K:\n" << cameras[i].K() << "\n旋转矩阵R:\n" << cameras[i].R << "\n焦距focal: " << cameras[i].focal << std::endl;
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

        // 创建掩码
        std::vector<Mat> masks(num_images);
        for (int i = 0; i < num_images; ++i) {
            masks[i].create(images[i].size(), CV_8U);
            masks[i].setTo(Scalar::all(255));
        }

        // 图像扭曲
        // cout<<"开始图像扭曲" << endl;
        std::vector<Mat> masks_warp(num_images);
        std::vector<Mat> images_warp(num_images);
        std::vector<Point> corners(num_images);
        std::vector<Size> sizes(num_images);

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
            {
                std::lock_guard<std::mutex> lock(transformation_mutex_); // 确保线程安全和效率
                corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warp[i]);
                warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warp[i]);
            }
            
            sizes[i] = images_warp[i].size();
            // cout<<"processFirstGroupImpl 掩码尺寸 = "<< masks_warp[i].size()<<endl;
            // cout<<"corners[" << i << "]: " << corners[i] << ", sizes[" << i << "]: " << sizes[i] << endl;
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
            cv::UMat img_umat = images_warp[i].getUMat(ACCESS_RW);
            cv::UMat mask_umat = masks_warp[i].getUMat(ACCESS_RW);
            compensator->apply(i, corners[i], img_umat, mask_umat);
            img_umat.copyTo(images_warp[i]);
            mask_umat.copyTo(masks_warp[i]);
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
        for (int i = 0; i < num_images; ++i) {
            // cout<<"在第一次处理组中，检查图像和掩码类型" << endl;
            // cout<< "图像期望为CV_32F->21,实际为->"<< images_warp_umat[i].type()<<endl;
            // cout<< "掩码期望为CV_38U->0,实际为->"<< masks_warp_umat[i].type()<<endl;
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
        auto end_time_match = chrono::high_resolution_clock::now();
        chrono::duration<double> duration_time_match = end_time_match - start_time_match;
        cout << "缝合线查找耗时: " << duration_time_match.count() << " seconds." << endl;

        // 图像融合
        // cout<<"开始图像融合" << endl;
        Ptr<Blender> blender = Blender::createDefault(Blender::NO, false);
        blender->prepare(corners, sizes);
        for (int i = 0; i < num_images; ++i) {
            Mat img_warp_16s;
            images_warp[i].convertTo(img_warp_16s, CV_16S);
            blender->feed(img_warp_16s, masks_warp[i], corners[i]);
        }

        Mat result, result_mask;
        blender->blend(result, result_mask);

        // 裁剪图像
        // cout<<"开始裁剪图像" << endl;
        Rect crop_roi = getValidRows(result_mask, roi_threshold_);
        if (crop_roi.empty()) {
            // cout<<"图像裁剪失败"<<endl;
            return false;
        }

        data.cameras = cameras;
        data.warper = warper;
        data.compensator = compensator;
        data.seam_finder = seam_finder;
        data.corners = corners;
        data.sizes = sizes;
        data.masks = masks_warp;
        data.dxdy = prepare_pxpy(data.corners,data.sizes);
        // data.crop_roi = crop_roi;
        // 裁剪区域也要缩放
        data.crop_roi = Rect(static_cast<int>(crop_roi.x*scale_),static_cast<int>(crop_roi.y*scale_),static_cast<int>(crop_roi.width*scale_),static_cast<int>(crop_roi.height*scale_)); 

        // 保存变换数据
        std::lock_guard<std::mutex> lock(transformation_mutex_);
        transformation_data_ = data; //transformation_data_为全局参数
        return true;
}

cv::Mat JHStitcher::processSubsequentGroupImpl
    (const std::vector<cv::Mat>& images,
     const std::unordered_map<std::string, std::vector<DetectionResult>>& latest_detections_) 
    {
    // 新建data用于规避拼接线检测时的读写冲突
        TransformationData data;
        {
            std::lock_guard<std::mutex> lock(transformation_mutex_);
            if (transformation_data_.cameras.empty()) {
                // cout<<"无有效变换数据，跳过后续处理"<<endl;
                return cv::Mat();
            }
            data = transformation_data_;
        }

    size_t num_images = images.size();
    if (num_images != data.cameras.size()) {
        // std::cout<< "图像数量与相机参数不匹配" <<std::endl;
        return cv::Mat();
    }

    // 图像扭曲
    // 计算耗时
    auto start_time_warp = chrono::high_resolution_clock::now();
    std::vector<cv::Mat> images_warp(num_images);
    for (size_t i = 0; i < num_images; ++i) {
        cv::Mat K;
        data.cameras[i].K().convertTo(K, CV_32F);
        data.warper->warp(images[i], K, data.cameras[i].R, cv::INTER_LINEAR, cv::BORDER_REFLECT, images_warp[i]);
    }
    auto end_time_warp = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_time_warp = end_time_warp - start_time_warp;
    cout << "图像扭曲耗时: " << duration_time_warp.count() << " seconds." << endl;

    // 写入时加独占锁（阻塞所有读和写）
    std::unique_lock<std::shared_mutex> lock(rw_mutex_);
    for (size_t i = 0; i < images_warp.size(); ++i) {
        images_warp[i].copyTo(latest_warp_images_32F[i]);
    }

    // 图像融合
    cv::Ptr<Blender> blender = Blender::createDefault(Blender::NO, false);
    blender->prepare(data.corners, data.sizes);
    for (size_t i = 0; i < num_images; ++i) {
        Mat img_warp_16s;
        images_warp[i].convertTo(img_warp_16s, CV_16S);
        blender->feed(img_warp_16s, data.masks[i], data.corners[i]);
    }

    Mat pano_result, result_mask;
    blender->blend(pano_result, result_mask);

    cv::Mat resized_pano_result = Mat();
    resize(pano_result,resized_pano_result,cv::Size(static_cast<int>(pano_result.cols*scale_),static_cast<int>(pano_result.rows*scale_)),0,0,cv::INTER_LINEAR);
    cout<<"resized_pano_result size = "<<resized_pano_result.size()<<endl;
    // 判断裁剪resized图像有效性
    if (data.crop_roi.empty() || data.crop_roi.x < 0 || data.crop_roi.y < 0 || 
        data.crop_roi.x + data.crop_roi.width > resized_pano_result.cols || 
        data.crop_roi.y + data.crop_roi.height > resized_pano_result.rows) {
        cout<<"裁剪区域无效"<<endl;
        return Mat();
    }
    
    // cout<<"开始过滤检测框"<<endl;
    filtBoxes(resized_pano_result,data,latest_detections_);
    

    // cout<<"image_warp size = "<<images_warp[0].size()<<endl;
    // cout<<"单张图片大小 = "<<images[0].size()<<endl;
    // cout<<"掩码大小 = "<<data.masks[0].size()<<endl;
    // cout <<"data.crop_roi = "<< data.crop_roi << endl;

    // 返回裁剪图像
    if(cropornot_){
        return  resized_pano_result(data.crop_roi);
    }
    else{
        return resized_pano_result;
    }
}

void JHStitcher::detectStitchLine() 
{
    std::vector<cv::UMat> images;
    try {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        //latest_warp_images_32F是一个全局变量，存储了最新的变形图像
        images.push_back(latest_warp_images_32F[0].clone());
        images.push_back(latest_warp_images_32F[1].clone());
        images.push_back(latest_warp_images_32F[2].clone());
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

void JHStitcher::filtBoxes(
    cv::Mat& pano, 
    const TransformationData& data,
    const std::unordered_map<std::string, std::vector<DetectionResult>>& detections) {
    // 0. 清空之前的检测框（避免累积旧数据）
    filtered_boxes_.clear();
    // 0.5. 
    cout<<"filtBoxes缩放比例: "<< scale_ << endl;


    // 1. 收集所有转换后的检测框信息
    std::vector<BoxInfo> all_boxes;
    
    for (const auto& [cam_name, results] : detections) {
        auto it = cam_name_to_idx_.find(cam_name);
        if (it == cam_name_to_idx_.end()) {
            // cout<<"未知相机名称: "<<cam_name.c_str()<<endl;
            continue;
        }
        int cam_idx = it->second;
        // cout << "cam_idx = " << cam_idx << endl;
        Mat K;
        data.cameras[cam_idx].K().convertTo(K, CV_32F);
        // cout << "收到的相机名称: " << cam_name << "，对应索引: " << cam_idx << endl;

        Rect dst_roi = Rect(data.corners[0], data.sizes[0]);
        for (size_t i = 1; i < data.corners.size(); ++i)
            dst_roi |= Rect(data.corners[i], data.sizes[i]);

        // cout << "dst_roi in filtBoxes = " << dst_roi << endl;
        for (const auto& dr : results) {
            if (dr.confidence < detect_confidence_) continue; // 过滤低置信度

            // 检测框的两个对角点（左上角：dr.x1,dr.y1；右下角：dr.x2,dr.y2）
            std::vector<cv::Point2f> four_corners(2); // 存储检测框的2个对角点
            four_corners[0] = cv::Point2f(dr.x1, dr.y1); // 原始检测框左上角
            four_corners[1] = cv::Point2f(dr.x2, dr.y2); // 原始检测框右下角


            // 计算投影后的的2个角点
            std::vector<cv::Point2f> four_corners_warp(2); // 与 four_corners 尺寸一致（2个点）
            // 投影检测框的角点（使用当前相机的内参 K 和旋转矩阵 R）
            for (int j = 0; j < 2; ++j) { // 循环范围修正为 0~1（对应2个角点）
                four_corners_warp[j] = data.warper->warpPoint(four_corners[j], K, data.cameras[cam_idx].R);
            }
            // cout<< "原始检测框角点: 左上(" << four_corners[0].x << ", " << four_corners[0].y 
            //     << "), 右下(" << four_corners[1].x << ", " << four_corners[1].y << ")" << endl;
            // cout<< "投影后检测框角点: 左上(" << four_corners_warp[0].x << ", " << four_corners_warp[0].y 
            //     << "), 右下(" << four_corners_warp[1].x << ", " << four_corners_warp[1].y << ")" << endl;

            // 【坐标转换逻辑修正】基于投影后的角点，叠加 dxdy 偏移并缩放
            cv::Point2i top_left(
                cvRound((four_corners_warp[0].x - dst_roi.x) * scale_ ),//+ data.dxdy[cam_idx].x * scale_), // 投影后左上角 + 偏移 + 缩放
                cvRound((four_corners_warp[0].y - dst_roi.y) * scale_ )//+ data.dxdy[cam_idx].y * scale_)
            );
            cv::Point2i bottom_right(
                cvRound((four_corners_warp[1].x - dst_roi.x) * scale_ ),//+ data.dxdy[cam_idx].x * scale_), // 投影后右下角 + 偏移 + 缩放
                cvRound((four_corners_warp[1].y - dst_roi.y) * scale_ )//+ data.dxdy[cam_idx].y * scale_)
            );
            // cout<< "data.dxdy偏移: (" << data.dxdy[cam_idx].x << ", " << data.dxdy[cam_idx].y << ")" << endl;
            // cout<< "转换后检测框坐标: 左上(" << top_left.x << ", " << top_left.y 
            //     << "), 右下(" << bottom_right.x << ", " << bottom_right.y << ")" << endl;

             // 确保检测框坐标非负（避免超出图像范围）
            top_left.x = std::max(0, top_left.x);
            top_left.y = std::max(0, top_left.y);
            bottom_right.x = std::max(top_left.x + 1, bottom_right.x); // 确保宽度≥1
            bottom_right.y = std::max(top_left.y + 1, bottom_right.y); // 确保高度≥1

            all_boxes.push_back({top_left, bottom_right, dr.class_name, dr.confidence});
        }
    }

    // 2. 按类别分组
    std::unordered_map<std::string, std::vector<BoxInfo>> class_boxes;
    for (const auto& box : all_boxes) {
        class_boxes[box.class_name].push_back(box);
    }

    // 3. 每组内筛选：保留重叠区域中置信度最高的框
    for (const auto& [cls, boxes] : class_boxes) {
        std::vector<bool> is_processed(boxes.size(), false);

        for (size_t i = 0; i < boxes.size(); ++i) {
            if (is_processed[i]) continue;

            // 找到当前框的所有重叠框
            std::vector<BoxInfo> overlapping_group;
            overlapping_group.push_back(boxes[i]);

            for (size_t j = i + 1; j < boxes.size(); ++j) {
                if (is_processed[j]) continue;

                Rect rect_i(boxes[i].top_left, boxes[i].bottom_right);
                Rect rect_j(boxes[j].top_left, boxes[j].bottom_right);
                float iou = calculateIoU(rect_i, rect_j);

                if (iou > iou_threshold_) {
                    overlapping_group.push_back(boxes[j]);
                    is_processed[j] = true;
                }
            }

            // 保留组内置信度最高的框
            if (!overlapping_group.empty()) {
                auto max_conf_it = std::max_element(
                    overlapping_group.begin(),
                    overlapping_group.end(),
                    [](const BoxInfo& a, const BoxInfo& b) {
                        return a.confidence < b.confidence;
                    }
                );
                filtered_boxes_.push_back(*max_conf_it);
            }
        }
    }
    // std::cout << "filtered_boxes_ 筛选后的检测框数量: " << filtered_boxes_.size() << std::endl;
    // for (const auto& box : filtered_boxes_) {
        // std::cout << "检测框: " << box.class_name 
        //           << ", 置信度: " << box.confidence
        //           << ", 坐标: (" << box.top_left.x << ", " << box.top_left.y << ") - ("
        //           << box.bottom_right.x << ", " << box.bottom_right.y << ")" << std::endl;
    // }
    if(drawboxornot_)
    {
        // 新增功能：将最终检测框绘制到pano上
        if (pano.empty()) {
            std::cerr << "警告：全景图为空，无法绘制检测框！" << std::endl;
            return;
        }
        // 为不同类别设置不同颜色（可根据需要扩展）
        std::unordered_map<std::string, cv::Scalar> class_colors;
        cv::RNG rng(12345); // 随机数生成器，确保同类别颜色一致

        for (const auto& box : filtered_boxes_) {
            // 为未分配颜色的类别生成随机颜色
            if (class_colors.find(box.class_name) == class_colors.end()) {
                class_colors[box.class_name] = cv::Scalar(
                    rng.uniform(0, 256),
                    rng.uniform(0, 256),
                    rng.uniform(0, 256)
                );
            }
            cv::Scalar color = class_colors[box.class_name];

            // 绘制矩形框
            cv::rectangle(
                pano,
                box.top_left,
                box.bottom_right,
                color,
                2 // 线宽
            );

            // 准备显示文本（类别 + 置信度）
            std::string text = box.class_name + " (" + std::to_string(int(box.confidence * 100)) + "%)";
            cv::Point text_pos = box.top_left + cv::Point(2, 15); // 文本位置（框左上角偏移）

            // 绘制文本背景（避免文字与图像重叠难以辨认）
            int base_line;
            cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base_line);
            cv::rectangle(
                pano,
                text_pos - cv::Point(1, base_line),
                text_pos + cv::Point(text_size.width, text_size.height),
                color,
                -1 // 填充背景
            );

            // 绘制文本
            cv::putText(
                pano,
                text,
                text_pos,
                cv::FONT_HERSHEY_SIMPLEX,
                0.5, // 字体大小
                cv::Scalar(255, 255, 255), // 白色文字
                1 // 文字线宽
            );
        }
    std::cout << "已在全景图上绘制 " << filtered_boxes_.size() << " 个检测框" << std::endl;

    }
    

    

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

float JHStitcher::calculateIoU(const cv::Rect& a, const cv::Rect& b) {
    int x1 = std::max(a.x, b.x);
    int y1 = std::max(a.y, b.y);
    int x2 = std::min(a.x + a.width, b.x + b.width);
    int y2 = std::min(a.y + a.height, b.y + b.height);
    
    int intersection_area = std::max(0, x2 - x1) * std::max(0, y2 - y1);
    int area_a = a.width * a.height;
    int area_b = b.width * b.height;
    int union_area = area_a + area_b - intersection_area;
    
    if (union_area == 0) return 0.0f;
    return static_cast<float>(intersection_area) / union_area;
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
                std::cerr << "Error: 有效匹配对的最低置信度仅为:" + to_string(match_info.confidence) + "低于阈值"<< to_string(detect_confidence_) <<",换一帧检测" << std::endl;
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
    // 2. 检查x坐标是否递减分布
    for (size_t i = 1; i < four_corners_warp.size(); ++i) {
        if (four_corners_warp[i][0].x >= four_corners_warp[i-1][0].x) {
            std::cerr << "错误：图像 " << i-1 << " 和 " << i << " 的x坐标未递增（" 
                    << four_corners_warp[i-1][0].x << " >= " << four_corners_warp[i][0].x << "）" << std::endl;
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