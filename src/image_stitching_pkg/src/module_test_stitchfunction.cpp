// 离线拼接测试节点，用于测试各种拼接模块的效果，打算后续测试手动修改相机内外参矩阵的效果
// 相较于origin添加了seam拼缝检测和曝光补偿
// 并且添加了图像缩放功能
// 比较了各类曝光补偿器、拼缝查找器、融合器的性能
// 目前不缩放图片时，单帧处理时间为90ms浮动，长宽缩放一半时为40ms左右
#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/opencv_modules.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
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
#include <chrono> // 添加头文件


using namespace cv;
using namespace std;
using namespace cv::detail;

// 定义拼接图片的缩放比例
const double scale = 1; // 1.0表示不缩放，0.5表示缩小一半
// 特征点匹配的最低置信度阈值
const double MIN_CONFIDENCE = 0.8;  


// 曝光补偿器类型名获取函数
std::string getCompensatorTypeName(const cv::Ptr<cv::detail::ExposureCompensator>& compensator) {
    using namespace cv::detail;
    if (dynamic_cast<GainCompensator*>(compensator.get())) return "GainCompensator";
    if (dynamic_cast<ChannelsCompensator*>(compensator.get())) return "ChannelsCompensator";
    if (dynamic_cast<BlocksGainCompensator*>(compensator.get())) return "BlocksGainCompensator";
    if (dynamic_cast<BlocksChannelsCompensator*>(compensator.get())) return "BlocksChannelsCompensator";
    return "UnknownCompensator";
}

// 缩放图像（保持比例）
cv::Mat scaleImage(const cv::Mat& img, double scale) {
    cv::Mat scaled;
    cv::resize(img, scaled, cv::Size(), scale, scale, cv::INTER_AREA); // 按比例缩小，INTER_AREA适合缩小
    return scaled;
}

// 还原缝合线掩码到原始尺寸（双线性插值，保证边缘平滑）
cv::Mat upsaleMask(const cv::Mat& mask, const cv::Size& original_size) {
    cv::Mat scaled_mask;
    cv::resize(mask, scaled_mask, original_size, 0, 0, cv::INTER_LINEAR); // 线性插值还原
    scaled_mask.convertTo(scaled_mask, CV_8U); // 确保掩码类型正确
    return scaled_mask;
}

cv::Mat cropImage(const cv::Mat& result, const cv::Point& top_left, const cv::Point& bottom_right) {
    // 确定矩形区域（确保坐标有效）
    int x = std::max(0, top_left.x);
    int y = std::max(0, top_left.y);
    int width = std::min(bottom_right.x - x, result.cols - x);
    int height = std::min(bottom_right.y - y, result.rows - y);

    // 确保宽度和高度至少为1
    width = std::max(1, width);
    height = std::max(1, height);

    // 创建ROI并复制到新图像
    cv::Rect roi(x, y, width, height);
    cv::Mat cropped = result(roi).clone(); // 使用clone()确保数据被复制

    return cropped;
}

// 在processFirstGroup函数中添加此筛选逻辑
bool checkAndFilterMatches(std::vector<cv::detail::MatchesInfo>& pairwise_matches) {
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
            // 有效匹配对：检查置信度是否达标
            if (match_info.confidence < MIN_CONFIDENCE) {
                // 置信度不足，返回失败
                return false;
            }
        }
    }
    
    // 所有有效匹配对均满足置信度要求
    return true;
}

// 定义一个结构体来保存变换数据
struct TransformationData {
    vector<CameraParams> cameras;
    Ptr<cv::detail::CylindricalWarperGpu> warper;
    Ptr<detail::ExposureCompensator> compensator;
    Ptr<SeamFinder> seam_finder;
    vector<Point> corners;
    vector<Size> sizes;
    vector<Mat> masks;
    Ptr<Blender> blender;
    Rect roi;
};

// 函数用于处理第一组图片并保存变换数据
TransformationData processFirstGroup(const vector<String>& image_names) {
    int num_images = image_names.size(); //图片数量
    //存储图像、尺寸、特征点
        // 记录处理开始时间
    auto start_time_features = chrono::high_resolution_clock::now();

    vector<ImageFeatures> features(num_images);  //存储图像特征点
    vector<Mat> images(num_images); //存储所有图像
    vector<Size> images_sizes(num_images); //存储图像的尺寸
    cv::Ptr<cv::AKAZE> featurefinder = cv::AKAZE::create();

    for (int i = 0; i < num_images; ++i)
    {
        images[i] = cv::imread(samples::findFile(image_names[i]));//读取每一张图片
        cout<< "Image " << i + 1 << ": " << image_names[i] << endl;
        imshow("Image " + to_string(i + 1), images[i]); // 显示图像
        waitKey(0); // 等待用户按键
        // 用金字塔方式读取图像
        images[i] = scaleImage(images[i], scale); // 缩放图像
        images_sizes[i] = images[i].size();
        computeImageFeatures(featurefinder, images[i], features[i]);    //计算图像特征
        // cout << "image #" << i + 1 << "特征点为: " << features[i].keypoints.size() << " 个"<<"  "<< "尺寸为: " << images_sizes[i] << endl;
    }
    // cout << endl;
            // 记录处理结束时间
    auto end_time_features = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_time_features = end_time_features - start_time_features;
    cout << "Processing in 特征点检测 took: " << duration_time_features.count() << " seconds." << endl;
// ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

    //图像特征点匹配
        // 记录处理开始时间
    auto start_time_match = chrono::high_resolution_clock::now();

    vector<MatchesInfo> pairwise_matches; //表示特征匹配信息变量
    // 一共有AffineBestOf2NearestMatcher BestOf2NearestMatcher BestOf2NearestRangeMatcher 这三个
    // // 使用CPU特征匹配器
    // Ptr<FeaturesMatcher> matcher = makePtr<BestOf2NearestMatcher>(false, 0.3f, 6, 6); //定义特征匹配器，2NN方法
    // 使用GPU特征匹配器
    // Ptr<FeaturesMatcher> matcher = makePtr<BestOf2NearestMatcher>(true, 0.3f, 6, 6); //定义特征匹配器，2NN方法
    Ptr<FeaturesMatcher> matcher = makePtr<BestOf2NearestRangeMatcher>(1, true, 0.3f, 6, 6); //定义特征匹配器，最邻2NN方法

    (*matcher)(features, pairwise_matches);  //进行特征匹配

        // 执行筛选和置信度检查
    if (!checkAndFilterMatches(pairwise_matches)) {
        // 若检查失败，输出错误信息并退出函数
        std::cerr << "Error: 有效匹配对的置信度低于阈值0.8" << std::endl;
        return TransformationData();  // 返回空数据表示失败
    }

// （弃用）关键修改：过滤非相邻图像对的匹配结果，只保留i和i+1的相邻对
// 方法一，使用索引
// for (int i = 0; i < num_images; ++i) {
//     for (int j = i + 1; j < num_images; ++j) {
//         // 计算当前匹配对在pairwise_matches中的索引
//         int idx = i * num_images + j;
        
//         // 只保留相邻的匹配对（j = i + 1），其他标记为无效
//         if (j != i + 1) {
//             pairwise_matches[idx].matches.clear();        // 清空匹配点
//             pairwise_matches[idx].inliers_mask.clear();   // 清空内点掩码
//             pairwise_matches[idx].num_inliers = 0;        // 内点数设为0
//             pairwise_matches[idx].H = cv::Mat();          // 清空单应矩阵
//             pairwise_matches[idx].confidence = 0.0;       // 置信度设为0
//         }
//     }
// }

        // 记录处理结束时间
    auto end_time_match = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_time_match = end_time_match - start_time_match;
    cout << "Processing in 特征点匹配 took: " << duration_time_match.count() << " seconds." << endl;
// ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

    //预估相机参数
        // 记录处理开始时间
    auto start_time_est = chrono::high_resolution_clock::now();

    Ptr<Estimator> estimator;
    estimator = makePtr<HomographyBasedEstimator>();  //水平估计
    vector<CameraParams> cameras;  //相机参数素组
    (*estimator)(features, pairwise_matches, cameras);  //得到相机参数
    // cout << "预估相机参数:" << endl;
    for (size_t i = 0; i < cameras.size(); ++i)
    {
        Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras[i].R = R;
        // cout << "camera #" << i + 1 << ":\n内参数矩阵K:\n" << cameras[i].K() << "\n旋转矩阵R:\n" << cameras[i].R << "\n焦距focal: " << cameras[i].focal << endl;
    }
    // cout << endl;
        // 记录处理结束时间
    auto end_time_est = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_time_est = end_time_est - start_time_est;
    cout << "Processing in 预估相机参数 took: " << duration_time_est.count() << " seconds." << endl;
// ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

    // 光束平差，精确相机参数
        // 记录处理开始时间
    auto start_time_BA = chrono::high_resolution_clock::now();

    Ptr<detail::BundleAdjusterBase> adjuster;
    adjuster = makePtr<detail::BundleAdjusterRay>();
    (*adjuster)(features, pairwise_matches, cameras);
    // cout << "精确相机参数" << endl;
    for (size_t i = 0; i < cameras.size(); ++i)
    {
        Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras[i].R = R;
        // cout << "camera #" << i + 1 << ":\n内参数矩阵K:\n" << cameras[i].K() << "\n旋转矩阵R:\n" << cameras[i].R << "\n焦距focal: " << cameras[i].focal << endl;
    }
    // cout << endl;
        // 记录处理结束时间
    auto end_time_BA = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_time_BA = end_time_BA - start_time_BA;
    cout << "Processing in 光束平差 took: " << duration_time_BA.count() << " seconds." << endl;
// ———————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————— 
    
    // 波形矫正
        // 记录处理开始时间
    auto start_time_waveCorrect = chrono::high_resolution_clock::now();

    vector<Mat> mat;
    for (size_t i = 0; i < cameras.size(); ++i)
        mat.push_back(cameras[i].R);
    waveCorrect(mat, WAVE_CORRECT_HORIZ); //水平校正
    for (size_t i = 0; i < cameras.size(); ++i)
        cameras[i].R = mat[i];

    // //创建mask图像
    vector<Mat> masks(num_images);
    for (int i = 0; i < num_images; ++i)
    {
        masks[i].create(images[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }
       // 记录处理结束时间
    auto end_time_waveCorrect = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_time_waveCorrect = end_time_waveCorrect - start_time_waveCorrect;
    cout << "Processing in 波形矫正 took: " << duration_time_waveCorrect.count() << " seconds." << endl;
// ———————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————


    //图像、掩码变换
        // 记录处理开始时间
    auto start_time_warper = chrono::high_resolution_clock::now();

    vector<Mat> masks_warp(num_images);  //mask扭曲
    vector<Mat> images_warp(num_images); //图像扭曲
    vector<Point> corners(num_images);   //图像左角点
    vector<Size> sizes(num_images);		 //图像尺寸

    vector<Point2f> four_corners(4); //四个角点
    vector<vector<Point2f>> four_corners_warp(num_images); //扭曲图像左角点
    // // cpu：使用 CylindricalWarperGpu 创建柱面投影器
    // Ptr<WarperCreator> warper_creator=makePtr<cv::CylindricalWarper>();  //柱面投影
    // Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(cameras[0].focal));  //因为图像焦距都差不多

    // gpu：直接创建 CylindricalWarperGpu 对象
    Ptr<cv::detail::CylindricalWarperGpu> warper = makePtr<cv::detail::CylindricalWarperGpu>(static_cast<float>(cameras[0].focal));
    
    for (int i = 0; i < num_images; ++i)
    { 
        Mat K;
        cameras[i].K().convertTo(K, CV_32F);
        corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warp[i]);  //扭曲图像images->images_warp
        sizes[i] = images_warp[i].size();
        warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warp[i]);  //扭曲masks->masks_warp


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
    
        // 记录处理结束时间
    auto end_time_warper = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_time_warper = end_time_warper - start_time_warper;
    cout << "Processing in 图像、掩码变换 took: " << duration_time_warper.count() << " seconds." << endl;
// ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

    // 修正曝光补偿器的创建和使用
// 记录处理开始时间
auto start_time_1 = chrono::high_resolution_clock::now();

// 定义缩放比例（如缩小为1/4，可根据需求调整）
const double scale = 1;

// 缩放图像和掩码用于计算曝光补偿参数
vector<UMat> scaled_images_warp_umat(num_images);
vector<UMat> scaled_masks_warp_umat(num_images);
vector<cv::Size> original_sizes(num_images); // 保存原始尺寸用于还原

for (int i = 0; i < num_images; ++i) {
    original_sizes[i] = images_warp[i].size(); // 记录原始尺寸
    cv::Mat scaled_img = scaleImage(images_warp[i], scale); // 缩小图像
    cv::Mat scaled_mask = scaleImage(masks_warp[i], scale); // 缩小掩码
    scaled_img.copyTo(scaled_images_warp_umat[i]);
    scaled_mask.copyTo(scaled_masks_warp_umat[i]);
}

// 在缩小图像上计算曝光补偿参数
// GainCompensator-0.0442372s
// ChannelsCompensator-0.0487535s
// BlocksGainCompensator-2.30637s
// BlocksChannelsCompensator-3.14044s
Ptr<detail::ExposureCompensator> compensator = makePtr<detail::ChannelsCompensator>();
std::string compensator_type = getCompensatorTypeName(compensator);
compensator->feed(corners, scaled_images_warp_umat, scaled_masks_warp_umat); // 注意：corner无需缩放（坐标对应原始图像）

// 将补偿参数应用于原始尺寸图像
for(int i = 0; i < num_images; ++i) {
    UMat img_umat, mask_umat;
    images_warp[i].copyTo(img_umat);
    masks_warp[i].copyTo(mask_umat);
    compensator->apply(i, corners[i], img_umat, mask_umat); // 直接用原始图像和掩码
    img_umat.copyTo(images_warp[i]);
    mask_umat.copyTo(masks_warp[i]);
}

// 记录处理结束时间
auto end_time_1 = chrono::high_resolution_clock::now();
chrono::duration<double> duration_time_1 = end_time_1 - start_time_1;
cout << "MAIN Processing in 曝光补偿 took: " << duration_time_1.count() << " seconds." << endl;
// ———————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
    // 寻找缝合线
        // 记录处理开始时间
auto start_time_2 = chrono::high_resolution_clock::now();

// 1、图割法查找 
// 注意：在缩小后的图像上进行，以提高速度和减少内存开销：*1-1.29s *0.9-1.05498s
// Ptr<SeamFinder> seam_finder = makePtr<GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);

// 2、动态规划缝合线查找（速度快，效果好）：*1-0.34071s *0.7-0.137051s
Ptr<SeamFinder> seam_finder = makePtr<DpSeamFinder>(DpSeamFinder::COLOR_GRAD); // 使用颜色梯度作为成本函数

// 3、Voronoi图缝合线查找（极快，适合实时）：*1-0.33803s
// Ptr<SeamFinder> seam_finder = makePtr<VoronoiSeamFinder>();
// 准备缩小后的图像和掩码用于计算缝合线


for (int i = 0; i < num_images; ++i) {
    cv::Mat scaled_img = scaleImage(images_warp[i], scale); // 同曝光补偿的缩放比例
    cv::Mat scaled_mask = scaleImage(masks_warp[i], scale);
    scaled_img.convertTo(scaled_img, CV_32F); // 图割法要求CV_32F
    scaled_mask.convertTo(scaled_mask, CV_8U);
    scaled_img.copyTo(scaled_images_warp_umat[i]);
    scaled_mask.copyTo(scaled_masks_warp_umat[i]);
    imwrite("image_warped_" + to_string(i) + "_" + compensator_type + ".png", scaled_images_warp_umat[i]); // 保存缩小后的图像
}

// 在缩小图像上计算缝合线
try {
    seam_finder->find(scaled_images_warp_umat, corners, scaled_masks_warp_umat); // corners仍为原始坐标
} catch (const cv::Exception& e) {
    cerr << "Error in seam finding: " << e.what() << endl;
    return TransformationData();
}

// 将缩小后的缝合线掩码还原到原始尺寸
for (int i = 0; i < num_images; ++i) {
    cv::Mat scaled_mask;
    scaled_masks_warp_umat[i].copyTo(scaled_mask);
    cv::Mat original_mask = upsaleMask(scaled_mask, original_sizes[i]); // 还原到原始尺寸
    original_mask.copyTo(masks_warp[i]); // 更新原始掩码
    imwrite("mask_warped_" + to_string(i) + "_" + compensator_type + ".png", original_mask); // 保存还原后的掩码
}

// 记录处理结束时间
auto end_time_2 = chrono::high_resolution_clock::now();
chrono::duration<double> duration_time_2 = end_time_2 - start_time_2;
cout << "MAIN Processing in 寻找缝合线 took: " << duration_time_2.count() << " seconds." << endl;
// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

// 保存images_warp中被masks_warp覆盖的有效区域
for (int i = 0; i < num_images; ++i) {
    // 创建与images_warp同尺寸的空白图像（背景为黑色）
    Mat masked_image = Mat::zeros(images_warp[i].size(), images_warp[i].type());
    // 将images_warp中掩码为255的区域复制到空白图像（即提取有效区域）
    images_warp[i].copyTo(masked_image, masks_warp[i]);
    
    // 保存结果（文件名包含索引，区分不同图像）
    string save_path = "masked_image_warp_" + to_string(i) + ".png";
    imwrite(save_path, masked_image);
    cout << "corners[i]: " << corners[i] << endl;
}
    //图像融合
// 改进方向：
/** 
    2. 优化方向（减少重复开销）
如果帧数极多（如每秒 30 帧以上）且对实时性要求较高，可通过以下方式优化：
（1）复用Blender实例（安全重置）
虽然Blender没有公开的reset方法，但可通过分析源码（结合你提供的blenders.hpp）手动清空内部状态，避免重复创建实例。以FeatherBlender为例：
cpp
运行
// 复用FeatherBlender实例的示例
Ptr<FeatherBlender> blender = makePtr<FeatherBlender>(0.02f); // 初始化一次

for (int frame_idx = 0; frame_idx < N; ++frame_idx) {
    // 1. 清空内部状态（关键步骤）
    blender->dst_.release();
    blender->dst_mask_.release();
    blender->weight_map_.release();
    blender->dst_weight_map_.release();
    blender->dst_roi_ = Rect(); // 重置输出区域

    // 2. 准备当前帧（同之前的逻辑）
    std::vector<Point> frame_corners(3);
    std::vector<Size> frame_sizes(3);
    // ... 填充当前帧数据

    blender->prepare(frame_corners, frame_sizes);

    // 3. 喂入图像并混合（同之前的逻辑）
    for (int i = 0; i < 3; ++i) {
        // feed图像...
    }
    Mat frame_result, frame_result_mask;
    blender->blend(frame_result, frame_result_mask);
}
注意：
不同Blender的内部变量不同（如MultiBandBlender有dst_pyr_laplace_等金字塔变量），重置时需针对性清空（参考头文件中的成员变量）。
此方法依赖Blender的内部实现，若 OpenCV 版本更新导致成员变量变化，可能需要调整重置逻辑。



（2）预分配内存（针对固定尺寸帧）
若所有帧的图像尺寸固定（输出dst_roi大小不变），可在首次创建Blender时预分配足够的内存，后续帧复用该内存：
cpp
运行
Ptr<FeatherBlender> blender = makePtr<FeatherBlender>(0.02f);

// 首次准备时确定固定的输出区域（假设所有帧的范围相同）
std::vector<Point> first_frame_corners(3);
std::vector<Size> first_frame_sizes(3);
// ... 填充第一帧数据
Rect fixed_roi = blender->prepare(first_frame_corners, first_frame_sizes);

for (int frame_idx = 0; frame_idx < N; ++frame_idx) {
    // 直接使用固定区域，避免重新分配内存
    blender->prepare(fixed_roi); 

    // 喂入当前帧图像并混合...
}**/


    auto start_subsequent_group_2 = chrono::high_resolution_clock::now();

    Ptr<Blender> blender; //定义图像融合器

// GPU版本：
//多频带融合Blender::MULTI_BAND-0.0872498s 羽化Blender::FEATHER-0.121885s
    blender = Blender::createDefault(Blender::FEATHER, true); 

// CPU版本：
    // blender = Blender::createDefault(Blender::NO, false); //简单融合方法

    blender->prepare(corners, sizes);  //生成全景图像区域
    for (int i = 0; i < num_images; ++i)
    {
        images_warp[i].convertTo(images_warp[i], CV_16S); 
        blender->feed(images_warp[i], masks_warp[i], corners[i]);  //处理图像 初始化数据
    }
    Mat result, result_mask;
    blender->blend(result, result_mask);

    // cout << "裁剪前的图像尺寸: " << result_mask.size() << endl;    
    
    auto end_subsequent_group_2 = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_subsequent_group_2 = end_subsequent_group_2 - start_subsequent_group_2;
    cout << "Processing in 修正图像融合 took: " << duration_subsequent_group_2.count() << " seconds." << endl;
// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

// 保存和剪裁第一组图像融合结果
    auto start_subsequent_group_3 = chrono::high_resolution_clock::now();

    imwrite("result_first_" + compensator_type + ".png", result); //保存第一组图像融合结果
    imwrite("result_mask_first_" + compensator_type + ".png", result_mask); //保存第一组图像融合掩码
    
    // 裁剪图像result
    int top_left_y = std::max(
    std::abs(std::abs(corners.front().y) - std::abs(four_corners_warp.front()[0].y)),//要么左边
    std::abs(std::abs(corners.back().y) - std::abs(four_corners_warp.back()[1].y))//要么右边
    );
    int weidth = result.cols;
    int height = min(four_corners_warp[0][3].y-four_corners_warp[0][0].y,four_corners_warp[2][2].y-four_corners_warp[2][1].y);
    Point top_left(0, top_left_y); // 左上角点
    Point bottom_right(top_left.x + weidth, top_left.y + height); // 右下角点

    
    cv::Rect roi(top_left ,bottom_right);
    cv::Mat croppedImage = result(roi);
    imwrite("result_first_roi_" + compensator_type + ".png", croppedImage); //保存裁剪后的图像
    // cout << "裁剪后的图像尺寸: " << croppedImage.size() << endl;
    // cout << "左上: " << top_left << ", 右下: " << bottom_right <<"\n"<< endl;

    auto end_subsequent_group_3 = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_subsequent_group_3 = end_subsequent_group_3 - start_subsequent_group_3;
    cout << "Processing in 保存和剪裁图像 took: " << duration_subsequent_group_3.count() << " seconds." << "/n" << endl;
// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
    TransformationData data;
    data.cameras = cameras;
    data.warper = warper;
    data.compensator = compensator;
    data.seam_finder = seam_finder;
    data.corners = corners;
    data.sizes = sizes;
    data.masks = masks_warp;
    data.blender = blender;
    data.roi = roi; 
    return data;
}

// 函数用于使用保存的变换数据处理后续组图片
void processSubsequentGroup(const vector<String>& image_names, const TransformationData& data) {
    int num_images = image_names.size(); //图片数量
    vector<Mat> images(num_images); //存储所有图像

    for (int i = 0; i < num_images; ++i)
    {
        images[i] = cv::imread(samples::findFile(image_names[i]));//读取每一张图片
        // 用金字塔方式读取图像
        images[i] = scaleImage(images[i], scale); // 缩放图像
        // cout << "image #" << i + 1 << "尺寸为:
    }


    //图像变换
    auto start_subsequent_group_0 = chrono::high_resolution_clock::now();

    vector<Mat> images_warp(num_images); //图像扭曲
    for (int i = 0; i < num_images; ++i)
    {
        Mat K;
        data.cameras[i].K().convertTo(K, CV_32F);
        data.warper->warp(images[i], K, data.cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warp[i]);  //扭曲图像images->images_warp
    }
    auto end_subsequent_group_0 = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_subsequent_group_0 = end_subsequent_group_0 - start_subsequent_group_0;
    cout << "\nProcessing in 图像变换 took: " << duration_subsequent_group_0.count() << " seconds." << endl;

    // 修正曝光补偿，三张图的拼接耗时多了0.03s
    std::string compensator_type = getCompensatorTypeName(data.compensator);
    auto start_subsequent_group_1 = chrono::high_resolution_clock::now();
    for (int i = 0; i < num_images; ++i) {
        data.compensator->apply(i, data.corners[i], images_warp[i], data.masks[i]);
    }
    auto end_subsequent_group_1 = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_subsequent_group_1 = end_subsequent_group_1 - start_subsequent_group_1;
    cout << "Processing in 修正曝光补偿 took: " << duration_subsequent_group_1.count() << " seconds." << endl;
    
    //图像融合
    auto start_subsequent_group_2 = chrono::high_resolution_clock::now();

    Ptr<Blender> blender; //定义图像融合器

// GPU版本：
    blender = Blender::createDefault(Blender::NO, true); //多频带融合Blender::MULTI_BAND 或者羽化Blender::FEATHER, 或者Blender::NO

// CPU版本：
    // blender = Blender::createDefault(Blender::NO, false); //简单融合方法

// 继承版本
    // blender = data.blender; //使用传入的blender
    blender->prepare(data.corners, data.sizes);  //生成全景图像区域

    for (int i = 0; i < num_images; ++i)
    {
        images_warp[i].convertTo(images_warp[i], CV_16S); 
                    auto start_subsequent_group_feed = chrono::high_resolution_clock::now();
        blender->feed(images_warp[i], data.masks[i], data.corners[i]);  //处理图像 初始化数据
                    auto end_subsequent_group_feed = chrono::high_resolution_clock::now();
                    chrono::duration<double> duration_subsequent_group_feed = end_subsequent_group_feed - start_subsequent_group_feed;
                    cout << "Processing in 修正图像融合-in 2 feed took: " << duration_subsequent_group_feed.count() << " seconds." << endl;
    } 







    auto start_subsequent_group_blend = chrono::high_resolution_clock::now();

    Mat result, result_mask;
    blender->blend(result, result_mask);  //blend( InputOutputArray dst, InputOutputArray dst_mask  )混合并返回最后的pano。
    cv::Mat croppedImage = result(data.roi);

    auto end_subsequent_group_blend = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_subsequent_group_blend = end_subsequent_group_blend - start_subsequent_group_blend;
    cout << "Processing in 修正图像融合-blend took: " << duration_subsequent_group_blend.count() << " seconds." << endl;



    auto end_subsequent_group_2 = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_subsequent_group_2 = end_subsequent_group_2 - start_subsequent_group_2;
    cout << "Processing in 修正图像融合 total took: " << duration_subsequent_group_2.count() << " seconds." << endl;


// 保存结果
    // auto start_subsequent_group_3 = chrono::high_resolution_clock::now();

    imwrite("result_subsequent_" + compensator_type + ".png", result); 
    imwrite("result_subsequent_roi_" + compensator_type + ".png", croppedImage);
    
    // auto end_subsequent_group_3 = chrono::high_resolution_clock::now();
    // chrono::duration<double> duration_subsequent_group_3 = end_subsequent_group_3 - start_subsequent_group_3;
    // cout << "Processing in 保存图像 took: " << duration_subsequent_group_3.count() << " seconds." << endl;

}


int main()
{
    // 第一组图片路径
    vector<String> first_group_image_names;
    String first_group_filepath = "/home/tl/RV/src/image_stitching_pkg/images/first_group/*.jpg";
    glob(first_group_filepath, first_group_image_names, false);

    // 记录处理第一组图片的开始时间
    auto start_first_group = chrono::high_resolution_clock::now();
    // 处理第一组图片并保存变换数据
    TransformationData data = processFirstGroup(first_group_image_names);
    // 记录处理第一组图片的结束时间
    auto end_first_group = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_first_group = end_first_group - start_first_group;
    cout << "Processing first group took: " << duration_first_group.count() << " seconds." << endl;

    // 后续组图片路径
    vector<String> subsequent_group_image_names;
    String subsequent_group_filepath = "/home/tl/RV/src/image_stitching_pkg/images/subsequent_group/*.jpg";
    glob(subsequent_group_filepath, subsequent_group_image_names, false);

    // 记录处理后续组图片的开始时间
    auto start_subsequent_group = chrono::high_resolution_clock::now();
    // 使用保存的变换数据处理后续组图片
    processSubsequentGroup(subsequent_group_image_names, data);
    // 记录处理后续组图片的结束时间
    auto end_subsequent_group = chrono::high_resolution_clock::now();
    chrono::duration<double> duration_subsequent_group = end_subsequent_group - start_subsequent_group;
    cout << "cuda Processing subsequent group took: " << duration_subsequent_group.count() << " seconds." << endl;

    return 0;
}