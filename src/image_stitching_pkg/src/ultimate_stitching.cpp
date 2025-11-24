// https://blog.csdn.net/cxyhjl/article/details/137738548
// 开源的图像拼接程序，基于OpenCV库实现，支持多种图像拼接算法和配置选项，保存于此处以备查阅。
#include <iostream> // 引入输入输出流库
#include <fstream> // 引入文件流库，用于文件输入输出
#include <string> // 引入字符串库
#include "opencv2/opencv_modules.hpp" // 引入OpenCV模块
#include <opencv2/core/utility.hpp> // 引入OpenCV核心和辅助功能
#include "opencv2/imgcodecs.hpp" // 引入图像编解码模块
#include "opencv2/highgui.hpp" // 引入高级GUI模块，用于展示图片等
#include "opencv2/stitching/detail/autocalib.hpp" // 引入图像缝合细节中的自动校准模块
#include "opencv2/stitching/detail/blenders.hpp" // 引入图像缝合细节中的图像融合模块
#include "opencv2/stitching/detail/timelapsers.hpp" // 引入时间错位模块
#include "opencv2/stitching/detail/camera.hpp" // 引入图像缝合细节中的相机模块
#include "opencv2/stitching/detail/exposure_compensate.hpp" // 引入曝光补偿模块
#include "opencv2/stitching/detail/matchers.hpp" // 引入特征匹配模块
#include "opencv2/stitching/detail/motion_estimators.hpp" // 引入运动估计模块
#include "opencv2/stitching/detail/seam_finders.hpp" // 引入接缝查找模块
#include "opencv2/stitching/detail/warpers.hpp" // 引入图像扭曲模块
#include "opencv2/stitching/warpers.hpp" // 引入图像扭曲模块


#ifdef HAVE_OPENCV_XFEATURES2D // 条件编译，检查是否包含特征检测模块
#include "opencv2/xfeatures2d.hpp" // 引入扩展特征检测模块
#include "opencv2/xfeatures2d/nonfree.hpp" // 引入非自由特征检测模块（如SIFT、SURF等）
#endif

#ifdef HAVE_OPENCV_CUDALEGACY
#include <opencv2/cudafeatures2d.hpp> // 引入CUDA特征检测模块
#include <opencv2/cudawarping.hpp> // 引入CUDA图像扭
#include <opencv2/cudaarithm.hpp> // 引入CUDA算术运算模块
#endif

#define ENABLE_LOG 1 // 定义一个用于控制是否打印日志的宏
#define LOG(msg) std::cout << msg // 定义一个宏，用于输出日志信息到控制台
#define LOGLN(msg) std::cout << msg << std::endl // 定义一个宏，用于输出日志信息到控制台，并换行


using namespace std; // 使用标准命名空间
using namespace cv; // 使用OpenCV命名空间
using namespace cv::detail; // 使用OpenCV细节命名空间


// 函数printUsage用于打印使用说明
// 打印使用说明
static void printUsage(char** argv)
{
    cout <<
        "旋转模型图像拼接器。\n\n"
         << argv[0] << " img1 img2 [...imgN] [flags]\n\n"
        "Flags:\n"
        "  --preview\n"
        "      以预览模式运行拼接。比普通模式更快，但输出图像分辨率较低。\n"
        "  --try_cuda (yes|no)\n"
        "      尝试使用CUDA。默认值为'no'。所有默认值均为CPU模式。\n"
        "\n运动估计标志:\n"
        "  --work_megapix <float>\n"
        "      图像配准步骤的分辨率。默认值为0.6百万像素。\n"
        "  --features (surf|orb|sift|akaze)\n"
        "      用于图像匹配的特征类型。\n"
        "      如果可用，默认为surf，否则为orb。\n"
        "  --matcher (homography|affine)\n"
        "      用于两两图像匹配的匹配器。\n"
        "  --estimator (homography|affine)\n"
        "      用于变换估计的估计器类型。\n"
        "  --match_conf <float>\n"
        "      特征匹配步骤的置信度。对于surf，默认值为0.65，对于orb，默认值为0.3。\n"
        "  --conf_thresh <float>\n"
        "      两个图像为同一全景图的置信度阈值。\n"
        "      默认值为1.0。\n"
        "  --ba (no|reproj|ray|affine)\n"
        "      捆绑调整成本函数。默认为ray。\n"
        "  --ba_refine_mask (mask)\n"
        "      为捆绑调整设置细化掩模。格式为'x_xxx'，其中'x'表示细化相应参数，'_'表示不细化，格式如下：\n"
        "      <fx><skew><ppx><aspect><ppy>。默认掩模为'xxxxx'。如果捆绑调整不支持所选参数的估计，则相应标志将被忽略。\n"
        "  --wave_correct (no|horiz|vert)\n"
        "      执行波效果校正。默认为'horiz'。\n"
        "  --save_graph <file_name>\n"
        "      将匹配图保存为DOT语言表示的<file_name>文件。\n"
        "      标签说明：Nm是匹配数，Ni是内点数，C是置信度。\n"
        "\n合成标志:\n"
        "  --warp (affine|plane|cylindrical|spherical|fisheye|stereographic|compressedPlaneA2B1|compressedPlaneA1.5B1|compressedPlanePortraitA2B1|compressedPlanePortraitA1.5B1|paniniA2B1|paniniA1.5B1|paniniPortraitA2B1|paniniPortraitA1.5B1|mercator|transverseMercator)\n"
        "      变换表面类型。默认为'spherical'。\n"
        "  --seam_megapix <float>\n"
        "      接缝估计步骤的分辨率。默认为0.1百万像素。\n"
        "  --seam (no|voronoi|gc_color|gc_colorgrad)\n"
        "      接缝估计方法。默认为'gc_color'。\n"
        "  --compose_megapix <float>\n"
        "      合成步骤的分辨率。使用-1表示原始分辨率。\n"
        "      默认值为-1。\n"
        "  --expos_comp (no|gain|gain_blocks|channels|channels_blocks)\n"
        "      曝光补偿方法。默认为'gain_blocks'。\n"
        "  --expos_comp_nr_feeds <int>\n"
        "      曝光补偿反馈次数。默认为1。\n"
        "  --expos_comp_nr_filtering <int>\n"
        "      曝光补偿增益的过滤迭代次数。仅在使用块曝光补偿方法时使用。\n"
        "      默认为2。\n"
        "  --expos_comp_block_size <int>\n"
        "      曝光补偿器使用的块大小（以像素为单位）。\n"
        "      仅在使用块曝光补偿方法时使用。\n"
        "      默认为32。\n"
        "  --blend (no|feather|multiband)\n"
        "      混合方法。默认为'multiband'。\n"
        "  --blend_strength <float>\n"
        "      混合强度范围为[0,100]。默认值为5。\n"
        "  --output <result_img>\n"
        "      输出图像的文件名。默认为'result.jpg'。\n"
        "  --timelapse (as_is|crop) \n"
        "      将扭曲后的图像单独输出为时间-lapse电影的帧，输入文件名前加上'fixed_'。\n"
        "  --rangewidth <int>\n"
        "      使用range_width限制要匹配的图像数量。\n";
}


// 默认的命令行参数
vector<String> img_names; // 存储图像文件名的向量
bool preview = false; // 是否预览模式的标志，默认关闭
bool try_cuda = false; // 是否尝试使用CUDA加速的标志，默认关闭
double work_megapix = 0.6; // 用于图像注册步骤的分辨率，默认为0.6百万像素
double seam_megapix = 0.1; // 用于估算缝合线的图像分辨率，默认为0.1百万像素
double compose_megapix = -1; // 图像融合步骤的分辨率，默认为原始分辨率
float conf_thresh = 1.f; // 配准图像的置信度阈值，默认为1.0
#ifdef HAVE_OPENCV_XFEATURES2D
string features_type = "surf"; // 默认特征类型为SURF，如果库中包含xfeatures2d特征
float match_conf = 0.65f; // 特征匹配的置信度，默认为0.65
#else
string features_type = "orb"; // 不包含xfeatures2d时，默认特征类型为ORB
float match_conf = 0.3f; // ORB特征的匹配置信度，默认为0.3
#endif
string matcher_type = "homography"; // 特征匹配器类型，默认为基于单应性矩阵的匹配器
string estimator_type = "homography"; // 用于变换估计的类型，默认也是单应性
string ba_cost_func = "ray"; // 光束平差(Bundle Adjustment)的代价函数，默认为"ray"
string ba_refine_mask = "xxxxx"; // 光束平差细化掩模，默认对所有参数进行细化
bool do_wave_correct = true; // 是否进行波形校正，默认开启
WaveCorrectKind wave_correct = detail::WAVE_CORRECT_HORIZ; // 波形校正的方向，默认水平方向
bool save_graph = false; // 是否保存图像匹配状态图，默认关闭
std::string save_graph_to; // 保存图像匹配状态图的文件名称
string warp_type = "spherical"; // 进行图像变形的类型，默认为球形变形
int expos_comp_type = ExposureCompensator::GAIN_BLOCKS; // 曝光补偿的类型，默认为gain_blocks
int expos_comp_nr_feeds = 1; // 曝光补偿阶段的输入数，默认为1
int expos_comp_nr_filtering = 2; // 曝光补偿增益的滤波迭代次数，默认为2次
int expos_comp_block_size = 32; // 曝光补偿时块的大小，默认为32像素
string seam_find_type = "gc_color"; // 缝合线发现的方法，默认为基于颜色的图割方法
int blend_type = Blender::MULTI_BAND; // 图像融合的方法，默认为多频段融合
int timelapse_type = Timelapser::AS_IS; // 创建时光延展图像的类型，默认为AS_IS（即原样）
float blend_strength = 5; // 融合的强度，默认值为5
string result_name = "result.jpg"; // 最终生成的全景图像的文件名，默认为result.jpg
bool timelapse = false; // 是否创建时光延展图像，默认为否
int range_width = -1; // 限制匹配图像数量的范围宽度，默认不限制


// 解析命令行参数的静态函数定义
static int parseCmdArgs(int argc, char** argv)
{
    // 如果只有一个参数（程序名），打印使用说明并返回-1
    if (argc == 1)
    {
        printUsage(argv);
        return -1;
    }
    // 遍历命令行提供的所有参数
    for (int i = 1; i < argc; ++i)
    {
        // 如果命令行参数为"--help"或"/?"，打印使用说明并返回-1
        if (string(argv[i]) == "--help" || string(argv[i]) == "/?")
        {
            printUsage(argv);
            return -1;
        }
        // 如果命令行参数为"--preview"，设置预览模式标志为真
        else if (string(argv[i]) == "--preview")
        {
            preview = true;
        }
        // 如果命令行参数为"--try_cuda"，根据后续参数选择是否尝试CUDA加速
        else if (string(argv[i]) == "--try_cuda")
        {
            // 根据后续参数的值 ("no" 或 "yes") 设定try_cuda变量
            if (string(argv[i + 1]) == "no")
                try_cuda = false;
            else if (string(argv[i + 1]) == "yes")
                try_cuda = true;
            else
            {
                cout << "Bad --try_cuda flag value\n"; // 错误的命令行参数值
                return -1;
            }
            i++;
        }
        // 如果命令行参数为"--work_megapix"，设置图像注册步骤的分辨率
        else if (string(argv[i]) == "--work_megapix")
        {
            work_megapix = atof(argv[i + 1]);
            i++;
        }
        // 如果命令行参数为"--seam_megapix"，设置缝合线估算步骤的分辨率
        else if (string(argv[i]) == "--seam_megapix")
        {
            seam_megapix = atof(argv[i + 1]);
            i++;
        }
        // 如果命令行参数为"--compose_megapix"，设置图像融合步骤的分辨率
        else if (string(argv[i]) == "--compose_megapix")
        {
            compose_megapix = atof(argv[i + 1]);
            i++;
        }
        // 如果命令行参数为"--result"，设置最终结果图像的文件名
        else if (string(argv[i]) == "--result")
        {
            result_name = argv[i + 1];
            i++;
        }
        // 如果命令行参数为"--features"，设置特征类型，并根据特征类型可能调整特征匹配置信度
        else if (string(argv[i]) == "--features")
        {
            features_type = argv[i + 1];
            if (string(features_type) == "orb")
                match_conf = 0.3f; // 如果是ORB特征，则将特征匹配置信度调整为0.3
            i++;
        }
        // 如果命令行参数为"--matcher"，设置特征匹配器的类型
        else if (string(argv[i]) == "--matcher")
        {
            if (string(argv[i + 1]) == "homography" || string(argv[i + 1]) == "affine")
                matcher_type = argv[i + 1];
            else
            {
                cout << "Bad --matcher flag value\n"; // 错误的命令行参数值
                return -1;
            }
            i++;
        }
        // 设置变换估计器类型，例如单应性或仿射
        else if (string(argv[i]) == "--estimator")
        {
            if (string(argv[i + 1]) == "homography" || string(argv[i + 1]) == "affine")
                estimator_type = argv[i + 1];
            else
            {
                cout << "Bad --estimator flag value\n";
                return -1;
            }
            i++; // 增加索引来跳过已经处理的参数值
        }
        // 允许用户设置特征匹配置信度
        else if (string(argv[i]) == "--match_conf")
        {
            match_conf = static_cast<float>(atof(argv[i + 1]));
            i++;
        }
        // 设置配准图像的置信度阈值
        else if (string(argv[i]) == "--conf_thresh")
        {
            conf_thresh = static_cast<float>(atof(argv[i + 1]));
            i++;
        }
        // 用户可以设定光束平差的代价函数类型
        else if (string(argv[i]) == "--ba")
        {
            ba_cost_func = argv[i + 1];
            i++;
        }
        // 设置光束平差细化掩码，用于指定哪些相机参数会被进行细化
        else if (string(argv[i]) == "--ba_refine_mask")
        {
            ba_refine_mask = argv[i + 1];
            if (ba_refine_mask.size() != 5)
            {
                cout << "Incorrect refinement mask length.\n";
                return -1;
            }
            i++;
        }
        // 是否开启波形校正以及校正的方向
        else if (string(argv[i]) == "--wave_correct")
        {
            if (string(argv[i + 1]) == "no")
                do_wave_correct = false;
            else if (string(argv[i + 1]) == "horiz")
            {
                do_wave_correct = true;
                wave_correct = detail::WAVE_CORRECT_HORIZ;
            }
            else if (string(argv[i + 1]) == "vert")
            {
                do_wave_correct = true;
                wave_correct = detail::WAVE_CORRECT_VERT;
            }
            else
            {
                cout << "Bad --wave_correct flag value\n";
                return -1;
            }
            i++;
        }
        // 是否保存图像匹配状态图到指定文件
        else if (string(argv[i]) == "--save_graph")
        {
            save_graph = true;
            save_graph_to = argv[i + 1];
            i++;
        }
        // 允许用户设置图像变形的类型
        else if (string(argv[i]) == "--warp")
        {
            warp_type = string(argv[i + 1]);
            i++;
        }
        // 设置曝光补偿的类型
        else if (string(argv[i]) == "--expos_comp")
        {
            if (string(argv[i + 1]) == "no")
                expos_comp_type = ExposureCompensator::NO;
            else if (string(argv[i + 1]) == "gain")
                expos_comp_type = ExposureCompensator::GAIN;
            else if (string(argv[i + 1]) == "gain_blocks")
                expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
            else if (string(argv[i + 1]) == "channels")
                expos_comp_type = ExposureCompensator::CHANNELS;
            else if (string(argv[i + 1]) == "channels_blocks")
                expos_comp_type = ExposureCompensator::CHANNELS_BLOCKS;
            else
            {
                cout << "Bad exposure compensation method\n";
                return -1;
            }
            i++;
        }
         // 参数：设置曝光补偿时所用的feeds数目
        else if (string(argv[i]) == "--expos_comp_nr_feeds")
        {
            expos_comp_nr_feeds = atoi(argv[i + 1]);
            i++; // 跳过下一步读取的参数值
        }
        // 参数：设置曝光补偿时过滤使用的迭代次数
        else if (string(argv[i]) == "--expos_comp_nr_filtering")
        {
            expos_comp_nr_filtering = atoi(argv[i + 1]);
            i++;
        }
        // 参数：设置曝光补偿区块的大小
        else if (string(argv[i]) == "--expos_comp_block_size")
        {
            expos_comp_block_size = atoi(argv[i + 1]);
            i++;
        }
        // 参数：设置接缝发现的方法
        else if (string(argv[i]) == "--seam")
        {
            if (string(argv[i + 1]) == "no" ||
                string(argv[i + 1]) == "voronoi" ||
                string(argv[i + 1]) == "gc_color" ||
                string(argv[i + 1]) == "gc_colorgrad" ||
                string(argv[i + 1]) == "dp_color" ||
                string(argv[i + 1]) == "dp_colorgrad")
                seam_find_type = argv[i + 1]; // 根据用户指定的类型来设置接缝发现策略
            else
            {
                cout << "Bad seam finding method\n"; // 输入了错误的接缝发现方法
                return -1;
            }
            i++;
        }
        // 参数：设置图像融合的方法
        else if (string(argv[i]) == "--blend")
        {
            if (string(argv[i + 1]) == "no")
                blend_type = Blender::NO; // 不进行融合
            else if (string(argv[i + 1]) == "feather")
                blend_type = Blender::FEATHER; // 羽化融合
            else if (string(argv[i + 1]) == "multiband")
                blend_type = Blender::MULTI_BAND; // 多带融合
            else
            {
                cout << "Bad blending method\n"; // 输入了错误的融合方法
                return -1;
            }
            i++;
        }
        // 参数：设置是否开启延时摄影模式以及其类型
        else if (string(argv[i]) == "--timelapse")
        {
            timelapse = true; // 开启延时摄影模式


            if (string(argv[i + 1]) == "as_is")
                timelapse_type = Timelapser::AS_IS; // 按原样
            else if (string(argv[i + 1]) == "crop")
                timelapse_type = Timelapser::CROP; // 裁剪对齐
            else
            {
                cout << "Bad timelapse method\n"; // 输入了错误的延时摄影方法
                return -1;
            }
            i++;
        }
        // 参数：设置运动范围宽度
        else if (string(argv[i]) == "--rangewidth")
        {
            range_width = atoi(argv[i + 1]);
            i++;
        }
        // 参数：设置融合强度
        else if (string(argv[i]) == "--blend_strength")
        {
            blend_strength = static_cast<float>(atof(argv[i + 1]));
            i++;
        }
        // 参数：指定输出文件的名称
        else if (string(argv[i]) == "--output")
        {
            result_name = argv[i + 1];
            i++;
        }
        // 如果参数不是以"--"开头的标志，则被当作输入图片的文件名
        else
            img_names.push_back(argv[i]);
    }
    // 如果在预览模式下，设置合成的图像分辨率为0.6
    if (preview)
    {
        compose_megapix = 0.6;
    }
    // 返回0表示参数解析成功
    return 0;
}


// 主函数定义
int main(int argc, char* argv[])
{
    // 开启日志
#if ENABLE_LOG
    int64 app_start_time = getTickCount();
#endif


    // 如果需要，设置异常情况下暂停（调试用）
#if 0
    cv::setBreakOnError(true);
#endif


    // 解析命令行参数
    int retval = parseCmdArgs(argc, argv);
    if (retval)
        return retval;  // 如果解析失败则退出


    // 检查是否有足够的图像进行拼接
    int num_images = static_cast<int>(img_names.size());
    if (num_images < 2)
    {
        LOGLN("Need more images");
        return -1;  // 需要至少两张图像
    }


    // 初始化各种尺度变量，并设置标志
    double work_scale = 1, seam_scale = 1, compose_scale = 1;
    bool is_work_scale_set = false, is_seam_scale_set = false, is_compose_scale_set = false;


    LOGLN("Finding features...");
#if ENABLE_LOG
    int64 t = getTickCount();  // 开始记录查找特征的时间
#endif


    // 根据用户选择的特征类型创建特征检测器
    Ptr<Feature2D> finder;
    if (features_type == "orb")
    {
        finder = ORB::create();
    }
    else if (features_type == "akaze")
    {
        finder = AKAZE::create();
    }
    // 如果编译时包含了xfeatures2d模块，则添加SURF作为可能的特征
#ifdef HAVE_OPENCV_XFEATURES2D
    else if (features_type == "surf")
    {
        finder = xfeatures2d::SURF::create();
    }
#endif
    else if (features_type == "sift")
    {
        finder = SIFT::create();
    }
    else
    {
        cout << "Unknown 2D features type: '" << features_type << "'.\n";
        return -1;  // 如果输入的特征类型未知则退出
    }


    // 初始化用于图像处理的变量
    Mat full_img, img;
    vector<ImageFeatures> features(num_images);
    vector<Mat> images(num_images);
    vector<Size> full_img_sizes(num_images);
    double seam_work_aspect = 1;


    // 开始读取和处理所有图像
    for (int i = 0; i < num_images; ++i)
    {
        full_img = imread(samples::findFile(img_names[i]));
        full_img_sizes[i] = full_img.size();


        // 如果图像读取失败，则退出
        if (full_img.empty())
        {
            LOGLN("Can't open image " << img_names[i]);
            return -1;
        }
        // 确定工作图像的尺度
        if (work_megapix < 0)
        {
            img = full_img;
            work_scale = 1;
            is_work_scale_set = true;
        }
        else
        {
            // 如果未设置工作尺度，则根据megapixels计算尺度
            if (!is_work_scale_set)
            {
                work_scale = min(1.0, sqrt(work_megapix * 1e6 / full_img.size().area()));
                is_work_scale_set = true;
            }
            // 调整图像尺寸
            resize(full_img, img, Size(), work_scale, work_scale, INTER_LINEAR_EXACT);
        }
        // 确定接缝图像的尺度
        if (!is_seam_scale_set)
        {
            seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / full_img.size().area()));
            seam_work_aspect = seam_scale / work_scale;
            is_seam_scale_set = true;
        }


        // 查找图像特征
        computeImageFeatures(finder, img, features[i]);
        features[i].img_idx = i;  // 记录图像索引
        LOGLN("Features in image #" << i+1 << ": " << features[i].keypoints.size());


        // 将图像调整到接缝尺寸并存储
        resize(full_img, img, Size(), seam_scale, seam_scale, INTER_LINEAR_EXACT);
        images[i] = img.clone();
    }


    // 释放不再使用的图像内存
    full_img.release();
    img.release();


    // 打印查找特征的时间
    LOGLN("Finding features, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");


    // 配对匹配
    LOG("Pairwise matching");
#if ENABLE_LOG
    t = getTickCount();  // 开始记录配对匹配的时间
#endif
    vector<MatchesInfo> pairwise_matches;
    Ptr<FeaturesMatcher> matcher;
    // 选择匹配器类型
    if (matcher_type == "affine")
        matcher = makePtr<AffineBestOf2NearestMatcher>(false, try_cuda, match_conf);
    else if (range_width==-1)
        matcher = makePtr<BestOf2NearestMatcher>(try_cuda, match_conf);
    else
        matcher = makePtr<BestOf2NearestRangeMatcher>(range_width, try_cuda, match_conf);


    // 进行配对匹配
    (*matcher)(features, pairwise_matches);
    // 回收匹配器的垃圾空间
    matcher->collectGarbage();


    // 打印配对匹配的时间
    LOGLN("Pairwise matching, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");


    // 检查是否需要保存图像匹配关系图
    if (save_graph)
    {
        LOGLN("Saving matches graph..."); // 记录日志
        ofstream f(save_graph_to.c_str()); // 打开文件流
        f << matchesGraphAsString(img_names, pairwise_matches, conf_thresh); // 将匹配关系图转换为字符串并保存
    }


    // 保留确定属于同一全景的图像集
    vector<int> indices = leaveBiggestComponent(features, pairwise_matches, conf_thresh); // 留下最大组件的索引
    vector<Mat> img_subset; // 新图像集合
    vector<String> img_names_subset; // 新图像名称集合
    vector<Size> full_img_sizes_subset; // 新图像尺寸集合
    for (size_t i = 0; i < indices.size(); ++i)
    {
        img_names_subset.push_back(img_names[indices[i]]); // 添加图像名称
        img_subset.push_back(images[indices[i]]); // 添加图像
        full_img_sizes_subset.push_back(full_img_sizes[indices[i]]); // 添加图像尺寸
    }


    images = img_subset; // 更新图像集合
    img_names = img_names_subset; // 更新图像名称集合
    full_img_sizes = full_img_sizes_subset; // 更新图像尺寸集合


    // 再次检查是否有足够的图像
    num_images = static_cast<int>(img_names.size()); // 更新图像数量
    if (num_images < 2)
    {
        LOGLN("Need more images"); // 记录日志
        return -1; // 若图像不够，则退出
    }


    // 创建估算器来估计相机参数
    Ptr<Estimator> estimator;
    if (estimator_type == "affine")
        estimator = makePtr<AffineBasedEstimator>(); // 仿射估算器
    else
        estimator = makePtr<HomographyBasedEstimator>(); // 单应性估算器


    vector<CameraParams> cameras; // 相机参数集合
    if (!(*estimator)(features, pairwise_matches, cameras)) // 如果估计失败
    {
        cout << "Homography estimation failed.\n"; // 打印失败信息
        return -1; // 失败则退出
    }


    // 将内定参数更正为适合的格式并记录
    for (size_t i = 0; i < cameras.size(); ++i)
    {
        Mat R;
        cameras[i].R.convertTo(R, CV_32F); // 将旋转矩阵转换为浮点数格式
        cameras[i].R = R; // 更新旋转矩阵
        LOGLN("Initial camera intrinsics #" << indices[i]+1 << ":\nK:\n" << cameras[i].K() << "\nR:\n" << cameras[i].R); // 记录相机内参
    }


    // 创建捆绑调整器来微调相机参数
    Ptr<detail::BundleAdjusterBase> adjuster;
    if (ba_cost_func == "reproj") adjuster = makePtr<detail::BundleAdjusterReproj>(); // 重投影调整器
    else if (ba_cost_func == "ray") adjuster = makePtr<detail::BundleAdjusterRay>(); // 射线调整器
    else if (ba_cost_func == "affine") adjuster = makePtr<detail::BundleAdjusterAffinePartial>(); // 仿射调整器
    else if (ba_cost_func == "no") adjuster = makePtr<NoBundleAdjuster>(); // 不使用捆绑调整
    else
    {
        cout << "Unknown bundle adjustment cost function: '" << ba_cost_func << "'.\n"; // 未知的调整器
        return -1; // 未知的调整器则退出
    }
    adjuster->setConfThresh(conf_thresh); // 设置置信度阈值
    Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U); // 创造细化掩码
    if (ba_refine_mask[0] == 'x') refine_mask(0,0) = 1; // 设置细化掩码
    if (ba_refine_mask[1] == 'x') refine_mask(0,1) = 1; // 设置细化掩码
    if (ba_refine_mask[2] == 'x') refine_mask(0,2) = 1; // 设置细化掩码
    if (ba_refine_mask[3] == 'x') refine_mask(1,1) = 1; // 设置细化掩码
    if (ba_refine_mask[4] == 'x') refine_mask(1,2) = 1; // 设置细化掩码
    adjuster->setRefinementMask(refine_mask); // 应用细化掩码
    if (!(*adjuster)(features, pairwise_matches, cameras)) // 如果调整失败
    {
        cout << "Camera parameters adjusting failed.\n"; // 打印失败信息
        return -1; // 调整失败则退出
    }


    // 计算所有相机的中值焦距
    vector<double> focals;
    for (size_t i = 0; i < cameras.size(); ++i)
    {
        LOGLN("Camera #" << indices[i]+1 << ":\nK:\n" << cameras[i].K() << "\nR:\n" << cameras[i].R); // 记录相机内参
        focals.push_back(cameras[i].focal); // 记录焦距
    }


    sort(focals.begin(), focals.end()); // 对焦距进行排序
    float warped_image_scale; // 变形后的图像尺度
    if (focals.size() % 2 == 1)
        warped_image_scale = static_cast<float>(focals[focals.size() / 2]); // 选择中值焦距
    else
        warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f; // 计算中值焦距


    // 波形矫正
    if (do_wave_correct)
    {
        vector<Mat> rmats; // 旋转矩阵集合
        for (size_t i = 0; i < cameras.size(); ++i)
            rmats.push_back(cameras[i].R.clone()); // 复制所有的旋转矩阵
        
        waveCorrect(rmats, wave_correct); // 波形矫正
        
        for (size_t i = 0; i < cameras.size(); ++i)
            cameras[i].R = rmats[i]; // 更新矫正后的旋转矩阵
    }


    LOGLN("Warping images (auxiliary)... "); // 记录日志
#if ENABLE_LOG
    t = getTickCount(); // 记录时间
#endif


    vector<Point> corners(num_images); // 图像变形后的角点
    vector<UMat> masks_warped(num_images); // 变形后图像的蒙版
    vector<UMat> images_warped(num_images); // 变形后的图像
    vector<Size> sizes(num_images); // 变形后的尺寸
    vector<UMat> masks(num_images); // 初始化蒙版


    // 准备图像蒙版
    for (int i = 0; i < num_images; ++i)
    {
        masks[i].create(images[i].size(), CV_8U);  // 创建大小相同的空蒙版
        masks[i].setTo(Scalar::all(255));  // 将蒙版置为白色，意味着图像的所有区域都用于拼接
    }


    // 对图像和它们的掩模进行变形处理
    Ptr<WarperCreator> warper_creator;
    // 如果定义了CUDA变形，并且CUDA可用，则根据变形类型创建对应的GPU变形器
#ifdef HAVE_OPENCV_CUDAWARPING
    if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
    {
        if (warp_type == "plane")
            warper_creator = makePtr<cv::PlaneWarperGpu>();
        else if (warp_type == "cylindrical")
            warper_creator = makePtr<cv::CylindricalWarperGpu>();
        else if (warp_type == "spherical")
            warper_creator = makePtr<cv::SphericalWarperGpu>();
    }
    else
#endif
    {
        // 根据变形类型创建相应的CPU变形器
        if (warp_type == "plane")
            warper_creator = makePtr<cv::PlaneWarper>(); // 平面变形器
        else if (warp_type == "affine")
            warper_creator = makePtr<cv::AffineWarper>(); // 仿射变形器
        else if (warp_type == "cylindrical")
            warper_creator = makePtr<cv::CylindricalWarper>(); // 圆柱变形器
        else if (warp_type == "spherical")
            warper_creator = makePtr<cv::SphericalWarper>(); // 球形变形器
        else if (warp_type == "fisheye")
            warper_creator = makePtr<cv::FisheyeWarper>(); // 鱼眼变形器
        else if (warp_type == "stereographic")
            warper_creator = makePtr<cv::StereographicWarper>(); // 立体图变形器
        else if (warp_type == "compressedPlaneA2B1")
            warper_creator = makePtr<cv::CompressedRectilinearWarper>(2.0f, 1.0f); // 压缩平面变形器 A=2,B=1
        else if (warp_type == "compressedPlaneA1.5B1")
            warper_creator = makePtr<cv::CompressedRectilinearWarper>(1.5f, 1.0f); // 压缩平面变形器 A=1.5,B=1
        else if (warp_type == "compressedPlanePortraitA2B1")
            warper_creator = makePtr<cv::CompressedRectilinearPortraitWarper>(2.0f, 1.0f); // 压缩平面人像变形器 A=2,B=1
        else if (warp_type == "compressedPlanePortraitA1.5B1")
            warper_creator = makePtr<cv::CompressedRectilinearPortraitWarper>(1.5f, 1.0f); // 压缩平面人像变形器 A=1.5,B=1
        else if (warp_type == "paniniA2B1")
            warper_creator = makePtr<cv::PaniniWarper>(2.0f, 1.0f); // Panini变形器 A=2,B=1
        else if (warp_type == "paniniA1.5B1")
            warper_creator = makePtr<cv::PaniniWarper>(1.5f, 1.0f); // Panini变形器 A=1.5,B=1
        else if (warp_type == "paniniPortraitA2B1")
            warper_creator = makePtr<cv::PaniniPortraitWarper>(2.0f, 1.0f); // Panini人像变形器 A=2,B=1
        else if (warp_type == "paniniPortraitA1.5B1")
            warper_creator = makePtr<cv::PaniniPortraitWarper>(1.5f, 1.0f); // Panini人像变形器 A=1.5,B=1
        else if (warp_type == "mercator")
            warper_creator = makePtr<cv::MercatorWarper>(); // Mercator变形器
        else if (warp_type == "transverseMercator")
            warper_creator = makePtr<cv::TransverseMercatorWarper>(); // 横向Mercator变形器
    }


    // 如果创建变形器失败，则输出错误并返回
    if (!warper_creator)
    {
        cout << "Can't create the following warper '" << warp_type << "'\n";
        return 1;
    }


    // 创建变形器，并设置变形尺度
    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));


    for (int i = 0; i < num_images; ++i)
    {
        Mat_<float> K;
        // 获取并调整第i个相机的内部参数以适应变形尺度
        cameras[i].K().convertTo(K, CV_32F);
        float swa = (float)seam_work_aspect;
        K(0,0) *= swa; K(0,2) *= swa;
        K(1,1) *= swa; K(1,2) *= swa;


        // 变形第i张图像及其掩模
        corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
        sizes[i] = images_warped[i].size();
        warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
    }


    // 将变形后图像的数据类型从8位无符号整数转换到32位浮点数
    vector<UMat> images_warped_f(num_images);
    for (int i = 0; i < num_images; ++i)
        images_warped[i].convertTo(images_warped_f[i], CV_32F);


    // 打印变形图像的时间
    LOGLN("Warping images, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");


    // 开始进行曝光补偿处理
    LOGLN("Compensating exposure...");
#if ENABLE_LOG
    t = getTickCount(); // 开始记录曝光补偿的时间
#endif


    // 创建曝光补偿器，并根据类型进行相应的配置
    Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(expos_comp_type);
    // 曝光补偿器的特定配置
    // 根据补偿器类型对其进行动态类型转换并配置参数
    if (dynamic_cast<GainCompensator*>(compensator.get()))
    {
        GainCompensator* gcompensator = dynamic_cast<GainCompensator*>(compensator.get());
        gcompensator->setNrFeeds(expos_comp_nr_feeds); // 设置增益补偿器的饲喂次数
    }


    if (dynamic_cast<ChannelsCompensator*>(compensator.get()))
    {
        ChannelsCompensator* ccompensator = dynamic_cast<ChannelsCompensator*>(compensator.get());
        ccompensator->setNrFeeds(expos_comp_nr_feeds); // 设置通道补偿器的饲喂次数
    }


    if (dynamic_cast<BlocksCompensator*>(compensator.get()))
    {
        BlocksCompensator* bcompensator = dynamic_cast<BlocksCompensator*>(compensator.get());
        bcompensator->setNrFeeds(expos_comp_nr_feeds); // 设置块补偿器的饲喂次数
        bcompensator->setNrGainsFilteringIterations(expos_comp_nr_filtering); // 设置增益过滤迭代次数
        bcompensator->setBlockSize(expos_comp_block_size, expos_comp_block_size); // 设置块大小
    }


    // 使用曝光补偿器处理变形后的图像和掩模
    compensator->feed(corners, images_warped, masks_warped);


    // 打印曝光补偿时间
    LOGLN("Compensating exposure, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");


    // 开始寻找最佳的缝合线
    LOGLN("Finding seams...");
#if ENABLE_LOG
    t = getTickCount(); // 开始记录寻找缝合线的时间
#endif


    // 创建缝合线查找器，根据选择的类型进行实例化
    Ptr<SeamFinder> seam_finder;
    // 如果不存在缝合线查找，将其设置为NoSeamFinder；否则，根据选择的类型进行实例化
    // 创建缝合线查找器，根据选择的类型进行实例化
    if (seam_find_type == "no")
        seam_finder = makePtr<detail::NoSeamFinder>(); // 不使用缝合线查找器
    else if (seam_find_type == "voronoi")
        seam_finder = makePtr<detail::VoronoiSeamFinder>(); // Voronoi缝合线查找器
    else if (seam_find_type == "gc_color")
    {
#ifdef HAVE_OPENCV_CUDALEGACY
        if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
            seam_finder = makePtr<detail::GraphCutSeamFinderGpu>(GraphCutSeamFinderBase::COST_COLOR); // 使用GPU的图割缝合线查找器，颜色成本
        else
#endif
            seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR); // 图割缝合线查找器，颜色成本
    }
    else if (seam_find_type == "gc_colorgrad")
    {
#ifdef HAVE_OPENCV_CUDALEGACY
        if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
            seam_finder = makePtr<detail::GraphCutSeamFinderGpu>(GraphCutSeamFinderBase::COST_COLOR_GRAD); // 使用GPU的图割缝合线查找器，颜色梯度成本
        else
#endif
            seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR_GRAD); // 图割缝合线查找器，颜色梯度成本
    }
    else if (seam_find_type == "dp_color")
        seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR); // 动态规划缝合线查找器，颜色成本
    else if (seam_find_type == "dp_colorgrad")
        seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR_GRAD); // 动态规划缝合线查找器，颜色梯度成本


    // 如果缝合线查找器创建失败，则输出错误并返回
    if (!seam_finder)
    {
        cout << "Can't create the following seam finder '" << seam_find_type << "'\n";
        return 1;
    }
    // 使用缝合线查找器来寻找图像之间的缝合线
    seam_finder->find(images_warped_f, corners, masks_warped);


    // 记录缝合线查找的耗时
    LOGLN("Finding seams, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");


    // 释放不再使用的内存资源
    images.clear();
    images_warped.clear();
    images_warped_f.clear();
    masks.clear();


    // 开始图像合成步骤
    LOGLN("Compositing...");
    // 如果启用了日志，则记录合成开始的时间
#if ENABLE_LOG
    t = getTickCount();
#endif


    // 初始化合成所需的变量
    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    Ptr<Blender> blender;
    Ptr<Timelapser> timelapser;
    // 变量用于计算合成图像的工作尺寸，这里没有用到注释掉的变量
    double compose_work_aspect = 1;


    // 对所有图像进行遍历处理，以实现拼接
    // 遍历所有需要合成的图片
    for (int img_idx = 0; img_idx < num_images; ++img_idx)
    {
        // 记录正在合成的图片编号
        LOGLN("Compositing image #" << indices[img_idx]+1);


        // 读取图片，并根据需要调整尺寸
        full_img = imread(samples::findFile(img_names[img_idx]));
        if (!is_compose_scale_set)
        {
            // 如果还没有设置合成图片的比例，则根据需要进行设置
            if (compose_megapix > 0)
                compose_scale = min(1.0, sqrt(compose_megapix * 1e6 / full_img.size().area()));
            is_compose_scale_set = true;


            // 计算相对尺度，以及必要的缩放比例
            compose_work_aspect = compose_scale / work_scale;


            // 更新变形图像的比例
            warped_image_scale *= static_cast<float>(compose_work_aspect);
            warper = warper_creator->create(warped_image_scale);


            // 更新每张图片的角点和尺寸
            for (int i = 0; i < num_images; ++i)
            {
                // 更新相机内参
                cameras[i].focal *= compose_work_aspect;
                cameras[i].ppx *= compose_work_aspect;
                cameras[i].ppy *= compose_work_aspect;


                // 更新图片角点和尺寸
                Size sz = full_img_sizes[i];
                if (std::abs(compose_scale - 1) > 1e-1)
                {
                    sz.width = cvRound(full_img_sizes[i].width * compose_scale);
                    sz.height = cvRound(full_img_sizes[i].height * compose_scale);
                }


                // 计算变形后的ROI
                Mat K;
                cameras[i].K().convertTo(K, CV_32F);
                Rect roi = warper->warpRoi(sz, K, cameras[i].R);
                corners[i] = roi.tl();
                sizes[i] = roi.size();
            }
        }


        // 根据比例调整图片大小
        if (abs(compose_scale - 1) > 1e-1)
            resize(full_img, img, Size(), compose_scale, compose_scale, INTER_LINEAR_EXACT);
        else
            img = full_img;
        full_img.release();
        Size img_size = img.size();


        // 准备图片变形所需的矩阵
        Mat K;
        cameras[img_idx].K().convertTo(K, CV_32F);


        // 对当前图片进行变形处理
        warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);


        // 创建并对当前图片的掩码进行变形处理
        mask.create(img_size, CV_8U);
        mask.setTo(Scalar::all(255));
        warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);


        // 应用曝光补偿
        compensator->apply(img_idx, corners[img_idx], img_warped, mask_warped);


        // 将变形后的图片转换为有符号的16位整数格式用于合成
        img_warped.convertTo(img_warped_s, CV_16S);
        img_warped.release();
        img.release();
        mask.release();


        // 对蒙版进行膨胀处理，以便用于缝合线的掩饰
        dilate(masks_warped[img_idx], dilated_mask, Mat());
        resize(dilated_mask, seam_mask, mask_warped.size(), 0, 0, INTER_LINEAR_EXACT);
        mask_warped = seam_mask & mask_warped;


        // 如果没在延时摄影模式中并且还未创建blender对象，则创建它
        if (!blender && !timelapse)
        {
            blender = Blender::createDefault(blend_type, try_cuda);
            Size dst_sz = resultRoi(corners, sizes).size();
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            if (blend_width < 1.f)
                blender = Blender::createDefault(Blender::NO, try_cuda);
            else if (blend_type == Blender::MULTI_BAND)
            {
                // 如果是多频段混合则创建并设置多频段blender
                MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
                mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
                LOGLN("Multi-band blender, number of bands: " << mb->numBands());
            }
            else if (blend_type == Blender::FEATHER)
            {
                // 如果是羽化混合类型则创建并设置羽化blender
                FeatherBlender* fb = dynamic_cast<FeatherBlender*>(blender.get());
                fb->setSharpness(1.f/blend_width);
                LOGLN("Feather blender, sharpness: " << fb->sharpness());
            }
            // 为blender对象准备图片合成
            blender->prepare(corners, sizes);
        }
        else if (!timelapser && timelapse)
        {
            // 如果在延时摄影模式且未创建timelapser则创建它
            timelapser = Timelapser::createDefault(timelapse_type);
            timelapser->initialize(corners, sizes);
        }


        // 把当前图片合成到全景图中
        if (timelapse)
        {
            timelapser->process(img_warped_s, Mat::ones(img_warped_s.size(), CV_8UC1), corners[img_idx]);
            String fixedFileName;
            size_t pos_s = String(img_names[img_idx]).find_last_of("/\\");
            if (pos_s == String::npos)
            {
                fixedFileName = "fixed_" + img_names[img_idx];
            }
            else
            {
                fixedFileName = "fixed_" + String(img_names[img_idx]).substr(pos_s + 1, String(img_names[img_idx]).length() - pos_s);
            }
            imwrite(fixedFileName, timelapser->getDst());
        }
        else
        {
            // 如果不在延时摄影模式则用blender合成图片
            blender->feed(img_warped_s, mask_warped, corners[img_idx]);
        }
    }


    // 如果不在延时摄影模式则完成图片合成，融合所有图片并写入输出文件
    if (!timelapse)
    {
        Mat result, result_mask;
        blender->blend(result, result_mask);


        // 记录合成的耗时
        LOGLN("Compositing, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");


        // 将合成的结果图片写入文件
        imwrite(result_name, result);
    }


    // 输出程序总体完成的时间信息
    LOGLN("Finished, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec");
    return 0;
}