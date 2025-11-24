import os
import time
import imutils

import cv2
import pandas as pd
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from marnav_interfaces.msg import AisBatch, Gnss
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

import threading
from concurrent.futures import ThreadPoolExecutor # 用于管理线程池
import queue # 而 queue.Queue 天生支持线程安全和长度限制，相较于list，queue是多线程图像传递的最佳选择
from collections import deque # 双端队列，双端队列（Double-Ended Queue），支持高效的队头队尾插入删除

from utils.file_read import read_all, ais_initial, update_time, time2stamp
from utils.VIS_utils import VISPRO
from utils.AIS_utils import AISPRO
from utils.FUS_utils import FUSPRO
from utils.gen_result import gen_result
from utils.draw import DRAW

# DeepSORVF ROS 2 版本节点
# 修改自 DeepSORVF_ros_v2.py
# 修改内容主要包括：
# 1. 扩展为多相机支持，使用JH_ros2_stitch_node类似的同步器

# 当前使用标准：
# 1.时间戳统一以AIS数据集的2022年为准

# 存在问题:
# 1. AIS数据比GNSS数据延迟0.5秒左右，导致绘制时GNSS数据总是落后于AIS数据0.5秒左右，需要在处理时进行时间对齐
# 2. 为了匹配ais.csv数据集中，由于数据集录制时间和视频节点发布时间不一致，ais时间戳和程序运行时的时间戳不匹配的问题，需要对此处的时间戳进行调整:
# 目前的解决方案是通过获得ais数据集的起始时间，然后通过计算当前时间和起始时间的差值，来调整程序运行时的时间戳，使其与ais时间对齐
# 创建了 ('ais_start_timestamp', 1654315512000),  # AIS数据集起始时间戳，单位：毫秒
# 参数用于配置ais数据集的起始时间戳 self.time_offset = now_timestamp - self.ais_start_timestamp]
# 3. 运行一分钟左右后检测效果变差，而原代码不会，怀疑是线程问题，需要进一步排查


# 4. 多线程并行处理执行一段时间后，在视频流正常发布的情况下，程序却不再执行process_single_camera_frame这个函数了,
#    初步判定是线程池任务队列满了，导致无法再提交新任务，需要增加队列长度限制和日志提示,
#    因为默认的ThreadPoolExecutor使用的是无界队列，可能会导致任务积压过多，从而影响系统性能和响应速度
#    解决方案是自定义一个带有最大任务队列长度限制的线程池执行器BoundThreadPoolExecutor,
#    并在提交任务时进行检查，如果队列已满则丢弃任务并记录日志，避免阻塞主线程（目前并未编写完整的丢弃逻辑）


# 5. 考虑通过交错帧，在不同时间处理不同相机的帧来进一步提升效率，目前是每个相机独立线程并行处理（目前未实现）

# 6. 或者增添进程中函数处理的类型，比如AIS处理器、VIS处理器、FUS处理器等，分别创建线程池进行处理（目前未实现）:
#   这样可以更细粒度地控制每种处理器的并发数量，避免某一类处理器过载
#   例如，可以为隔帧进行AIS-VIS-FUS处理器创建一个线程池，为每一帧都进行绘图的DRA处理器创建另一个线程池
#   这样可以确保绘图处理器始终有足够的资源进行处理，而不会被其他处理器的任务阻塞
# 6.5 必须严格保证处理器执行顺序（AIS→VIS→FUS→DRA），通过add_done_callback链式触发。
#     共享数据（如bin_inf、latest_processed_images）必须用锁保护。
#     中间结果（如ais_vis、vis_tra）通过回调参数传递，避免跨线程共享变量。





class BoundThreadPoolExecutor(ThreadPoolExecutor):
    """带有最大任务队列长度限制的线程池执行器"""
    def __init__(self, max_workers=None, max_queue_size=0):
        super().__init__(max_workers=max_workers)
        self._work_queue = queue.Queue(maxsize=max_queue_size) # 替换为有界队列
    def submit(self, fn, *args, **kwargs):
        try:
            # 非阻塞提交任务，队列满时直接丢弃并记录日志
            return super().submit(fn, *args, **kwargs)
        except queue.Full:
            # 队列满时打印警告，避免阻塞
            rclpy.logging.get_logger("BoundThreadPool").warn("任务队列已满，丢弃当前任务")
            return None
    def shutdown(self, wait = True, timeout = None):
        # 取消未执行的任务
        while not self._work_queue.empty():
            try:
                self._work_queue.get_nowait()
            except queue.Empty:
                break
        super().shutdown(wait=wait, timeout=timeout)
        
class AisVisNode(Node):
    def __init__(self):
        super().__init__('ais_vis_node')  # ROS 2节点名称
        
        # ========================== 1. 声明ROS 2参数（替代argparse）==========================
        self.declare_parameters(
            namespace='',
            parameters=[
                # ========= 后需要改为动态传参，目前暂时是手动定义的参数 =========
                ('ais_start_timestamp', 1654315512000),  # AIS数据集起始时间戳，单位：毫秒
                ('width_height', [2560, 1440]),  # 图像宽高
                # =========================================================
                # ('camera_topic', 'camera_image_topic'),      # 相机话题名称
                # 改为多相机话题列表，默认3个相机
                ('camera_topics', ['/camera_image_topic_1', '/camera_image_topic_2', '/camera_image_topic_3']),
                ('aisbatch_topic', 'ais_batch_topic'),       # AIS批量话题名称
                ('gnss_topic', 'gnss_topic'),                # GNSS话题名称
                ('input_fps', 25),                                 # 视频帧率
                ('output_fps', 10),                                # 输出帧率
                ('camera_para', 
                 [56.0, 30.94, 2391.26, 2446.89, 1305.04, 855.214]),  # 相机Horizontal-FoV	Vertical-FoV	fx	fy	u0	v0
                ('anti', 1),                        # 抗遮挡开关
                ('anti_rate', 0),                    # 遮挡率
                ('sync_queue_size', 10),               # 同步队列大小
                ('sync_slop', 0.1)                    # 同步时间容差，单位秒
            ]
        )
        
        # ========================== 2. 获取参数值 ==========================
        self.ais_start_timestamp = self.get_parameter('ais_start_timestamp').get_parameter_value().integer_value
        self.im_shape = tuple(self.get_parameter('width_height').get_parameter_value().integer_array_value)
        self.camera_topics = self.get_parameter('camera_topics').get_parameter_value().string_array_value
        self.aisbatch_topic = self.get_parameter('aisbatch_topic').get_parameter_value().string_value
        self.gnss_topic = self.get_parameter('gnss_topic').get_parameter_value().string_value
        self.input_fps = self.get_parameter('input_fps').get_parameter_value().integer_value # 输入帧率
        self.t = int(1000 / self.input_fps)  # 每帧持续时间（毫秒）
        self.output_fps = self.get_parameter('output_fps').get_parameter_value().integer_value # 输出帧率
        self.camera_para = list(self.get_parameter('camera_para').get_parameter_value().double_array_value)
        self.anti = self.get_parameter('anti').get_parameter_value().integer_value
        self.anti_rate = self.get_parameter('anti_rate').get_parameter_value().integer_value
        self.sync_queue_size = self.get_parameter('sync_queue_size').get_parameter_value().integer_value
        self.sync_slop = self.get_parameter('sync_slop').get_parameter_value().double_value

        # ========================== 3. 初始化其他成员变量 ==========================
        now_timestamp = self.get_clock().now().to_msg().sec * 1000 + self.get_clock().now().to_msg().nanosec // 1000000  # 毫秒级时间戳
        self.time_offset = now_timestamp - self.ais_start_timestamp  # 计算时间偏移量,用于时间戳对齐的差值
        self.get_logger().info(f"Current timestamp: {now_timestamp}, AIS start timestamp: {self.ais_start_timestamp}, Time offset: {self.time_offset} ms\n")

        self.bridge = CvBridge()  # 用于图像消息转换
        self.aisbatch_cache = pd.DataFrame(columns=['ID', 'mmsi', 'timestamp', 'lat', 'lon', 'sog', 'cog', 'heading', 'status', 'type'])    
        self.aisbatch_time = None  # 用于记录最新的AIS时间   
        self.gnss_cache = None  # 用于缓存最新的GNSS数据
        self.camera_pos_para = None  # 相机位置和内部参数
        self.max_dis = min(self.im_shape) // 2  # 最大距离阈值
        self.name = 'ROS version 2 demo'
        
        self.num_cameras = len(self.camera_topics) # 相机数量
        self.processor_groups = self._init_processor_groups() # 每个相机一个处理器组
        # 修改bin_inf为列表，为每个相机存储独立的匹配信息
        self.bin_inf = [pd.DataFrame(columns=['ID', 'mmsi', 'timestamp', 'match']) for _ in range(self.num_cameras)] # 用于存储 每个相机的 视觉目标ID-nmsi 匹配信息
        # self.thread_pool = ThreadPoolExecutor(max_workers=self.num_cameras)  # 线程数与相机数一致
        self.thread_pool = BoundThreadPoolExecutor(
            max_workers= self.num_cameras * 2,  # 增加线程数
            max_queue_size=20   # 适当增大队列容量
        )  



        # ================================ useless ================================
        # self.stitch_image_queue = queue.Queue(maxsize=self.num_cameras*10)   # maxsize=3*10：最多缓存30帧（每个相机10帧）
        self.stitch_image_queue = deque(maxlen=self.num_cameras*10)  # 使用双端队列deque，最多缓存30帧（每个相机10帧）

        self.latest_stitch = None # 保存最近一帧的拼接结果
        # ==========================================================================


        # =================== debug ======================
        self.print_logger = False  # 是否打印日志
        # =================== debug ======================


        self.latest_processed_images = {
            'cam0': None,
            'cam1': None,
            'cam2': None
        }
        self.data_lock = threading.Lock()  # 保护最新处理图像的锁


        # 初始化时创建窗口（设置为可调整大小）
        cv2.namedWindow(self.name, cv2.WINDOW_NORMAL)

        # ========================== 4. 创建订阅器 ==========================
        # 创建相机图像订阅者
        qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=3
        )

        # 创建多相机同步订阅器
        self.camera_subscribers = []
        for topic in self.get_parameter('camera_topics').get_parameter_value().string_array_value:
            sub = Subscriber(self, Image, topic, qos_profile=qos_profile)
            self.camera_subscribers.append(sub)
        # 创建时间同步器,只同步多订阅器步相机图像，AIS和GNSS只有各自一个订阅者
        self.ts = ApproximateTimeSynchronizer(
            self.camera_subscribers,
            queue_size=self.sync_queue_size,
            slop=self.sync_slop
        )
        # 绑定同步回调函数
        self.ts.registerCallback(self.synchronized_camera_callback)


        # 创建AIS批量数据订阅者
        self.aisbatch_subscriber = self.create_subscription(
            AisBatch,
            self.aisbatch_topic,
            self.aisbatch_callback,
            10
        )
        # 创建GNSS数据订阅者
        self.gnss_subscriber = self.create_subscription(
            Gnss,
            self.gnss_topic,
            self.gnss_callback,
            10
        )
        # 创建定时器，每帧间隔（如40ms）在主线程刷新窗口
        self.window_timer = self.create_timer(1/self.output_fps, self.refresh_window_callback)
        
        # ========================== 5. 初始化资源（原main函数初始化逻辑）==========================
        # self.initialize_resources()

        # ================= 6. # 添加线程锁保护共享数据 =================
        self.ais_lock = threading.Lock()  # 保护AIS缓存的锁
        self.gnss_lock = threading.Lock()  # 保护GNSS缓存的锁
        self.bin_lock = threading.Lock()   # 保护匹配信息的锁


        # ========================== 6. 打印参数配置 ==========================
        # 打印参数配置（类似原代码的打印逻辑）
        self.get_logger().info("\nVesselSORT_ROS")
        self.get_logger().info(f"Camera topics: {self.camera_topics}")
        self.get_logger().info(f"AIS batch topic: {self.aisbatch_topic}")
        self.get_logger().info(f"GNSS topic: {self.gnss_topic}")

        # ========================== 7. 创建对应的回调函数 ==========================

    # 多相机同步回调函数
    def synchronized_camera_callback(self, *msgs):
        """处理接收到的多相机同步图像消息"""
        # self.get_logger().info(f"收到同步图像消息，数量：{len(msgs)}，当前队列大小：{self.thread_pool._work_queue.qsize()}")
        
        # 将ROS图像消息转换为OpenCV图像列表
        cv_images = []
        for msg in msgs:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_images.append(cv_image)
        
        with self.ais_lock:
            with self.gnss_lock:
                has_ais = not self.aisbatch_cache.empty
                has_gnss = self.gnss_cache is not None

        if not has_ais or not has_gnss:
            # self.get_logger().info("没有AIS或GNSS数据则直接返回")
            return  # 如果没有AIS或GNSS数据则直接返回
        

        # 获得当前图像的时间戳（假设所有相机图像时间戳相同，取第一个即可）
        current_timestamp = msgs[0].header.stamp.sec * 1000 + msgs[0].header.stamp.nanosec // 1000000  # 毫秒级时间戳

        # 1. 计算当前队列积压程度
        queue_size = self.thread_pool._work_queue.qsize()
        
        # 2在提交任务前检查
        # 根据积压程度动态调整抽帧比例
        if queue_size < 5:
            skip_ratio = 1  # 不抽帧
        elif 5 <= queue_size < 15:
            skip_ratio = 2  # 每2秒处理1次（跳过1秒）
        else:
            skip_ratio = 4  # 每4秒处理1次（跳过3秒）
        

        


        # 3. 处理剩余帧（按之前的逐个相机提交逻辑）

        # 为每个相机启动独立的后台线程处理（传入相机索引、图像、时间戳）,cam_idx 从0开始
        for cam_idx in range(self.num_cameras):
            # 确保索引不越界（避免同步消息数量与相机数量不匹配）
            if cam_idx >= len(cv_images):
                continue
            
            # 提交任务到线程池，而非创建新线程
            self.thread_pool.submit(
                self.process_single_camera_frame,
                cam_idx, cv_images[cam_idx], current_timestamp ,skip_ratio
            )


    def process_single_camera_frame(self, cam_idx, cv_image, current_timestamp, skip_ratio):
        start_time = time.time()

 
        """处理单个相机的图像帧"""

        try:
            # 获得对应相机的处理器组
            processor_group = self.processor_groups[cam_idx]
            AIS_processor = processor_group['AIS']
            VIS_processor = processor_group['VIS']
            FUS_processor = processor_group['FUS']
            DRA_processor = processor_group['DRA']
        # 线程安全地获取共享数据
            with self.ais_lock:
                with self.gnss_lock:
                    ais_batch = self.aisbatch_cache.copy()
                    camera_pos = self.camera_pos_para.copy()
            with self.bin_lock:
                # 为每个相机维护独立的匹配信息（需修改bin_inf为列表，按相机索引存储）
                bin_info = self.bin_inf[cam_idx].copy() if len(self.bin_inf) > cam_idx else pd.DataFrame()
 
            # 保留最近N条数据（如100条）
            if len(self.bin_inf[cam_idx]) > 100:
                self.bin_inf[cam_idx] = self.bin_inf[cam_idx].tail(100)


            # 1. 当前相机的AIS轨迹提取（使用专属AIS处理器）
            # ===================== debug ======================
            ais_skip_ratio = 4  # AIS数据抽帧比例
            # ===================== debug ======================
            
            time_start_ais = time.time()
            AIS_vis, AIS_cur = AIS_processor.process(ais_batch, camera_pos, current_timestamp, skip_ratio = ais_skip_ratio)
            time_end_ais = time.time()
            time_cost_ais = time_end_ais - time_start_ais

            # 2. 当前相机的视觉轨迹提取（使用专属VIS处理器）
            # ===================== debug ======================
            vis_skip_ratio = 4  # VIS数据抽帧比例
            # ===================== debug ======================

            time_start_vis = time.time()    
            Vis_tra, Vis_cur = VIS_processor.feedCap(cv_image, current_timestamp, AIS_vis, bin_info, skip_ratio = vis_skip_ratio)
            time_end_vis = time.time()
            time_cost_vis = time_end_vis - time_start_vis

            # 3. 当前相机的数据融合（使用专属FUS处理器）
            # ===================== debug ======================
            fus_skip_ratio = 4  # VIS数据抽帧比例
            # ===================== debug ======================
            time_start_fus = time.time()
            Fus_tra, updated_bin = FUS_processor.fusion(AIS_vis, AIS_cur, Vis_tra, Vis_cur, current_timestamp, skip_ratio = fus_skip_ratio)
            time_end_fus = time.time()
            time_cost_fus = time_end_fus - time_start_fus

            


            # 4. 线程安全地更新当前相机的匹配信息
            with self.bin_lock:
                if cam_idx < len(self.bin_inf):
                    self.bin_inf[cam_idx] = updated_bin
                else:
                    # 若列表长度不足，动态扩展
                    self.bin_inf.extend([pd.DataFrame()]*(cam_idx - len(self.bin_inf) + 1))
                    self.bin_inf[cam_idx] = updated_bin

            # 5. 绘制当前相机的结果（使用专属DRA处理器）
            im = DRA_processor.draw_traj(
                cv_image, AIS_vis, AIS_cur, Vis_tra, Vis_cur, Fus_tra, timestamp=current_timestamp,  skip_ratio = 4
                # cam_id=cam_idx  # (will be 可选)在绘制时标注相机ID
            )
            
            with self.data_lock:
                self.latest_processed_images[f'cam{cam_idx}'] = im



        except Exception as e:
            self.get_logger().error(f"Camera {cam_idx} 致命错误: {str(e)}", exc_info=True)  # 打印完整堆栈
            # 异常后重置当前相机的处理器状态（可选，根据业务逻辑）
            self.processor_groups[cam_idx] = self._init_processor_groups()[cam_idx]
        finally:
            end_time = time.time()
            total_cost_time = end_time - start_time
            # ===================== debug ======================
            if current_timestamp % int(ais_skip_ratio * 1000) < self.t \
                and current_timestamp % int(vis_skip_ratio * 1000) < self.t \
                and current_timestamp % int(fus_skip_ratio * 1000) < self.t:
                self.get_logger().info(
                    f"Camera {cam_idx} 时间戳 {current_timestamp} 处理时间: "
                    f"AIS: {time_cost_ais:.3f}s, VIS: {time_cost_vis:.3f}s, FUS: {time_cost_fus:.3f}s, Camera {cam_idx} 总处理耗时: {total_cost_time:.3f}秒"
                )
                self.print_logger = False
            # ===================== debug ======================
            

    def aisbatch_callback(self, msg: AisBatch):
        
        # self.get_logger().info(f"Received AIS batch with {len(msg.ais_list)} records")
        # 用列表收集数据，避免频繁 append
        data_list = []
        for ais in msg.ais_list:
            timestamp_ms = ais.timestamp.sec * 1000 + ais.timestamp.nanosec // 1000000
            data_list.append({
                'mmsi': ais.mmsi,
                'lon': ais.lon,
                'lat': ais.lat,
                'speed': ais.speed,
                'course': ais.course,
                'heading': ais.heading,
                'type': ais.type,
                'timestamp': timestamp_ms
            })
        # 一次性创建 DataFrame
        aisbatch_df = pd.DataFrame(data_list, columns=['mmsi','lon','lat','speed','course','heading','type','timestamp'])
        self.aisbatch_cache = aisbatch_df
        # 保留最近100条
        if len(self.aisbatch_cache) > 100:
            self.aisbatch_cache = self.aisbatch_cache.tail(100).copy()
    
    
    def gnss_callback(self, msg: Gnss):
        """处理接收到的GNSS消息"""
        # self.get_logger().info("Received GNSS message")
        self.gnss_cache = msg
        self.camera_pos_para = [msg.longitude, msg.latitude, msg.horizontal_orientation, msg.vertical_orientation, msg.camera_height] + self.camera_para
        # GNSS数据处理逻辑（后续补充）


    def refresh_window_callback(self):
        # refresh_window_callback 必须在主线程调用

        with self.data_lock:
            current_images  = self.latest_processed_images.copy()

        for cam_name, img in current_images.items():
            if img is  None:
                self.get_logger().info(f"没有处理图像 {cam_name}，跳过显示")
                return  # 如果没有图像则直接返回
            
            
        # 拼接所有相机图像
        stitched_image = cv2.hconcat([current_images[f'cam{idx}'] for idx in range(self.num_cameras)])

        # 2. 获取当前时间（精确到秒）
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

        # 3. 绘制时间到图片上
        # 参数说明：图片对象、文本内容、起始坐标、字体、字体大小、颜色（BGR）、线条粗细
        cv2.putText(
            stitched_image,
            f"Time: {current_time}",  # 文本内容（可自定义前缀，如"Current Time: "）
            (50, 500),                 # 起始坐标（x=50, y=50，避免贴边）
            cv2.FONT_HERSHEY_SIMPLEX, # 常用字体（清晰易读）
            3,                      # 字体大小（根据图片分辨率调整，如2.0适合大图片）
            (0, 0, 255),              # 颜色（BGR格式，此处为红色）
            2                         # 线条粗细（2像素足够清晰，避免过细模糊）
        )


        # 显示拼接图像
        # self.get_logger().info("定时显示函数")
        cv2.imshow(self.name, stitched_image)
        cv2.waitKey(1)
    

    def save_to_local(self, AIS_vis, AIS_cur, Vis_tra, Vis_tra_cur, Fus_tra, current_timestamp):
        """将AIS_vis和AIS_cur保存到本地CSV文件"""
        if current_timestamp % 1000 < self.t:
            result_name_vis = f'result/AIS_vis/AIS_vis_{current_timestamp}.csv'
            result_name_cur = f'result/AIS_cur/AIS_cur_{current_timestamp}.csv'
            result_name_tra_cur = f'result/Vis_tra_cur/Vis_tra_cur_{current_timestamp}.csv'
            result_name_tra = f'result/Vis_tra/Vis_tra_{current_timestamp}.csv'
            result_name_fus_tra = f'result/Fus_tra/Fus_tra_{current_timestamp}.csv'
            AIS_vis.to_csv(result_name_vis, index=False)
            AIS_cur.to_csv(result_name_cur, index=False)
            Vis_tra.to_csv(result_name_tra, index=False)
            Vis_tra_cur.to_csv(result_name_tra_cur, index=False)
            Fus_tra.to_csv(result_name_fus_tra, index=False)


    def _init_processor_groups(self):
        """初始化处理器组，每个相机一个处理器组"""
        processor_groups = []
        for i in range(self.num_cameras):
            processor_group = {
                'AIS': AISPRO(self.im_shape , self.t),  # 初始化AIS处理器
                'VIS': VISPRO(self.anti, self.anti_rate, self.t), # 视觉处理器初始化
                'FUS': FUSPRO(self.max_dis, self.im_shape, self.t), # 融合处理器初始化
                'DRA': DRAW(self.im_shape, self.t) # 绘图处理器初始化
            }
            processor_groups.append(processor_group)
        return processor_groups
    
    def destroy_node(self):
        # 关闭线程池，等待所有任务完成（超时时间可调整）
        self.thread_pool.shutdown(wait=True, timeout=5)
        super().destroy_node()  # 调用原来的父类销毁方法
        
        
        


    


    


def main(args=None):
    # 初始化ROS 2
    rclpy.init(args=args)
    
    # 创建节点并运行
    node = AisVisNode()
    
    try:
        # 进入事件循环，保持节点运行以处理回调
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获Ctrl+C中断，优雅退出
        node.get_logger().info("Node interrupted by user")
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
