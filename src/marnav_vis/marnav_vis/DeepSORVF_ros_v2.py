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

import threading

from utils.file_read import read_all, ais_initial, update_time, time2stamp
from utils.VIS_utils import VISPRO
from utils.AIS_utils import AISPRO
from utils.FUS_utils import FUSPRO
from utils.gen_result import gen_result
from utils.draw import DRAW

# 当前使用标准：
# 1.时间戳统一以AIS数据集的2022年为准
# 存在问题:
# 1. AIS数据比GNSS数据延迟0.5秒左右，导致绘制时GNSS数据总是落后于AIS数据0.5秒左右，需要在处理时进行时间对齐
# 2. 为了匹配ais.csv数据集中，由于数据集录制时间和视频节点发布时间不一致，ais时间戳和程序运行时的时间戳不匹配的问题，需要对此处的时间戳进行调整:
# 目前的解决方案是通过获得ais数据集的起始时间，然后通过计算当前时间和起始时间的差值，来调整程序运行时的时间戳，使其与ais时间对齐
# 创建了 ('ais_start_timestamp', 1654315512000),  # AIS数据集起始时间戳，单位：毫秒
# 参数用于配置ais数据集的起始时间戳 self.time_offset = now_timestamp - self.ais_start_timestamp
# 3. 运行一分钟左右后检测效果变差，而原代码不会，怀疑是线程问题，需要进一步排查

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
                ('camera_topic', 'camera_image_topic'),      # 相机话题名称
                ('aisbatch_topic', 'ais_batch_topic'),       # AIS批量话题名称
                ('gnss_topic', 'gnss_topic'),                # GNSS话题名称
                ('fps', 25),                                 # 视频帧率
                ('camera_para', 
                 [56.0, 30.94, 2391.26, 2446.89, 1305.04, 855.214]),  # 相机Horizontal-FoV	Vertical-FoV	fx	fy	u0	v0
                ('anti', 1),                        # 抗遮挡开关
                ('anti_rate', 0)                    # 遮挡率
            ]
        )
        
        # ========================== 2. 获取参数值 ==========================
        self.ais_start_timestamp = self.get_parameter('ais_start_timestamp').get_parameter_value().integer_value
        self.im_shape = tuple(self.get_parameter('width_height').get_parameter_value().integer_array_value)
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.aisbatch_topic = self.get_parameter('aisbatch_topic').get_parameter_value().string_value
        self.gnss_topic = self.get_parameter('gnss_topic').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.t = int(1000 / self.fps)  # 每帧持续时间（毫秒）
        self.camera_para = list(self.get_parameter('camera_para').get_parameter_value().double_array_value)
        self.anti = self.get_parameter('anti').get_parameter_value().integer_value
        self.anti_rate = self.get_parameter('anti_rate').get_parameter_value().integer_value
        
        # ========================== 3. 初始化其他成员变量 ==========================
        now_timestamp = self.get_clock().now().to_msg().sec * 1000 + self.get_clock().now().to_msg().nanosec // 1000000  # 毫秒级时间戳
        self.time_offset = now_timestamp - self.ais_start_timestamp  # 计算时间偏移量,用于时间戳对齐的差值
        self.get_logger().info(f"Current timestamp: {now_timestamp}, AIS start timestamp: {self.ais_start_timestamp}, Time offset: {self.time_offset} ms\n")

        self.bridge = CvBridge()  # 用于图像消息转换
        self.aisbatch_cache = pd.DataFrame(columns=['ID', 'mmsi', 'timestamp', 'lat', 'lon', 'sog', 'cog', 'heading', 'status', 'type'])    
        self.aisbatch_time = None  # 用于记录最新的AIS时间
        self.gnss_cache = None  # 用于缓存最新的GNSS数据
        self.camera_pos_para = None  # 相机位置和内部参数
        self.bin_inf = pd.DataFrame(columns=['ID', 'mmsi', 'timestamp', 'match']) # 用于存储 视觉目标ID-nmsi 匹配信息
        self.max_dis = min(self.im_shape) // 2  # 最大距离阈值
        self.name = 'ROS version 2 demo'

        self.image_queue = []  # 图像队列
        # 1. 初始化时创建窗口（设置为可调整大小）
        cv2.namedWindow(self.name, cv2.WINDOW_NORMAL)

        # ========================== 4. 创建订阅器 ==========================
        # 创建相机图像订阅者
        qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=3
        )
        self.camera_subscriber = self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            qos_profile
        )
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
        self.window_timer = self.create_timer(1/self.fps, self.refresh_window_callback)
        
        # ========================== 5. 初始化资源（原main函数初始化逻辑）==========================
        self.initialize_resources()

        # ========================== 6. 打印参数配置 ==========================
        # 打印参数配置（类似原代码的打印逻辑）
        self.get_logger().info("\nVesselSORT_ROS")
        self.get_logger().info(f"Camera topic: {self.camera_topic}")
        self.get_logger().info(f"AIS batch topic: {self.aisbatch_topic}")
        self.get_logger().info(f"GNSS topic: {self.gnss_topic}")

        # ========================== 7. 创建对应的回调函数 ==========================
    def camera_callback(self, msg: Image):
        """处理接收到的相机图像消息"""
        """是整个程序的入口，后续处理均基于接收到的图像进行"""
        self.get_logger().info("Received camera image message")
        
        # 将ROS图像消息转换为OpenCV图像
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # 获得当前图像的时间戳
        if self.aisbatch_cache.empty:
            self.get_logger().info("没有AIS数据则直接返回")
            return  # 如果没有AIS数据则直接返回
        elif self.gnss_cache is None:
            self.get_logger().info("没有GNSS数据则直接返回")
            return  # 如果没有GNSS数据则直接返回
        else:
            self.get_logger().info(f"满足条件，开始处理图像\n")

        current_timestamp = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1000000  # 毫秒级时间戳
        # current_timestamp -= self.time_offset  # 时间戳统一以AIS数据集的2022年为准
        # print(f"Adjusted current timestamp for AIS data: {current_timestamp}\n")

        threading.Thread(
            target=self.process_frame,
            args=(cv_image, current_timestamp),
            daemon=True # 线程随节点退出
        ).start()

    def process_frame(self, cv_image, current_timestamp):
        # 1. AIS轨迹提取，处理AIS数据，获得当前时间点的AIS信息
        AIS_vis, AIS_cur = self.AIS.process(self.aisbatch_cache,self.camera_pos_para, current_timestamp)
        
        # 显示 AIS_vis AIS_cur 信息的数量和内容
        # self.get_logger().info(f"AIS_vis contains {len(AIS_vis)} records")
        # self.get_logger().info(f"AIS_cur contains {len(AIS_cur)} records\n")

        # 2. 视觉轨迹提取
        Vis_tra, Vis_cur = self.VIS.feedCap(cv_image, current_timestamp, AIS_vis, self.bin_inf)

        # 3. 视觉与AIS数据融合
        Fus_tra, self.bin_inf = self.FUS.fusion(AIS_vis, AIS_cur, Vis_tra, Vis_cur, current_timestamp)

        # 保存到本地
        # self.save_to_local(AIS_vis, AIS_cur, Vis_tra, Vis_cur, Fus_tra, current_timestamp)

        # 4. 结果的保存
        im = self.DRA.draw_traj(cv_image, AIS_vis, AIS_cur, Vis_tra, Vis_cur, Fus_tra, timestamp=current_timestamp)
        self.image_queue.append(im)

        # cv2.imshow(self.name, result)
        # cv2.waitKey(1)
        
        
        # 处理ROS 2事件（如节点关闭信号）
        # rclpy.spin_once(self, timeout_sec=0.001) # 0.001 
        
    
    def aisbatch_callback(self, msg: AisBatch):
        """处理接收到的AIS批量消息"""
        self.get_logger().info(f"Received AIS batch message with {len(msg.ais_list)} records")
        # 新建一个函数，将AisBatch消息转换为DataFrame格式以便处理
        aisbatch_df = pd.DataFrame(columns=['mmsi','lon','lat','speed','course','heading','type','timestamp'])
        for ais in msg.ais_list:
            timestamp_ms = ais.timestamp.sec * 1000 + ais.timestamp.nanosec // 1000000  # 核心修改
            aisbatch_df = aisbatch_df.append({
                'mmsi': ais.mmsi,
                'lon': ais.lon,
                'lat': ais.lat,
                'speed': ais.speed,
                'course': ais.course,
                'heading': ais.heading,
                'type': ais.type,
                'timestamp': timestamp_ms # 秒级+毫秒级-> 毫秒级时间戳
            }, ignore_index=True)        

        self.aisbatch_cache = aisbatch_df

        # =========================将当前AISBatch保存到本地CSV文件（用于调试）========================
        # result_name_vis = f'result/AIS_batch_cache/AIS_batch_cache_{self.aisbatch_time}.csv'
        # self.aisbatch_cache.to_csv(result_name_vis, index=False)
        # =========================================

        # 记录最新的AISBatch时间
        self.aisbatch_time = msg.batch_time
        # AIS数据处理逻辑（后续补充）
        # 保留最近 100 条数据，防止缓存过大
        if len(self.aisbatch_cache) > 100:
            self.aisbatch_cache = self.aisbatch_cache.tail(100) # tail的意思是保留最后几行数据
            

    def gnss_callback(self, msg: Gnss):
        """处理接收到的GNSS消息"""
        self.get_logger().info("Received GNSS message")
        self.gnss_cache = msg
        self.camera_pos_para = [msg.longitude, msg.latitude, msg.horizontal_orientation, msg.vertical_orientation, msg.camera_height] + self.camera_para
        # GNSS数据处理逻辑（后续补充）


    def refresh_window_callback(self):
        # 必须在主线程调用
        if not self.image_queue:
            return  # 如果没有图像则直接返回
        
        im = self.image_queue.pop(0) if self.image_queue else None # 取出最新图像
        result = imutils.resize(im, width=self.im_shape[0]//2)

        # # 先确保窗口已创建（关键修改）
        # if not cv2.getWindowProperty(self.name, cv2.WND_PROP_EXIST):
        #     cv2.namedWindow(self.name)
        
        # 检查窗口是否可见，不可见则重新显示
        if cv2.getWindowProperty(self.name, cv2.WND_PROP_VISIBLE) < 1:
            cv2.namedWindow(self.name)

        cv2.imshow(self.name, result)

        # 避免self.image_queue太多占用内存
        while len(self.image_queue) > 5:
            # 只保留最新的5帧图像
            self.image_queue.pop(0)

        key = cv2.waitKey(1)
        if key == ord('q'):  # 允许通过按键退出
            self.destroy_node()
            rclpy.shutdown()



    def save_to_local(self, AIS_vis, AIS_cur, Vis_tra, Vis_tra_cur, Fus_tra, current_timestamp):
        """将AIS_vis和AIS_cur保存到本地CSV文件"""
        # AIS_cur  = pd.DataFrame(columns=['mmsi','lon','lat','speed','course','heading','type','timestamp'])
        # AIS_vis  = pd.DataFrame(columns=['mmsi','lon','lat','speed','course','heading','type','x','y','timestamp'])
        # Vis_tra  = pd.DataFrame(columns=['ID','x1','y1','x2','y2','x','y','timestamp'])
        # Vis_tra_cur  = pd.DataFrame(columns=['ID','x1','y1','x2','y2','x','y','timestamp'])
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


    def initialize_resources(self):
        """初始化所需的资源和参数"""
        
        # 1. AIS处理器初始化
        self.AIS = AISPRO(self.im_shape , self.t)  # 初始化AIS处理器

        # 2. 视觉处理器初始化
        self.VIS = VISPRO(self.anti, self.anti_rate, self.t)

        # 3. 融合处理器初始化
        self.FUS = FUSPRO(self.max_dis, self.im_shape, self.t)

        # 4. 绘图处理器初始化
        self.DRA = DRAW(self.im_shape, self.t)

        
        
        
        
        


    


    


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
