# 版本特点:
# 1. 基于ROS2开发，要求轨迹数据在本程序完成不同相机之间的坐标映射，而不是在C++主节点完成。
# 2. 因此需要读取不同相机的内参和外参
# 3. 联动现有的JH拼接代码，实现多相机拼接显示，所用参数需要求与JH拼接代码一致
# 4. utils也需要适配JH拼接代码，另新建一个utils_JH文件夹
import os
import time
import imutils

import cv2
import pandas as pd
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import builtin_interfaces
from sensor_msgs.msg import Image
from marnav_interfaces.msg import AisBatch, Gnss, VisiableTra, VisiableTraBatch

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import traceback # traceback用于打印异常信息

# CPU密集型任务，使用多进程处理，而不是多线程处理，多线程适用于IO密集型任务
from multiprocessing import Process, Queue, cpu_count, set_start_method
import multiprocessing

from utils_JH.file_read import read_all, ais_initial, update_time, time2stamp
from utils_JH.VIS_utils import VISPRO
from utils_JH.AIS_utils import AISPRO
from utils_JH.FUS_utils import FUSPRO
from utils_JH.gen_result import gen_result
from utils_JH.draw import DRAW
from marnav_vis.config_loader import ConfigLoader

# 由v4的多线程改为v5的多进程，每个进程独立处理一帧: 包含AIS, VIS, FUS, DRAW

# ------------------------------------------------------
def multi_proc_worker(input_queue, output_queue, im_shape, t, max_dis, skip_interval = 1, camera_type = "normal", yolo_type = "yolo11m"):
    """
    每个进程独立处理一帧: 包含AIS, VIS, FUS, DRAW
     1. 从输入队列获取任务
     2. 处理AIS, VIS, FUS, DRAW
     3. 将结果放入输出队列
     参数:
        input_queue: 多进程输入队列，包含了:
                self.mp_input_queue[cam_topic].put({
                    "cam_idx": cam_idx,
                    "cv_image": cv_images[cam_idx],
                    "current_timestamp": current_timestamp,
                    "ais_batch": self.aisbatch_cache.copy(),
                    "camera_pos_para": self.camera_pos_para,
                    "bin_inf": self.bin_inf[cam_idx].copy() if len(self.bin_inf) > cam_idx else pd.DataFrame(),
                })
                    其中:
                    camera_pos_para = {
                        "longitude": msg.longitude,
                        "latitude": msg.latitude,
                        "horizontal_orientation": horizontal_orientation,
                        "vertical_orientation": msg.vertical_orientation,
                        "camera_height": msg.camera_height,
                        'fov_hor': resp.fov_hor,
                        'fov_ver': resp.fov_ver,
                        'focal': resp.focal,
                        'fx': resp.fx,
                        'fy': resp.fy,
                        'u0': resp.u0,
                        'v0': resp.v0,
                        # 下列限定于鱼眼相机，普通相机不需要
                        'k1': resp.k1,
                        'k2': resp.k2,
                        'k3': resp.k3,
                        'k4': resp.k4,
                    }


        output_queue: 多进程输出队列，包含了:
                    cam_idx,
                    image
                    timestamp 
                    updated_bin
                    time_cost
        t: 时间间隔 (ms)
        camera_para: 相机参数字典
        max_dis: 最大距离
    """
    print(f"worker process started, pid={os.getpid()}")

    aispro = AISPRO(im_shape, t)
    vispro = VISPRO(1, 0, t, yolo_type)  # 是否读取anti参数可以自定义
    fuspro = FUSPRO(max_dis, im_shape, t)
    dra = DRAW(im_shape, t)

    # 缓存上一窗口的融合轨迹
    Last_Visiable_Tra = pd.DataFrame()
    # 记录上次处理时所属的时间窗口
    last_processed_window = -1
    while True:
        start_time = time.time()
        task = input_queue.get()
        if task is None:
            break
        try:
            cam_idx = task["cam_idx"]
            cv_image = task["cv_image"]
            current_timestamp = task["current_timestamp"]
            ais_batch = task["ais_batch"]
            camera_pos_para = task["camera_pos_para"]
            bin_inf = task["bin_inf"]
            

            # 计算当前时间戳属于哪个窗口
            current_window = current_timestamp // skip_interval
            
            # 只有进入新窗口时才处理
            process_ais_vis_fus = (current_window != last_processed_window)
            
            if process_ais_vis_fus:
                # print(f"process_ais_vis_fus at timestamp {current_timestamp}, worker {cam_idx} processing task, pid={os.getpid()}")
                # 1. AIS
                AIS_vis, AIS_cur = aispro.process(ais_batch, camera_pos_para, current_timestamp, camera_type)
                # 2. VIS
                Vis_tra, Vis_cur = vispro.feedCap(cv_image, AIS_vis, bin_inf, current_timestamp)
                # 3. FUS
                Fus_tra, updated_bin = fuspro.fusion(AIS_vis, AIS_cur, Vis_tra, Vis_cur, current_timestamp)
                # 4. DRAW
                im, Visiable_Tra = dra.draw_match_traj(cv_image, AIS_vis, AIS_cur, Vis_tra, Vis_cur, Fus_tra, timestamp=current_timestamp)
                # 更新配对关系
                Last_Visiable_Tra = Visiable_Tra
                # 更新窗口标记
                last_processed_window = current_window
            else:
                # print(f"no process_ais_vis_fus at timestamp {current_timestamp}, worker {cam_idx} processing task, pid={os.getpid()}")
                im = dra.draw_no_match_traj(cv_image)
                Visiable_Tra = Last_Visiable_Tra
            
            end_time = time.time()
            time_cost = end_time - start_time
            result = {
                "cam_idx": cam_idx, 
                "image": im, 
                "timestamp": current_timestamp, 
                "Visiable_Tra": Visiable_Tra,
                # "fus_trajectory": pd.DataFrame(),
                "time_cost": time_cost
            }
            try:
                output_queue.put_nowait(result)
            except:
                print(f"[PID {os.getpid()}] output queue full for cam{cam_idx}, drop result")
        except Exception as e:
            error_msg = f"!!! worker cam{cam_idx} error: {e}\n{traceback.format_exc()}"
            print(error_msg)
            try:
                output_queue.put_nowait({"cam_idx": cam_idx, "error": error_msg})
            except:
                pass
# ------------------------------------------------------





class AisVisNode(Node):
    def __init__(self,):
        super().__init__('ais_vis_node')

        # 声明配置文件参数
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        # 如果未指定配置文件，使用默认路径
        if not config_file:
            try:
                config_file = ConfigLoader.find_config_file('marnav_vis', 'track_realtime_config.yaml')
                self.get_logger().info(f"未指定配置文件，使用默认路径: {config_file}")
            except Exception as e:
                self.get_logger().error(f"查找默认配置文件失败: {e}")
                raise
        
        # 加载配置
        try:
            config_loader = ConfigLoader(config_file)
            camera_config = config_loader.get_camera_config()
            ais_config = config_loader.get_ais_config()
            gnss_config = config_loader.get_gnss_config()
            deepsorvf_config = config_loader.get_deepsorvf_config()
        except Exception as e:
            self.get_logger().fatal(f"加载配置文件失败: {e}")
            raise
        
        # 从配置中读取参数
        self.angle_between_cameras = camera_config.get('angle_between_cameras')
        self.im_shape = tuple(camera_config.get('width_height', [2560, 1440]))
        self.camera_parameters = camera_config.get('camera_parameters', [])
        self.camera_topics = [cam['topic_name'] for cam in self.camera_parameters]
        self.camera_matrixs = [cam['camera_matrix'] for cam in self.camera_parameters]  
        self.camera_type = deepsorvf_config.get('camera_type', "normal")
        if self.camera_type == 'fisheye':
            self.distortion_coefficients = [cam['distortion_coefficients'] for cam in self.camera_parameters]
        else:
            self.distortion_coefficients = None # 普通相机不需要畸变系数
        # 构建相机topic到标准名称的映射（与C++拼接节点保持一致）
        self.camera_name_mapping = {}
        for cam in self.camera_parameters:
            self.camera_name_mapping[cam['topic_name']] = cam['camera_name']
        
        self.ais_batch_pub_topic = ais_config.get('ais_batch_pub_topic', '/ais_batch_topic')
        self.fus_trajectory_topic = deepsorvf_config.get('fus_trajectory_topic', '/fus_trajectory_topic')
        self.gnss_pub_topic = gnss_config.get('gnss_pub_topic', '/gnss_pub_topic')
        
        self.input_fps = deepsorvf_config.get('input_fps', 20)
        self.t = int(1000 / self.input_fps)
        self.output_fps = deepsorvf_config.get('output_fps', 10)
        
        self.anti = deepsorvf_config.get('anti', 1)
        self.anti_rate = deepsorvf_config.get('anti_rate', 0)
        self.sync_queue_size = deepsorvf_config.get('sync_queue_size', 10)
        self.sync_slop = deepsorvf_config.get('sync_slop', 0.1)
        self.skip_interval = deepsorvf_config.get('skip_interval', 1000)
        self.camera_type = deepsorvf_config.get('camera_type', "normal")
        self.yolo_type = deepsorvf_config.get('yolo_type', "yolo11m")
        
        # 打印配置信息
        self.get_logger().info("="*60)
        self.get_logger().info("🚢 船只跟踪节点配置")
        self.get_logger().info("="*60)
        self.get_logger().info(f"配置文件: {config_file}")
        self.get_logger().info(f"图像尺寸: {self.im_shape[0]}x{self.im_shape[1]}")
        self.get_logger().info(f"相机数量: {len(self.camera_topics)}")
        for i, (topic, name) in enumerate(self.camera_name_mapping.items()):
            self.get_logger().info(f"  相机映射关系{i}: {topic} -> {name}")
        self.get_logger().info(f"输入/输出FPS: {self.input_fps}/{self.output_fps}")
        self.get_logger().info(f"处理间隔: {self.skip_interval} ms")
        self.get_logger().info(f"同步队列: {self.sync_queue_size}, 同步误差: {self.sync_slop}s")
        self.get_logger().info(f"相机类型: {self.camera_type}")
        self.get_logger().info(f"YOLO模型类型: {self.yolo_type}")
        self.get_logger().info("="*60)

        self.bridge = CvBridge()
        self.aisbatch_cache = pd.DataFrame(columns=['ID', 'mmsi', 'timestamp', 'lat', 'lon', 'sog', 'cog', 'heading', 'status', 'type'])
        self.aisbatch_time = None
        # self.gnss_cache = None
        self.camera_pos_para = {}
        self.max_dis = min(self.im_shape) // 2
        self.name = 'ROS version 2 demo'
        
        # 存储GNSS配置（从配置文件读取，用于初始化）
        self.gnss_config = gnss_config

        self.num_cameras = len(self.camera_topics)
        self.bin_inf = [pd.DataFrame(columns=['ID', 'mmsi', 'timestamp', 'match']) for _ in range(self.num_cameras)]
        self.fus_trajectory = [pd.DataFrame() for _ in range(self.num_cameras)]
        # 初始化每个相机的最新处理图像为空白图像
        blank_image = np.zeros((self.im_shape[1], self.im_shape[0], 3), dtype=np.uint8)
        self.latest_processed_images = {
            self.camera_topics[i]: blank_image.copy() for i in range(self.num_cameras)
        }
        # ============ 多进程池核心部分（每摄像头一个进程） ===============
        self.mp_input_queues = [Queue(maxsize=10) for _ in range(self.num_cameras)]
        self.mp_output_queues = [Queue(maxsize=10) for _ in range(self.num_cameras)]
        self.workers = [
            Process(
                target=multi_proc_worker,
                args=(self.mp_input_queues[i], self.mp_output_queues[i], self.im_shape, self.t, self.max_dis, self.skip_interval, self.camera_type, self.yolo_type)
            )
            for i in range(self.num_cameras)
        ]
        for p in self.workers:
            p.daemon = True
            p.start()
        # ===========================================
        self.stitch_image_queue = []
        self.latest_stitch = None
        self.print_logger = False

        # 创建用于显示的图像窗口
        cv2.namedWindow(self.name, cv2.WINDOW_NORMAL)

        qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=3
        )

        # 订阅相机图像和对应的同步器
        self.camera_subscribers = []
        for topic in self.camera_topics:
            sub = Subscriber(self, Image, topic, qos_profile=qos_profile)
            self.camera_subscribers.append(sub)
        self.ts = ApproximateTimeSynchronizer(
            self.camera_subscribers,
            queue_size=self.sync_queue_size,
            slop=self.sync_slop
        )
        self.ts.registerCallback(self.synchronized_camera_callback)

        # 订阅AIS和GNSS数据
        self.aisbatch_subscriber = self.create_subscription(
            AisBatch,
            self.ais_batch_pub_topic,
            self.aisbatch_callback,
            10
        )
        self.gnss_subscriber = self.create_subscription(
            Gnss,
            self.gnss_pub_topic,
            self.gnss_callback,
            10
        )
        # 定时刷新显示窗口
        self.window_timer = self.create_timer(1/self.output_fps, self.refresh_window_callback)

        # 定时向外发布轨迹信息的发布器
        self.fus_trajectory_publisher = self.create_publisher(VisiableTraBatch, self.fus_trajectory_topic, qos_profile)
        
        # 定时发布轨迹（1秒一次）
        self.trajectory_publish_timer = self.create_timer(1.0, self.publish_trajectory_callback)


    # =================== 回调函数 ====================
    def synchronized_camera_callback(self, *msgs):
        # 收集输入数据封包推送进多进程队列
        # 检查相机位置参数是否已初始化（必需）
        if not self.camera_pos_para:
            self.get_logger().warning("等待GNSS数据更新相机位置参数，暂不处理图像帧", throttle_duration_sec=5.0)
            return
        
        # 检查AIS数据（非必需，仅警告）
        if self.aisbatch_cache.empty:
            self.get_logger().warning("未收到AIS数据，将仅使用视觉跟踪模式", throttle_duration_sec=5.0)
        # self.get_logger().info(f"收到同步相机帧，共{len(msgs)}帧")
        cv_images = [self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') for msg in msgs]
        current_timestamp = msgs[0].header.stamp.sec * 1000 + msgs[0].header.stamp.nanosec // 1000000
        for cam_idx in range(self.num_cameras):
            if cam_idx >= len(cv_images):
                continue

             # 检查 camera_pos_para 和 bin_inf 是否完整
            camera_ok = cam_idx in self.camera_pos_para and isinstance(self.camera_pos_para[cam_idx], dict)
            bin_ok = len(self.bin_inf) > cam_idx and isinstance(self.bin_inf[cam_idx], pd.DataFrame)

            if not camera_ok:
                self.get_logger().warning(f"camera_pos_para[{cam_idx}] 缺失或类型错误，内容：{self.camera_pos_para.get(cam_idx, None)}，跳过该帧")
                continue
            if not bin_ok:
                self.get_logger().warning(f"bin_inf[{cam_idx}] 缺失或类型错误，内容：{self.bin_inf[cam_idx] if len(self.bin_inf) > cam_idx else 'None'}，跳过该帧")
                continue


            # 提交任务给对应摄像头的进程
            try:
                self.mp_input_queues[cam_idx].put_nowait({
                    "cam_idx": cam_idx,
                    "cv_image": cv_images[cam_idx],
                    "current_timestamp": current_timestamp,
                    "ais_batch": self.aisbatch_cache,
                    "camera_pos_para": self.camera_pos_para[cam_idx],
                    "bin_inf": self.bin_inf[cam_idx] if len(self.bin_inf) > cam_idx else pd.DataFrame(),
                })
                # self.get_logger().info(f"提交 cam{cam_idx} 一帧到多进程队列")
            except Exception as e:
                self.get_logger().debug(f"多进程输入队列满，丢弃 cam{cam_idx} 当前帧: {e}")
                
    def aisbatch_callback(self, msg: AisBatch):
        # self.get_logger().info(f"收到AIS数据")  # 添加此行
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
        aisbatch_df = pd.DataFrame(data_list, columns=['mmsi','lon','lat','speed','course','heading','type','timestamp'])
        self.aisbatch_cache = aisbatch_df
        if len(self.aisbatch_cache) > 100:
            self.aisbatch_cache = self.aisbatch_cache.tail(100).copy()

    def gnss_callback(self, msg: Gnss):
        """GNSS回调函数：更新相机位置参数"""
        self.gnss_cache = msg
        
        # 更新相机位置信息，以中间相机为正前方
        # 左右相机的水平朝向分别调整-60和+60度
        for idx, topic in enumerate(self.camera_topics):
            # 计算水平朝向（中间相机不变，左右相机调整±N度,N为相机之间的夹角）
            horizontal_orientation = (msg.horizontal_orientation + (idx - 1) * self.angle_between_cameras) % 360
            # 开始混合赋值
            if self.camera_type == 'fisheye':
                self.camera_pos_para[idx] = {
                    "longitude": msg.longitude,
                    "latitude": msg.latitude,
                    "horizontal_orientation": horizontal_orientation,  # 每个相机的水平朝向由yaml文件配置
                    "vertical_orientation": msg.vertical_orientation,
                    "camera_height": msg.camera_height,

                    'fov_hor': self.camera_matrixs[idx]['fov_hor'], # 数据集模式下这参数是固定的
                    'fov_ver': self.camera_matrixs[idx]['fov_ver'],
                    'fx': self.camera_matrixs[idx]['fx'],
                    'fy': self.camera_matrixs[idx]['fy'],
                    'u0': self.camera_matrixs[idx]['u0'],
                    'v0': self.camera_matrixs[idx]['v0'],

                    'k1': self.distortion_coefficients[idx]['k1'],
                    'k2': self.distortion_coefficients[idx]['k2'],
                    'k3': self.distortion_coefficients[idx]['k3'],
                    'k4': self.distortion_coefficients[idx]['k4'],
                }
            elif self.camera_type == 'normal':
                self.camera_pos_para[idx] = {
                    "longitude": msg.longitude,
                    "latitude": msg.latitude,
                    "horizontal_orientation": horizontal_orientation,  # 每个相机的水平朝向由yaml文件配置
                    "vertical_orientation": msg.vertical_orientation,
                    "camera_height": msg.camera_height,

                    'fov_hor': self.camera_matrixs[idx]['fov_hor'], # 数据集模式下这参数是固定的
                    'fov_ver': self.camera_matrixs[idx]['fov_ver'],
                    'fx': self.camera_matrixs[idx]['fx'],
                    'fy': self.camera_matrixs[idx]['fy'],
                    'u0': self.camera_matrixs[idx]['u0'],
                    'v0': self.camera_matrixs[idx]['v0'],
                }
            

    def refresh_window_callback(self):
        # 回收多进程输出结果
        for cam_idx in range(self.num_cameras):
            try:
                while True:
                    result = self.mp_output_queues[cam_idx].get_nowait()
                    if "error" in result:
                        self.get_logger().error(f"子进程Camera {result['cam_idx']}出错: {result['error']}")
                        continue
                    cam_idx = result["cam_idx"]
                    self.fus_trajectory[cam_idx] = result["Visiable_Tra"]
                    self.fus_trajectory[cam_idx]['timestamp'] = result["timestamp"]
                    # time_cost = result.get("time_cost", 0)
                    # if time_cost is not None:
                    #     self.get_logger().info(f"Camera {cam_idx}  Time cost: {time_cost} seconds")
                    self.latest_processed_images[self.camera_topics[cam_idx]] = result["image"]
            except Exception:
                pass  # 当前相机队列空，继续下一个
        
        # 检查进程健康状态
        for i, p in enumerate(self.workers):
            if not p.is_alive():
                self.get_logger().error(f"Worker {i} (PID {p.pid}) 已死亡！")

        # 拼接图像并显示
        current_images = self.latest_processed_images.copy()
        
        # 验证所有图像是否存在且尺寸一致（避免hconcat错误）
        images_to_concat = []
        target_shape = None
        all_valid = True
        
        for cam_name in self.camera_topics:
            img = current_images.get(cam_name)
            if img is None:
                self.get_logger().debug(f"相机 {cam_name} 图像为None，跳过本次拼接")
                all_valid = False
                break
            
            # 检查图像尺寸和类型
            if target_shape is None:
                target_shape = img.shape
            elif img.shape != target_shape:
                self.get_logger().warning(
                    f"相机 {cam_name} 图像尺寸不匹配: {img.shape} != {target_shape}，"
                    f"将调整为目标尺寸后拼接"
                )
                # 调整图像尺寸以匹配目标
                img = cv2.resize(img, (target_shape[1], target_shape[0]))
            
            images_to_concat.append(img)
        
        # 只有当所有图像都有效时才进行拼接
        if all_valid and len(images_to_concat) == len(self.camera_topics):
            try:
                stitched_image = cv2.hconcat(images_to_concat)
                cv2.imshow(self.name, stitched_image)
                cv2.waitKey(1)
            except cv2.error as e:
                self.get_logger().error(f"图像拼接失败: {e}")
        else:
            self.get_logger().debug("等待所有相机图像就绪...")

    def publish_trajectory_callback(self):
        """定时发布融合轨迹（1秒一次）"""
        msg_batch = VisiableTraBatch()
        msg_batch.visiable_tra_list = []
        
        for cam_idx in range(self.num_cameras):
            if self.fus_trajectory[cam_idx].empty:
                continue  # 跳过空的轨迹数据
            
            # 使用第一个轨迹的时间戳作为batch的时间戳
            if not self.fus_trajectory[cam_idx].empty:
                first_tra = self.fus_trajectory[cam_idx].iloc[0]
                batch_timestamp_ms = int(first_tra.get('timestamp', 0))
                msg_batch.timestamp.sec = batch_timestamp_ms // 1000
                msg_batch.timestamp.nanosec = (batch_timestamp_ms % 1000) * 1000000

            for idx, tra in self.fus_trajectory[cam_idx].iterrows():
                try:
                    msg = VisiableTra()
                    # 使用映射后的相机名称（与C++拼接节点保持一致）
                    topic_name = self.camera_topics[cam_idx]
                    # =================================================== DEBUG ===================================================
                    msg.camera_name = self.camera_name_mapping.get(topic_name, topic_name) # 第一个topic_name作为键名，第二个是若没有对应的键值对，则返回原值
                    # =================================================== DEBUG ===================================================
                    # 将毫秒时间戳转换为 ROS Time (sec + nanosec)
                    timestamp_ms = int(tra.get('timestamp', 0))
                    msg.timestamp.sec = timestamp_ms // 1000
                    msg.timestamp.nanosec = (timestamp_ms % 1000) * 1000000
                    
                    # AIS相关字段，如果不存在则使用默认值
                    msg.ais = 0 if pd.isna(tra.get('ais')) or tra.get('ais') is None else int(tra['ais'])
                    
                    # 船只类型：从融合结果中获取class_name
                    class_name_value = tra.get('class_name')

                    if pd.isna(class_name_value) or class_name_value is None or class_name_value == '':
                        msg.ship_type = 'vessel'  # 默认类型
                    else:
                        msg.ship_type = str(class_name_value)
                    # mmsi 必须是无符号整数 [0, 4294967295]，负数转为0
                    mmsi_value = tra.get('mmsi', 0)
                    if pd.isna(mmsi_value) or mmsi_value < 0:
                        msg.mmsi = 0
                    else:
                        msg.mmsi = int(mmsi_value)
                    self.get_logger().info(f"最终 msg.mmsi: {msg.mmsi}, msg.ship_type: {msg.ship_type}")

                    # 速度和航向，负数转为0.0
                    sog_value = tra.get('sog', 0.0)
                    msg.sog = float(sog_value) if not pd.isna(sog_value) and sog_value >= 0 else 0.0
                    cog_value = tra.get('cog', 0.0)
                    msg.cog = float(cog_value) if not pd.isna(cog_value) and cog_value >= 0 else 0.0
                    
                    # 经纬度，负数可能是有效的（南纬、西经），只检查NaN
                    msg.lat = float(tra.get('lat', 0.0)) if not pd.isna(tra.get('lat')) else 0.0
                    msg.lon = float(tra.get('lon', 0.0)) if not pd.isna(tra.get('lon')) else 0.0
                    
                    # 视觉检测框坐标（必需字段）
                    msg.box_x1 = float(tra.get('box_x1', 0.0))
                    msg.box_y1 = float(tra.get('box_y1', 0.0))
                    msg.box_x2 = float(tra.get('box_x2', 0.0))
                    msg.box_y2 = float(tra.get('box_y2', 0.0))
                    
                    msg_batch.visiable_tra_list.append(msg)
                except Exception as e:
                    self.get_logger().warning(f"发布轨迹消息时出错: {e}")
        
        # 只有在有数据时才发布
        if len(msg_batch.visiable_tra_list) > 0:
            self.fus_trajectory_publisher.publish(msg_batch)
            # self.get_logger().info(f"发布了 {len(msg_batch.visiable_tra_list)} 个轨迹")

    def destroy_node(self):
        # 关闭所有多进程
        for q in self.mp_input_queues:
            q.put(None)
        for p in self.workers:
            p.join(timeout=5)
        super().destroy_node()

    def save_to_local(self, AIS_vis, AIS_cur, Vis_tra, Vis_tra_cur, Fus_tra, current_timestamp):
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

def main(args=None):
    try:
        set_start_method('spawn')
    except RuntimeError:
        pass
    rclpy.init(args=args)

    node = AisVisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
