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

# CPU密集型任务，使用多进程处理，而不是多线程处理，多线程适用于IO密集型任务
from multiprocessing import Process, Queue, cpu_count, set_start_method
import multiprocessing

from utils.file_read import read_all, ais_initial, update_time, time2stamp
from utils.VIS_utils import VISPRO
from utils.AIS_utils import AISPRO
from utils.FUS_utils import FUSPRO
from utils.gen_result import gen_result
from utils.draw import DRAW

# 由v4的多线程改为v5的多进程，每个进程独立处理一帧: 包含AIS, VIS, FUS, DRAW

# ------------------------------------------------------
def multi_proc_worker(input_queue, output_queue, im_shape, t, camera_para, max_dis):
    """每个进程独立处理一帧: 包含AIS, VIS, FUS, DRAW"""
    aispro = AISPRO(im_shape, t)
    vispro = VISPRO(1, 0, t)  # 是否读取anti参数可以自定义
    fuspro = FUSPRO(max_dis, im_shape, t)
    dra = DRAW(im_shape, t)
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
            gnss_cache = task["gnss_cache"]
            camera_pos = task["camera_pos"]
            bin_info = task["bin_info"]
            ais_skip_ratio = 1
            vis_skip_ratio = 1
            fus_skip_ratio = 1
            # 1. AIS
            AIS_vis, AIS_cur = aispro.process(ais_batch, camera_pos, current_timestamp, skip_ratio=ais_skip_ratio)
            # 2. VIS
            Vis_tra, Vis_cur = vispro.feedCap(cv_image, current_timestamp, AIS_vis, bin_info, skip_ratio=vis_skip_ratio)
            # 3. FUS
            Fus_tra, updated_bin = fuspro.fusion(AIS_vis, AIS_cur, Vis_tra, Vis_cur, current_timestamp, skip_ratio=fus_skip_ratio)
            # 4. DRAW
            im = dra.draw_traj(cv_image, AIS_vis, AIS_cur, Vis_tra, Vis_cur, Fus_tra, timestamp=current_timestamp,  skip_ratio = 4)
            end_time = time.time()
            time_cost = end_time - start_time
            result = {
                "cam_idx": cam_idx, 
                "image": im, 
                "timestamp": current_timestamp, 
                "updated_bin": updated_bin,
                "time_cost": time_cost
            }
            output_queue.put(result)
        except Exception as e:
            output_queue.put({"cam_idx": task.get("cam_idx",-1), "error": str(e)})
    
    
# ------------------------------------------------------

class AisVisNode(Node):
    def __init__(self):
        super().__init__('ais_vis_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ais_start_timestamp', 1654315512000),
                ('width_height', [2560, 1440]),
                ('camera_topics', ['/camera_image_topic_1', '/camera_image_topic_2', '/camera_image_topic_3']),
                ('aisbatch_topic', 'ais_batch_topic'),
                ('gnss_topic', 'gnss_topic'),
                ('input_fps', 25),
                ('output_fps', 10),
                ('camera_para', [56.0, 30.94, 2391.26, 2446.89, 1305.04, 855.214]),
                ('anti', 1),
                ('anti_rate', 0),
                ('sync_queue_size', 10),
                ('sync_slop', 0.1)
            ]
        )

        self.ais_start_timestamp = self.get_parameter('ais_start_timestamp').get_parameter_value().integer_value
        self.im_shape = tuple[int, ...](self.get_parameter('width_height').get_parameter_value().integer_array_value)
        self.camera_topics = self.get_parameter('camera_topics').get_parameter_value().string_array_value
        self.aisbatch_topic = self.get_parameter('aisbatch_topic').get_parameter_value().string_value
        self.gnss_topic = self.get_parameter('gnss_topic').get_parameter_value().string_value
        self.input_fps = self.get_parameter('input_fps').get_parameter_value().integer_value
        self.t = int(1000 / self.input_fps)
        self.output_fps = self.get_parameter('output_fps').get_parameter_value().integer_value
        self.camera_para = list(self.get_parameter('camera_para').get_parameter_value().double_array_value)
        self.anti = self.get_parameter('anti').get_parameter_value().integer_value
        self.anti_rate = self.get_parameter('anti_rate').get_parameter_value().integer_value
        self.sync_queue_size = self.get_parameter('sync_queue_size').get_parameter_value().integer_value
        self.sync_slop = self.get_parameter('sync_slop').get_parameter_value().double_value

        now_timestamp = self.get_clock().now().to_msg().sec * 1000 + self.get_clock().now().to_msg().nanosec // 1000000
        self.time_offset = now_timestamp - self.ais_start_timestamp
        self.get_logger().info(f"Current timestamp: {now_timestamp}, AIS start timestamp: {self.ais_start_timestamp}, Time offset: {self.time_offset} ms\n")

        self.bridge = CvBridge()
        self.aisbatch_cache = pd.DataFrame(columns=['ID', 'mmsi', 'timestamp', 'lat', 'lon', 'sog', 'cog', 'heading', 'status', 'type'])
        self.aisbatch_time = None
        self.gnss_cache = None
        self.camera_pos_para = None
        self.max_dis = min(self.im_shape) // 2
        self.name = 'ROS version 2 demo'

        self.num_cameras = len(self.camera_topics)
        self.bin_inf = [pd.DataFrame(columns=['ID', 'mmsi', 'timestamp', 'match']) for _ in range(self.num_cameras)]

        self.latest_processed_images = {
            f'cam{i}': None for i in range(self.num_cameras)
        }
        # ============ 多进程池核心部分 ===============
        self.mp_input_queue = Queue(maxsize=self.num_cameras * 10)
        self.mp_output_queue = Queue(maxsize=self.num_cameras * 10)
        self.num_workers = min(self.num_cameras * 2, cpu_count())
        self.workers = [
            Process(
                target=multi_proc_worker,
                args=(self.mp_input_queue, self.mp_output_queue, self.im_shape, self.t, self.camera_para, self.max_dis)
            )
            for _ in range(self.num_workers)
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

        self.camera_subscribers = []
        for topic in self.get_parameter('camera_topics').get_parameter_value().string_array_value:
            sub = Subscriber(self, Image, topic, qos_profile=qos_profile)
            self.camera_subscribers.append(sub)
        self.ts = ApproximateTimeSynchronizer(
            self.camera_subscribers,
            queue_size=self.sync_queue_size,
            slop=self.sync_slop
        )
        self.ts.registerCallback(self.synchronized_camera_callback)

        self.aisbatch_subscriber = self.create_subscription(
            AisBatch,
            self.aisbatch_topic,
            self.aisbatch_callback,
            10
        )
        self.gnss_subscriber = self.create_subscription(
            Gnss,
            self.gnss_topic,
            self.gnss_callback,
            10
        )
        self.window_timer = self.create_timer(1/self.output_fps, self.refresh_window_callback)

        self.get_logger().info("\nVesselSORT_ROS")
        self.get_logger().info(f"Camera topics: {self.camera_topics}")
        self.get_logger().info(f"AIS batch topic: {self.aisbatch_topic}")
        self.get_logger().info(f"GNSS topic: {self.gnss_topic}")

    # =================== 回调函数 ====================
    def synchronized_camera_callback(self, *msgs):
        # 收集输入数据封包推送进多进程队列
        if self.aisbatch_cache.empty or self.gnss_cache is None or self.camera_pos_para is None:
            return
        cv_images = []
        for msg in msgs:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_images.append(cv_image)
        current_timestamp = msgs[0].header.stamp.sec * 1000 + msgs[0].header.stamp.nanosec // 1000000
        for cam_idx in range(self.num_cameras):
            if cam_idx >= len(cv_images):
                continue
            # 提交任务给进程池
            if not self.mp_input_queue.full():
                self.mp_input_queue.put({
                    "cam_idx": cam_idx,
                    "cv_image": cv_images[cam_idx],
                    "current_timestamp": current_timestamp,
                    "ais_batch": self.aisbatch_cache.copy(),
                    "gnss_cache": self.gnss_cache,
                    "camera_pos": self.camera_pos_para,
                    "bin_info": self.bin_inf[cam_idx].copy() if len(self.bin_inf) > cam_idx else pd.DataFrame(),
                })
            else:
                self.get_logger().warn(f"多进程输入队列满，丢弃 cam{cam_idx} 一帧")

    def aisbatch_callback(self, msg: AisBatch):
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
        self.gnss_cache = msg
        self.camera_pos_para = [msg.longitude, msg.latitude, msg.horizontal_orientation, msg.vertical_orientation, msg.camera_height] + self.camera_para

    def refresh_window_callback(self):
        # 回收多进程输出结果
        try:
            while True:
                result = self.mp_output_queue.get_nowait()
                if "error" in result:
                    self.get_logger().error(f"子进程Camera {result['cam_idx']}出错: {result['error']}")
                    continue
                cam_idx = result["cam_idx"]
                time_cost = result.get("time_cost", 0)
                if time_cost is not None:
                    self.get_logger().info(f"Camera {cam_idx}  Time cost: {time_cost} seconds")
                im = result["image"]
                updated_bin = result.get("updated_bin", None)
                self.latest_processed_images[f'cam{cam_idx}'] = im
                if updated_bin is not None:
                    if cam_idx < len(self.bin_inf):
                        self.bin_inf[cam_idx] = updated_bin
                    else:
                        self.bin_inf.extend([pd.DataFrame()]*(cam_idx - len(self.bin_inf) + 1))
                        self.bin_inf[cam_idx] = updated_bin
        except Exception:
            pass  # 队列空，忽略
        # 拼接图像并显示
        current_images = self.latest_processed_images.copy()
        for cam_name, img in current_images.items():
            if img is None:
                return
        stitched_image = cv2.hconcat([current_images[f'cam{idx}'] for idx in range(self.num_cameras)])
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        cv2.putText(
            stitched_image,
            f"Time: {current_time}",
            (50, 500),
            cv2.FONT_HERSHEY_SIMPLEX,
            3,
            (0, 0, 255),
            2
        )
        cv2.imshow(self.name, stitched_image)
        cv2.waitKey(1)

    def destroy_node(self):
        # 关闭所有多进程
        for _ in self.workers:
            self.mp_input_queue.put(None)
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
