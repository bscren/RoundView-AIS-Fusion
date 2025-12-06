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
from sensor_msgs.msg import Image
from marnav_interfaces.msg import AisBatch, Gnss
from detect_interfaces.srv import GetCameraParams

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

# 由v4的多线程改为v5的多进程，每个进程独立处理一帧: 包含AIS, VIS, FUS, DRAW

# ------------------------------------------------------
def multi_proc_worker(input_queue, output_queue, im_shape, t, max_dis, skip_interval = 1):
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
                        'aspect': resp.aspect,
                        'ppx': resp.ppx,
                        'ppy': resp.ppy,
                        'R': list(resp.rotate_matrix),
                        't': list(resp.transport_matrix),
                        'K': list(resp.k_matrix)
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
    vispro = VISPRO(1, 0, t)  # 是否读取anti参数可以自定义
    fuspro = FUSPRO(max_dis, im_shape, t)
    dra = DRAW(im_shape, t)

    # 缓存上一帧的结果
    last_updated_bin = pd.DataFrame()
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
                AIS_vis, AIS_cur = aispro.process(ais_batch, camera_pos_para, current_timestamp)
                # 2. VIS
                Vis_tra, Vis_cur = vispro.feedCap(cv_image, AIS_vis, bin_inf, current_timestamp)
                # 3. FUS
                Fus_tra, updated_bin = fuspro.fusion(AIS_vis, AIS_cur, Vis_tra, Vis_cur, current_timestamp)
                # 4. DRAW
                im = dra.draw_match_traj(cv_image, AIS_vis, AIS_cur, Vis_tra, Vis_cur, Fus_tra, timestamp=current_timestamp)
                # 更新配对关系
                last_updated_bin = updated_bin
                # 更新窗口标记
                last_processed_window = current_window
            else:
                # print(f"no process_ais_vis_fus at timestamp {current_timestamp}, worker {cam_idx} processing task, pid={os.getpid()}")
                im = dra.draw_no_match_traj(cv_image)
                updated_bin = last_updated_bin
            
            end_time = time.time()
            time_cost = end_time - start_time
            result = {
                "cam_idx": cam_idx, 
                "image": im, 
                # "image": cv_image,
                "timestamp": current_timestamp, 
                "updated_bin": updated_bin,
                # "updated_bin": pd.DataFrame(),
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



# 作为客户端请求相机参数
def get_camera_params_client(node, camera_name):
    # node.get_logger().info(f'请求相机参数: {camera_name}')
    client = node.create_client(GetCameraParams, '/get_camera_params')
    timeout_sec = 10.0
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().error(f'已经过了{timeout_sec}秒，服务 /get_camera_params 不可用')
        return None
    request = GetCameraParams.Request()
    request.camera_name = camera_name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None and future.result().success:
        resp = future.result()
        # node.get_logger().info(
        #     f"相机参数: focal={resp.focal}, aspect={resp.aspect}, ppx={resp.ppx}, ppy={resp.ppy}\n"
        #     f"R={list(resp.rotate_matrix)}, t={list(resp.transport_matrix)}, K={list(resp.k_matrix)}"
        # )
        node.get_logger().info(f'获取相机参数成功: {camera_name}')
        return resp
    else:
        node.get_logger().error('获取相机参数失败')
        return None



class AisVisNode(Node):
    def __init__(self, camera_para):
        super().__init__('ais_vis_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('width_height', [2560, 1440]),
                ('camera_topics', ['/camera_image_topic_0', '/camera_image_topic_1', '/camera_image_topic_2']),
                # ('camera_topics', ['/rtsp_image_0', '/rtsp_image_1', '/rtsp_image_2']),
                ('aisbatch_topic', '/ais_batch_topic'),
                ('gnss_topic', '/gnss_topic'),
                ('input_fps', 15),
                ('output_fps', 10),
                ('anti', 1),
                ('anti_rate', 0),
                ('sync_queue_size', 10),
                ('sync_slop', 0.1),
                ('skip_interval', 1000) # 单位ms，表示每隔skip_interval ms处理一帧
            ]
        )
        self.camera_para = camera_para
        self.im_shape = tuple(map(int, self.get_parameter('width_height').get_parameter_value().integer_array_value))
        self.camera_topics = self.get_parameter('camera_topics').get_parameter_value().string_array_value
        self.aisbatch_topic = self.get_parameter('aisbatch_topic').get_parameter_value().string_value
        self.gnss_topic = self.get_parameter('gnss_topic').get_parameter_value().string_value
        self.input_fps = self.get_parameter('input_fps').get_parameter_value().integer_value
        self.t = int(1000 / self.input_fps)
        self.output_fps = self.get_parameter('output_fps').get_parameter_value().integer_value

        self.anti = self.get_parameter('anti').get_parameter_value().integer_value
        self.anti_rate = self.get_parameter('anti_rate').get_parameter_value().integer_value
        self.sync_queue_size = self.get_parameter('sync_queue_size').get_parameter_value().integer_value
        self.sync_slop = self.get_parameter('sync_slop').get_parameter_value().double_value
        self.skip_interval = self.get_parameter('skip_interval').get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.aisbatch_cache = pd.DataFrame(columns=['ID', 'mmsi', 'timestamp', 'lat', 'lon', 'sog', 'cog', 'heading', 'status', 'type'])
        self.aisbatch_time = None
        # self.gnss_cache = None
        self.camera_pos_para = {}
        self.max_dis = min(self.im_shape) // 2
        self.name = 'ROS version 2 demo'

        self.num_cameras = len(self.camera_topics)
        self.bin_inf = [pd.DataFrame(columns=['ID', 'mmsi', 'timestamp', 'match']) for _ in range(self.num_cameras)]
        self.latest_processed_images = {
            f'cam{i}': None for i in range(self.num_cameras)
        }
        # ============ 多进程池核心部分（每摄像头一个进程） ===============
        self.mp_input_queues = [Queue(maxsize=10) for _ in range(self.num_cameras)]
        self.mp_output_queues = [Queue(maxsize=10) for _ in range(self.num_cameras)]
        self.workers = [
            Process(
                target=multi_proc_worker,
                args=(self.mp_input_queues[i], self.mp_output_queues[i], self.im_shape, self.t, self.max_dis, self.skip_interval)
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

        # 订阅AIS和GNSS数据
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
        # 定时刷新显示窗口
        self.window_timer = self.create_timer(1/self.output_fps, self.refresh_window_callback)


    # =================== 回调函数 ====================
    def synchronized_camera_callback(self, *msgs):
        # 收集输入数据封包推送进多进程队列
        if self.aisbatch_cache.empty or not self.camera_pos_para:
            self.get_logger().info("等待AIS数据或GNSS数据更新相机位置参数，暂不处理图像帧")
            return
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
        # self.get_logger().info("收到GNSS数据")
        self.gnss_cache = msg
        # 更新相机位置信息，以相机camera_image_topic_1为正前方
        # 因此这里假设camera_image_topic_1是中间相机，camera_image_topic_0和camera_image_topic_2的水平朝向分别调整-60和+60度
        for idx, cam_topic in enumerate(self.camera_topics):
            horizontal_orientation = (msg.horizontal_orientation + (idx - 1) * 60) % 360  # 中间相机不变，左右相机调整±60度，若超出360度范围，进行归一化
            # self.camera_pos_para[idx] = {
            #     "longitude": msg.longitude,
            #     "latitude": msg.latitude,
            #     "horizontal_orientation": horizontal_orientation,
            #     "vertical_orientation": msg.vertical_orientation,
            #     "camera_height": msg.camera_height,
            #     'fov_hor': self.camera_para.get(idx, {}).get('fov_hor', None),
            #     'fov_ver': self.camera_para.get(idx, {}).get('fov_ver', None),
            #     'focal': self.camera_para.get(idx, {}).get('focal', None),
            #     'aspect': self.camera_para.get(idx, {}).get('aspect', None),
            #     'ppx': self.camera_para.get(idx, {}).get('ppx', None),
            #     'ppy': self.camera_para.get(idx, {}).get('ppy', None),
            #     'R': self.camera_para.get(idx, {}).get('R', None),
            #     't': self.camera_para.get(idx, {}).get('t', None),
            #     'K': self.camera_para.get(idx, {}).get('K', None)
            #     # "intrinsics": self.camera_para.get(cam_topic, {})
            # }
            # DEBUG 
            # Lon	Lat	Horizontal Orientation	Vertical Orientation	Camera Height	Horizontal FoV	Vertical FoV	fx	fy	u0	v0
            # 114.32583	30.60139	7	-1	20	55	30.94	2391.26	2446.89	1305.04	855.214
            self.camera_pos_para[idx] = {
                "longitude": 114.32583,
                "latitude": 30.60139,
                "horizontal_orientation": 7,
                "vertical_orientation": -1,
                "camera_height": 20,
                'fov_hor': 55,
                'fov_ver': 30.94,
                'fx': 2391.26,
                'fy': 2446.89,
                'u0': 1305.04,
                'v0': 855.214,
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
                    time_cost = result.get("time_cost", 0)
                    # if time_cost is not None:
                    #     self.get_logger().info(f"Camera {cam_idx}  Time cost: {time_cost} seconds")
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
        
        for idx in range(self.num_cameras):
            cam_key = f'cam{idx}'
            img = current_images.get(cam_key)
            if img is None:
                self.get_logger().debug(f"相机 {cam_key} 图像为None，跳过本次拼接")
                all_valid = False
                break
            
            # 检查图像尺寸和类型
            if target_shape is None:
                target_shape = img.shape
            elif img.shape != target_shape:
                self.get_logger().warning(
                    f"相机 {cam_key} 图像尺寸不匹配: {img.shape} != {target_shape}，"
                    f"将调整为目标尺寸后拼接"
                )
                # 调整图像尺寸以匹配目标
                img = cv2.resize(img, (target_shape[1], target_shape[0]))
            
            images_to_concat.append(img)
        
        # 只有当所有图像都有效时才进行拼接
        if all_valid and len(images_to_concat) == self.num_cameras:
            try:
                stitched_image = cv2.hconcat(images_to_concat)
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
            except cv2.error as e:
                self.get_logger().error(f"图像拼接失败: {e}")
        else:
            self.get_logger().debug("等待所有相机图像就绪...")

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
    # 先用临时node获取所有相机参数
    tmp_node = rclpy.create_node('tmp_param_client')
    camera_para = {}
    # 这里假设参数声明和读取与AisVisNode一致
    camera_topics = tmp_node.declare_parameter('camera_topics', ['rtsp_image_0', 'rtsp_image_1', 'rtsp_image_2']).get_parameter_value().string_array_value
    for idx, cam_name in enumerate(camera_topics):
        resp = get_camera_params_client(tmp_node, cam_name)
        if resp:
            camera_para[idx] = {
                'fov_hor': resp.fov_hor,
                'fov_ver': resp.fov_ver,
                'focal': resp.focal,
                'aspect': resp.aspect,
                'ppx': resp.ppx,
                'ppy': resp.ppy,
                'R': list(resp.rotate_matrix),
                't': list(resp.transport_matrix),
                'K': list(resp.k_matrix)
            }
    tmp_node.destroy_node()
    node = AisVisNode(camera_para)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
