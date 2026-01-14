# 用于制作模仿实际场景多相机发布的话题
# 读取指定路径的视频，将视频纵向分割成三等分，然后按照指定的帧率逐帧发布
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import cv2
import os
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from marnav_vis.config_loader import ConfigLoader
from ament_index_python.packages import get_package_share_directory
import os


class CameraPubNode(Node):
    def __init__(self):
        super().__init__("camera_publisher_node")
        
        # 声明配置文件参数
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        # 如果未指定配置文件，使用默认路径
        if not config_file:
            try:
                config_file = ConfigLoader.find_config_file('marnav_vis', 'track_offline_config.yaml')
                self.get_logger().info(f"未指定配置文件，使用默认路径: {config_file}")
            except Exception as e:
                self.get_logger().error(f"查找默认配置文件失败: {e}")
                raise
        
        # 加载配置
        try:
            config_loader = ConfigLoader(config_file)
            camera_config = config_loader.get_camera_config()
        except Exception as e:
            self.get_logger().fatal(f"加载配置文件失败: {e}")
            raise
        
        # 从配置中读取参数
        self.video_path = camera_config.get('video_path', '')
        self.publish_fps = camera_config.get('camera_publish_fps', 25)
        self.width_height = camera_config.get('width_height', [1280, 720])
        self.camera_microtimestamp = camera_config.get('camera_start_timestamp', 0)
        self.noise_range_ns = camera_config.get('noise_range_ns', 10000000)
        
        # video_path 添加动态链接
        if not os.path.isabs(self.video_path):
            current_file = os.path.abspath(__file__)
            # 从当前文件路径向上遍历，找到包含Datasets和src的目录（RV根）
            workspace_root = None
            current_dir = os.path.dirname(current_file)
            # 最多向上遍历10层，避免死循环
            for _ in range(10):
                # 检查当前目录是否是RV根（有Datasets和src文件夹）
                if os.path.exists(os.path.join(current_dir, 'Datasets')) and os.path.exists(os.path.join(current_dir, 'src')):
                    workspace_root = current_dir
                    break
                current_dir = os.path.dirname(current_dir)
            
            if not workspace_root:
                self.get_logger().error("❌ 无法找到RV工作空间根目录！")
                raise RuntimeError("工作空间根目录定位失败")
            
            self.video_path = os.path.join(workspace_root, self.video_path)

        # 提取相机话题列表
        self.camera_parameters = camera_config.get('camera_parameters',[])
        self.camera_topics = [cam['topic_name'] for cam in self.camera_parameters]
        if not self.camera_topics:
            self.get_logger().fatal("配置文件中未定义相机话题")
            raise ValueError("未定义相机话题")
        
        self.get_logger().info("="*60)
        self.get_logger().info("📹 相机发布节点配置")
        self.get_logger().info("="*60)
        self.get_logger().info(f"配置文件: {config_file}")
        self.get_logger().info(f"视频路径: {self.video_path}")
        self.get_logger().info(f"发布频率: {self.publish_fps} Hz")
        self.get_logger().info(f"图像尺寸: {self.width_height[0]}x{self.width_height[1]}")
        self.get_logger().info(f"起始时间戳: {self.camera_microtimestamp} ms")
        self.get_logger().info(f"时间戳噪声: ±{self.noise_range_ns/1000000:.1f} ms")
        self.get_logger().info(f"相机话题数量: {len(self.camera_topics)}")
        for topic in self.camera_topics:
            self.get_logger().info(f"  - {topic}")
        self.get_logger().info("="*60)

        self.cap = cv2.VideoCapture(self.video_path)
        # 


        # 获取视频的原始帧率和尺寸
        self.video_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.video_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.video_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        # 检测是否为空
        if self.video_fps == 0 or self.video_width == 0 or self.video_height == 0:
            self.get_logger().error(f"无法获取视频信息，请检查视频文件是否有效: {self.video_path}")
            return
        self.t = 1000/self.video_fps # 毫秒每帧
        self.get_logger().info(f"Original video FPS: {self.video_fps}, Width: {self.video_width}, Height: {self.video_height}, Frame interval: {self.t} ms")

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video file: {self.video_path}")
            return
        self.bridge = CvBridge()

        # 设定发布者
        # 若消息发布存在队列阻塞，可适当降低 QoS 的可靠性要求（例如使用 “最佳 - effort” 模式）或减小队列深度，减少消息处理开销
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # 非可靠传输（适合图像等实时数据）
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5  # 减小队列深度
        )
        self.image_publisher_0 = self.create_publisher(
            Image, 
            self.camera_topics[0], 
            # 10
            qos_profile
            )
        self.image_publisher_1 = self.create_publisher(
            Image, 
            self.camera_topics[1], 
            # 10
            qos_profile
            )
        self.image_publisher_2 = self.create_publisher(
            Image, 
            self.camera_topics[2], 
            # 10
            qos_profile
            )
        self.timer = self.create_timer(1.0 / self.publish_fps, self.timer_callback)

        # 记录触发时间
        # self.last_trigger_time = self.get_clock().now()

    # 将总纳秒数拆分为sec（秒）和nanosec（纳秒），确保nanosec在0~1e9
    def split_ns(self, total_ns):
        sec = total_ns // 1000000000  # 1秒 = 1e9纳秒
        nanosec = total_ns % 1000000000
        # 处理负数情况（若total_ns为负，nanosec需转为正数）
        if nanosec < 0:
            sec -= 1
            nanosec += 1000000000
        return int(sec), int(nanosec)

    def timer_callback(self):
        # 记录开始时间用于计算回调耗时
        start_time = time.time()
        
        # ============================================================
        # 1. 读取视频帧
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video file reached or failed to read frame.")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # 重新从头开始播放
            return
        
        # 2. 降低分辨率以提高性能（2560x1440 太大，降到 1280x720）
        #    如果需要更高性能，可以降到 640x360
        frame = cv2.resize(frame, (self.width_height[0], self.width_height[1]))
        
        # 3. 计算时间戳（一次性计算）
        original_total_ns = int(self.camera_microtimestamp * 1000000)  # 毫秒转纳秒
        
        # 4. 生成随机噪音（批量生成）
        noises = np.random.randint(-self.noise_range_ns, self.noise_range_ns + 1, size=3)
        
        # 5. 为3个相机分别创建消息并发布（避免数据共享）
        #    注意：不能共享 data，否则会导致消息丢失
        for i in range(3):
            # 为每个相机独立转换图像
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            
            # 设置独立的时间戳（加噪音）
            total_ns = original_total_ns + int(noises[i])
            sec, nanosec = self.split_ns(total_ns)
            img_msg.header.stamp.sec = sec
            img_msg.header.stamp.nanosec = nanosec
            
            # 立即发布（避免累积消息）
            if i == 0:
                self.image_publisher_0.publish(img_msg)
            elif i == 1:
                self.image_publisher_1.publish(img_msg)
            else:
                self.image_publisher_2.publish(img_msg)
        
        # 6. 更新时间戳
        self.camera_microtimestamp += self.t
        
        # 统计回调耗时
        callback_time = (time.time() - start_time) * 1000  # 转换为毫秒
        # if callback_time > 40:  # 如果超过理论间隔（40ms for 25fps），打印警告
        #     self.get_logger().warn(f"Callback耗时过长: {callback_time:.2f}ms (期望<40ms)")
        # else:
        #     self.get_logger().info(f"Callback耗时: {callback_time:.2f}ms")



def main(args=None):
    rclpy.init(args=args)
    camera_pub_node = CameraPubNode()
    rclpy.spin(camera_pub_node)
    camera_pub_node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()  
