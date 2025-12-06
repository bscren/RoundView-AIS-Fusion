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


class CameraPubNode(Node):
    def __init__(self):
        super().__init__("camera_publisher_node")
        self.declare_parameter('video_path', '/home/tl/RV/src/marnav_vis/clip-01/2022_06_04_12_05_12_12_07_02_b.mp4')
        self.declare_parameter('publish_fps', 25) # 发布帧率，单位Hz
        self.declare_parameter('width', 1280) # 图像宽度
        self.declare_parameter('height', 720) # 图像高度
        self.declare_parameter('start_timestamp', 1654315512000),  # AIS数据集起始时间戳，单位：毫秒
        self.declare_parameter('noise_range_ns ', 10 * 1000000),  # 时间戳噪音范围10ms，单位：纳秒 1 ms=1000000 ns

        # ============================================================
        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.publish_fps = self.get_parameter('publish_fps').get_parameter_value().integer_value
        self.get_logger().info(f"Publish FPS: {self.publish_fps}")
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.camera_microtimestamp = self.get_parameter('start_timestamp').get_parameter_value().integer_value
        self.noise_range_ns = self.get_parameter('noise_range_ns ').get_parameter_value().integer_value
        # ============================================================

        self.get_logger().info(f"Video path: {self.video_path}, Publish FPS: {self.publish_fps}, Width: {self.width}, Height: {self.height}, Start in Time: {self.camera_microtimestamp}")

        self.cap = cv2.VideoCapture(self.video_path)

        # 获取视频的原始帧率和尺寸
        self.video_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.video_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.video_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
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
            '/camera_image_topic_0', 
            # 10
            qos_profile
            )
        self.image_publisher_1 = self.create_publisher(
            Image, 
            '/camera_image_topic_1', 
            # 10
            qos_profile
            )
        self.image_publisher_2 = self.create_publisher(
            Image, 
            '/camera_image_topic_2', 
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
        frame = cv2.resize(frame, (self.width, self.height))
        
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
