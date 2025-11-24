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
        self.image_publisher_1 = self.create_publisher(
            Image, 
            'camera_image_topic_1', 
            # 10
            qos_profile
            )
        self.image_publisher_2 = self.create_publisher(
            Image, 
            'camera_image_topic_2', 
            # 10
            qos_profile
            )
        self.image_publisher_3 = self.create_publisher(
            Image, 
            'camera_image_topic_3', 
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
        # current_time = self.get_clock().now()
        # interval = (current_time - self.last_trigger_time).nanoseconds / 1e9  # 转换为秒
        # self.get_logger().info(f"实际间隔: {interval*1000:.2f}ms")
        # self.last_trigger_time = current_time

        # ============================================================
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video file reached or failed to read frame.")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # 重新从头开始播放
            return
        # 切割图像为三等分
        # height, width, _ = frame.shape
        # third_width = width // 3
        # frame_1 = frame[0:height, 0:third_width]
        # frame_2 = frame[0:height, third_width:2*third_width]
        # frame_3 = frame[0:height, 2*third_width:width]
        # frame = cv2.resize(frame, (self.width, self.height))
        frame_1 = frame
        frame_2 = frame
        frame_3 = frame

        # 发布图像消息
        # 2. 为每个相机生成独立的随机噪音（整数，单位：纳秒）
        #    范围：[-noise_range_ns, noise_range_ns]
        noise_1 = np.random.randint(-self.noise_range_ns, self.noise_range_ns + 1)
        noise_2 = np.random.randint(-self.noise_range_ns, self.noise_range_ns + 1)
        noise_3 = np.random.randint(-self.noise_range_ns, self.noise_range_ns + 1)

        # 3. 计算原始时间戳的总纳秒数（self.camera_microtimestamp是毫秒，需转纳秒）
        original_total_ns = int(self.camera_microtimestamp * 1000000)  # 1毫秒 = 1e6纳秒

        # 4. 为每个相机添加噪音，得到带抖动的总纳秒数
        total_ns_1 = original_total_ns + noise_1
        total_ns_2 = original_total_ns + noise_2
        total_ns_3 = original_total_ns + noise_3

        image_msg_1 = self.bridge.cv2_to_imgmsg(frame_1, encoding='bgr8')
        image_msg_2 = self.bridge.cv2_to_imgmsg(frame_2, encoding='bgr8')
        image_msg_3 = self.bridge.cv2_to_imgmsg(frame_3, encoding='bgr8')
        # image_msg.header.stamp = self.get_clock().now().to_msg()
        # 给时间戳添加噪音，模仿真实相机时间戳的抖动情况
        sec_1, nanosec_1 = self.split_ns(total_ns_1)
        image_msg_1.header.stamp.sec = sec_1
        image_msg_1.header.stamp.nanosec = nanosec_1

        sec_2, nanosec_2 = self.split_ns(total_ns_2)
        image_msg_2.header.stamp.sec = sec_2
        image_msg_2.header.stamp.nanosec = nanosec_2

        sec_3, nanosec_3 = self.split_ns(total_ns_3)
        image_msg_3.header.stamp.sec = sec_3
        image_msg_3.header.stamp.nanosec = nanosec_3

        self.camera_microtimestamp += self.t  # 增加相应的毫秒数
        
        self.image_publisher_1.publish(image_msg_1)
        self.image_publisher_2.publish(image_msg_2)
        self.image_publisher_3.publish(image_msg_3)
        # self.get_logger().info("Published a frame.")
        # self.get_logger().info(f"Published a frame. Callback耗时: {(time.time()-start_time)*1000:.2f}ms")  # 打印耗时



def main(args=None):
    rclpy.init(args=args)
    camera_pub_node = CameraPubNode()
    rclpy.spin(camera_pub_node)
    camera_pub_node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()  
