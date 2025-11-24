# 读取指定路径的视频，按照指定的帧率逐帧发布
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import cv2
import os
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class CameraPubNode(Node):
    def __init__(self):
        super().__init__("camera_publisher_node")
        self.declare_parameter('video_path', '/home/tl/RV/src/marnav_vis/clip-01/2022_06_04_12_05_12_12_07_02_b.mp4')
        self.declare_parameter('publish_fps', 25) # 发布帧率，单位Hz
        self.declare_parameter('width', 1280) # 图像宽度
        self.declare_parameter('height', 720) # 图像高度
        self.declare_parameter('start_timestamp', 1654315512000),  # AIS数据集起始时间戳，单位：毫秒

        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.publish_fps = self.get_parameter('publish_fps').get_parameter_value().integer_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.camera_microtimestamp = self.get_parameter('start_timestamp').get_parameter_value().integer_value

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
        self.image_publisher = self.create_publisher(
            Image, 
            'camera_image_topic', 
            # 10
            qos_profile
            )
        self.timer = self.create_timer(1.0 / self.publish_fps, self.timer_callback)

        # 记录触发时间
        # self.last_trigger_time = self.get_clock().now()


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
        # 调整图像大小
        # frame = cv2.resize(frame, (self.width, self.height))

        # 发布图像消息
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        # image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.stamp.sec = int(self.camera_microtimestamp // 1000)  # 秒部分
        image_msg.header.stamp.nanosec = int((self.camera_microtimestamp % 1000) * 1000000)  # 毫秒转纳秒
        self.camera_microtimestamp += self.t  # 增加相应的毫秒数
        image_msg.header.frame_id = "camera_frame"
        
        self.image_publisher.publish(image_msg)
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
