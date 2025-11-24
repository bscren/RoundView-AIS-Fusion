# 读取 camera_image_topic 话题中的图像并显示
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import cv2
import os
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class CameraSubNode(Node):
    def __init__(self):
        super().__init__("camera_sub_node")
        self.declare_parameter('window_name', 'Camera Image')
        self.window_name = self.get_parameter('window_name').get_parameter_value().string_value
        self.get_logger().info(f"Window name: {self.window_name}")

        self.bridge = CvBridge()
        qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=3
        )
        self.image_subscriber = self.create_subscription(Image, 'camera_image_topic', self.image_callback, qos_profile)

    def image_callback(self, msg):
        # get .header.stamp
        # timestamp = msg.header.stamp
        # self.get_logger().info(f"Received image with timestamp: sec={timestamp.sec}, nanosec={timestamp.nanosec}")
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(1) # 必须调用以刷新图像窗口 1 means 1 millisecond

def main(args=None):
    rclpy.init(args=args)
    camera_sub_node = CameraSubNode()
    rclpy.spin(camera_sub_node)
    camera_sub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
