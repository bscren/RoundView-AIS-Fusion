# 能够处理来自单个相机的图像数据，并使用YOLO模型进行物体检测
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')
        
        # 加载YOLO模型（指定的模型路径）
        self.model = YOLO('/home/tl/RV/src/yolo_detect/yolo_detect/yolov11n_best.pt')
        
        # 创建图像订阅者（接收相机图像）
        self.image_sub = self.create_subscription(
            Image,
            '/rtsp_image_0',  # 订阅的相机话题
            self.image_callback,
            10
        )
        
        # 创建检测结果发布者
        self.result_pub = self.create_publisher(
            String,  # 简化示例，实际可使用自定义消息类型
            'yolo/detection_results',  # 发布检测结果的话题
            10
        )
        
        # 创建标注后的图像发布者
        self.annotated_img_pub = self.create_publisher(
            Image,
            'yolo/annotated_image',  # 发布带检测框的图像
            10
        )
        
        self.bridge = CvBridge()  # 用于ROS图像与OpenCV格式转换
        self.get_logger().info('YOLO Inference Node started')

    def image_callback(self, msg):
        # 将ROS图像消息转换为OpenCV格式
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # 运行YOLO检测
        results = self.model(cv_img)
        
        # 处理检测结果
        detection_info = []
        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                cls_name = self.model.names[cls]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                detection_info.append(f"{cls_name} (conf: {conf:.2f}): [{x1},{y1},{x2},{y2}]")
        
        # 发布检测结果文本
        result_msg = String()
        result_msg.data = "; ".join(detection_info)
        self.result_pub.publish(result_msg)
        self.get_logger().info(f"Detected: {result_msg.data}")
        
        # 绘制检测框并发布标注后的图像
        annotated_img = results[0].plot()  # YOLO自带的绘制函数
        ros_annotated_img = self.bridge.cv2_to_imgmsg(annotated_img, 'bgr8')
        self.annotated_img_pub.publish(ros_annotated_img)

def main(args=None):
    rclpy.init(args=args)
    node = YoloInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
