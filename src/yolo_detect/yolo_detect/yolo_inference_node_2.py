# 注意事项，在虚拟环境中编译不能使用colcon build，而需要使用python3 -m colcon build，以调用虚拟环境的python解释器
# 同时，要想查看消息体的具体格式，在调用ros2 interface show detect_interfaces/msg/MultiDetectionsCameras之前，需要刷新：source ~/RV/install/setup.bash

# 在编译前执行，临时调整库搜索路径（系统库优先）————有这种方法，但是此处不适用
# export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from message_filters import Subscriber, ApproximateTimeSynchronizer
import cv2
from builtin_interfaces.msg import Time
from detect_interfaces.msg import DetectionResult, MultiDetectionResults, MultiDetectionsCameras

class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')
        
        # 加载YOLO模型
        self.model = YOLO('/home/tl/RV/src/yolo_detect/yolo_detect/yolov11n_best.pt')
        
        # 创建三个相机话题的订阅者
        self.img0_sub = Subscriber(self, Image, 'rtsp_image_0')
        self.img1_sub = Subscriber(self, Image, 'rtsp_image_1')
        self.img2_sub = Subscriber(self, Image, 'rtsp_image_2')
        
        # 创建时间同步器
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.img0_sub, self.img1_sub, self.img2_sub],
            queue_size=20,
            slop=0.1
        )
        self.synchronizer.registerCallback(self.sync_callback)
        
        # 创建检测结果发布者（改为发布MultiCameraDetections数组）
        self.result_pub = self.create_publisher(
            MultiDetectionsCameras, 
            'yolo/detection_results', 
            10
        )
        
        # 创建三个标注图像的发布者
        self.annotated_pubs = {
            'rtsp_image_0': self.create_publisher(Image, 'yolo/annotated_image_00', 10),
            'rtsp_image_1': self.create_publisher(Image, 'yolo/annotated_image_11', 10),
            'rtsp_image_2': self.create_publisher(Image, 'yolo/annotated_image_22', 10)
        }
        
        self.bridge = CvBridge()
        self.get_logger().info('YOLO Inference Node with time sync started')

    def sync_callback(self, img0_msg, img1_msg, img2_msg):
        """处理同步后的三个相机图像"""
        # 建立相机消息与名称的映射
        camera_data = [
            ('rtsp_image_0', img0_msg),
            ('rtsp_image_1', img1_msg),
            ('rtsp_image_2', img2_msg)
        ]
        
        # 存储所有相机的检测结果消息
        all_cam_detections = MultiDetectionsCameras()
        
        # 批量处理三个相机的图像
        for cam_name, img_msg in camera_data:
            # 图像格式转换
            try:
                cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            except Exception as e:
                self.get_logger().error(f"Image conversion error for {cam_name}: {e}")
                continue
            
            # 模型推理
            results = self.model(cv_img, verbose=False)
            # 获取当前时间（ROS节点的时间）
            currenttime = self.get_clock().now().to_msg()
            # 构造该相机的检测结果消息
            cam_detection_msg = MultiDetectionResults()
            # string camera_name
            # builtin_interfaces/Time timestamp
            # DetectionResult[] results
            cam_detection_msg.camera_name = cam_name
            cam_detection_msg.stamp = currenttime
            cam_detection_msg.results = []
            # 提取检测结果并填充消息
            for box in results[0].boxes:
                det_result = DetectionResult()
                det_result.class_name = self.model.names[int(box.cls[0])]
                det_result.confidence = float(box.conf[0])
                # 左上/右下的xy坐标
                det_result.x1, det_result.y1, det_result.x2, det_result.y2 = map(int, box.xyxy[0])
                cam_detection_msg.results.append(det_result)  # 添加到数组
            
            all_cam_detections.multicamdetections.append(cam_detection_msg)
            
            
            # 发布标注图像
            annotated_img = results[0].plot()
            ros_annotated_img = self.bridge.cv2_to_imgmsg(annotated_img, 'bgr8')
            self.annotated_pubs[cam_name].publish(ros_annotated_img)
        
        # 发布所有相机的检测结果（逐个发布每个相机的结果）
        self.result_pub.publish(all_cam_detections)


def main(args=None):
    rclpy.init(args=args)
    node = YoloInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
