import rclpy
from rclpy.node import Node
import json
from datetime import datetime
import os

from detect_interfaces.msg import MultiDetectionsCameras, MultiDetectionResults, DetectionResult

class DetectionSaver(Node):
    def __init__(self):
        super().__init__('detection_saver')
        
        # 订阅目标话题
        self.subscription = self.create_subscription(
            MultiDetectionsCameras,
            'yolo/detection_results',  # 替换为实际的话题名称
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量警告
        
        # 初始化保存文件
        self.save_directory = "detection_data"
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)
            
        # 生成带时间戳的文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"{self.save_directory}/detections_{timestamp}.json"
        
        self.get_logger().info(f"开始保存检测数据到 {self.filename}")

    def listener_callback(self, msg):
        """处理接收到的消息并保存"""
        # 将ROS消息转换为可序列化的字典
        data = self.msg_to_dict(msg)
        
        # 写入JSON文件
        with open(self.filename, 'a') as f:
            json.dump(data, f)
            f.write('\n')  # 每条消息占一行，方便后续处理
            
        self.get_logger().info(f"保存了 {len(msg.multicamdetections)} 个相机的检测结果")

    def msg_to_dict(self, msg):
        """将MultiDetectionsCameras消息转换为字典"""
        result = {
            "timestamp": datetime.now().isoformat(),
            "multicamdetections": []
        }
        
        for cam_detection in msg.multicamdetections:
            cam_data = {
                "camera_name": cam_detection.camera_name,
                "stamp": {
                    "sec": cam_detection.stamp.sec,
                    "nanosec": cam_detection.stamp.nanosec
                },
                "results": []
            }
            
            for detection in cam_detection.results:
                det_data = {
                    "class_name": detection.class_name,
                    "confidence": float(detection.confidence),
                    "x1": int(detection.x1),
                    "y1": int(detection.y1),
                    "x2": int(detection.x2),
                    "y2": int(detection.y2)
                }
                cam_data["results"].append(det_data)
                
            result["multicamdetections"].append(cam_data)
            
        return result

def main(args=None):
    rclpy.init(args=args)
    detection_saver = DetectionSaver()
    rclpy.spin(detection_saver)
    
    # 关闭节点
    detection_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
