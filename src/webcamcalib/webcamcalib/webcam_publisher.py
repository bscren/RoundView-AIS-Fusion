#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import yaml
from ament_index_python.packages import get_package_share_directory
#  自定义消息头文件
#include "mycustface/msg/j_hjpg.hpp"
#include "mycustface/msg/header.hpp"
from mycustface.msg import JHjpg,Header

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')  # 节点名称
        
        # 声明参数（带默认值）
        self.declare_parameter('image_raw_topic', '/image_raw')
        self.declare_parameter('image_calib_topic', '/image_calib')
        self.declare_parameter('JHjpg_raw_topic', '/JHjpg_raw')
        self.declare_parameter('JHjpg_calib_topic', '/JHjpg_calib')
        self.declare_parameter('relative_calib_file_path', 'CamsCalibrationData/DAHUA192.168.4.118/ost.yaml')
        self.declare_parameter('camera_url', "rtsp://admin:sjtu1234@192.168.4.118:554/cam/realmonitor?channel=1&subtype=0")
        self.declare_parameter('frame_rate', 20)
        self.declare_parameter('PubImageorJHjpgorRaworCalib', 'JHjpgCalib')  # # 可选 'ImageRaw' 或 'ImageRawCalib' 或 'JHjpgRaw' 或 'JHjpgCalib'
        self.declare_parameter('SCALE', 0.5)  # 图像缩放比例，默认不缩放
        # 获取参数值
        self.image_raw_topic = self.get_parameter('image_raw_topic').get_parameter_value().string_value
        self.image_calib_topic = self.get_parameter('image_calib_topic').get_parameter_value().string_value
        self.JHjpg_raw_topic = self.get_parameter('JHjpg_raw_topic').get_parameter_value().string_value
        self.JHjpg_calib_topic = self.get_parameter('JHjpg_calib_topic').get_parameter_value().string_value
        self.Ros2ImageorJHjpg = self.get_parameter('PubImageorJHjpgorRaworCalib').get_parameter_value().string_value
        self.SCALE = self.get_parameter('SCALE').get_parameter_value().double_value
        print("发布话题的类型为：",self.Ros2ImageorJHjpg)


        relative_calib_file_path = self.get_parameter('relative_calib_file_path').get_parameter_value().string_value
        self.camera_url = self.get_parameter('camera_url').get_parameter_value().string_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value
        
        # 计算标定文件绝对路径
        package_share_directory = get_package_share_directory('webcamcalib')  # 注意包名已改为webcamcalib
        # output: /home/tl/RV/install/webcamcalib/share/webcamcalib
        self.absolute_calib_file_path = os.path.join(package_share_directory, relative_calib_file_path)
        print("标定文件在编译后的install绝对路径为： ",self.absolute_calib_file_path) 
        # output: /home/tl/RV/install/webcamcalib/share/webcamcalib/CamsCalibrationData/DAHUA192.168.0.118/ost.yaml

        # 创建发布者
        self.image_raw_pub = self.create_publisher(Image, self.image_raw_topic, 10)
        self.image_calib_pub = self.create_publisher(Image, self.image_calib_topic, 10)
        self.JHjpg_raw_pub = self.create_publisher(JHjpg, self.JHjpg_raw_topic, 10)
        self.JHjpg_calib_pub = self.create_publisher(JHjpg, self.JHjpg_calib_topic, 10)

        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 加载标定文件
        if not self.load_calibration_file():
            return
        
        # 打开相机
        self.cap = cv2.VideoCapture(self.camera_url)
        if not self.cap.isOpened():
            self.get_logger().error(f"无法连接相机: {self.camera_url}")
            return
        
        # 创建定时器（按帧率触发回调）
        self.timer = self.create_timer(1.0 / self.frame_rate, self.timer_callback)
        # self.get_logger().info(f"节点启动成功！发布话题: {self.image_raw_topic} (原始), {self.image_calib_topic} (校正后)")

    def load_calibration_file(self):
        """加载相机标定文件并计算校正映射"""
        if not os.path.exists(self.absolute_calib_file_path):
            self.get_logger().error(f"标定文件不存在: {self.absolute_calib_file_path}")
            return False
        
        try:
            with open(self.absolute_calib_file_path, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            # 提取标定参数
            self.image_width = calib_data['image_width']
            self.image_height = calib_data['image_height']
            self.camera_matrix = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
            self.distortion_coeffs = np.array(calib_data['distortion_coefficients']['data'])
            
            # 计算校正映射
            self.new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.distortion_coeffs,
                (self.image_width, self.image_height), 0,
                (self.image_width, self.image_height)
            )
            self.mapx, self.mapy = cv2.initUndistortRectifyMap(
                self.camera_matrix, self.distortion_coeffs, None, self.new_camera_matrix,
                (self.image_width, self.image_height), cv2.CV_32FC1
            )
            
            self.get_logger().info("标定文件加载成功")
            return True
            
        except KeyError as e:
            self.get_logger().error(f"标定文件缺少参数: {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"解析标定文件失败: {e}")
            return False

    def timer_callback(self):
        """定时器回调：读取图像并发布"""
        ret, frame = self.cap.read()
        if ret:
            try:
                if self.Ros2ImageorJHjpg == 'ImageRaw':
                    # 发布sensor_msgs/Image类型
                    ros_raw_img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    ros_raw_img.header.stamp = self.get_clock().now().to_msg()
                    ros_raw_img.header.frame_id = "camera_link"
                    self.image_raw_pub.publish(ros_raw_img)
                    
                elif self.Ros2ImageorJHjpg == 'ImageRawCalib':
                    # 发布sensor_msgs/Image类型（校正后）
                    frame_calib = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
                    ros_calib_img = self.bridge.cv2_to_imgmsg(frame_calib, "bgr8")
                    ros_calib_img.header.stamp = self.get_clock().now().to_msg()
                    ros_calib_img.header.frame_id = "camera_link"
                    self.image_calib_pub.publish(ros_calib_img)
                
                elif self.Ros2ImageorJHjpg == 'JHjpgRaw':
                    # 发布mycustface/JHjpg类型
                    jh_msg_raw = JHjpg()  
                    custom_header = Header()
                    # 获取秒级时间戳（32位整数）
                    current_time = self.get_clock().now()
                    # 纳秒转秒（取整）
                    timestamp_32 = int(current_time.nanoseconds / 1000000000)
                    custom_header.timestamp = timestamp_32
                    # custom_header.timestamp = self.get_clock().now().nanoseconds
                    custom_header.id = "DAHUA_image"
                    jh_msg_raw.mheader = custom_header
                    jh_msg_raw.index = 1
                    jh_msg_raw.message = "DAHUA_image_raw"
                    
                    # 计算缩小一半的尺寸并截断取整
                    resized_img = cv2.resize(
                        frame,
                        (int(frame.shape[1] * self.SCALE), int(frame.shape[0] * self.SCALE)),
                        interpolation=cv2.INTER_LINEAR
                    )

                    success_raw, jpg_data_raw = cv2.imencode(".jpg", resized_img, [int(cv2.IMWRITE_JPEG_QUALITY),95])
                    if success_raw:
                        jpg_bytes_raw = jpg_data_raw.tobytes()
                        # 拆分为单个字节的bytes对象列表（每个元素是1字节）
                        # 例如：b'abc' → [b'a', b'b', b'c']
                        jh_msg_raw.picture = [bytes([a]) for a in jpg_bytes_raw]
                        jh_msg_raw.size = len(jh_msg_raw.picture[0])
                        self.JHjpg_raw_pub.publish(jh_msg_raw)
                    else:
                        self.get_logger().error("JPEG编码失败，无法发布消息")

                elif self.Ros2ImageorJHjpg == 'JHjpgCalib':
                    # 发布mycustface/JHjpg类型
                    jh_msg_calib = JHjpg()
                    custom_header = Header()
                    # 获取秒级时间戳（32位整数）
                    current_time = self.get_clock().now()
                    # 纳秒转秒（取整）
                    timestamp_32 = int(current_time.nanoseconds / 1000000000)
                    custom_header.timestamp = timestamp_32
                    custom_header.id = "DAHUA_image"
                    jh_msg_calib.mheader = custom_header
                    jh_msg_calib.index = 1
                    jh_msg_calib.message = "DAHUA_image_calib"
                    frame_calib = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
                    # 计算缩小一半的尺寸并截断取整
                    resized_img = cv2.resize(
                        frame_calib,
                        (int(frame_calib.shape[1] * self.SCALE), int(frame_calib.shape[0] * self.SCALE)),
                        interpolation=cv2.INTER_LINEAR
                    )


                    success_calib, jpg_data_calib = cv2.imencode(".jpg", resized_img, [int(cv2.IMWRITE_JPEG_QUALITY),95])
                    if  success_calib:
                        jpg_bytes_calib = jpg_data_calib.tobytes()
                        # 拆分为单个字节的bytes对象列表（每个元素是1字节）
                        # 例如：b'abc' → [b'a', b'b', b'c']
                        jh_msg_calib.picture = [bytes([b]) for b in jpg_bytes_calib]
                        jh_msg_calib.size = len(jh_msg_calib.picture[0])

                        # 发布校正后图像
                        self.JHjpg_calib_pub.publish(jh_msg_calib)
                    else:
                        self.get_logger().error("JPEG编码失败，无法发布消息")

                
            except CvBridgeError as e:
                self.get_logger().error(f"图像转换错误: {e}")
        else:
            self.get_logger().warn("无法获取图像帧，尝试重连...")
            self.cap.release()
            self.cap = cv2.VideoCapture(self.camera_url)
            if not self.cap.isOpened():
                self.get_logger().error("重连失败，关闭节点")
                self.destroy_node()

    def __del__(self):
        """析构函数：释放相机资源"""
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    try:
        camera_pub = CameraPublisher()
        rclpy.spin(camera_pub)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"运行错误: {e}")
    finally:
        if 'camera_pub' in locals():
            camera_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()