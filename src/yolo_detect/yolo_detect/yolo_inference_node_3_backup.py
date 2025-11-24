# 该版本用于读取指定视频文件并使用JHjpg格式作为节点信息类型发布
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from mycustface.msg import JHjpg, Header  # 导入自定义消息类型
from sensor_msgs.msg import Image

class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')
        
        # 视频配置参数
        self.video_path = "/home/tl/RV/VideonImage/船舶02.mp4"  # 指定视频地址，需修改为实际路径
        self.SCALE = 1  # 图像缩放比例
        self.msg_index_ = 0  # 消息计数器
        
        # 加载YOLO模型
        self.model = YOLO('/home/tl/RV/src/yolo_detect/yolo_detect/yolov11n_jh.pt')
        
        # 打开视频文件
        self.video_capture = cv2.VideoCapture(self.video_path)
        if not self.video_capture.isOpened():
            self.get_logger().error(f"无法打开视频文件: {self.video_path}")
            return
        
        # 创建处理结果发布者（JHjpg类型）
        self.processed_pub = self.create_publisher(
            JHjpg,
            # Image, 
            'yolo/processed_video', 
            10
        )
        
        # 根据视频帧率设置定时器周期（默认30fps，约33ms）
        # fps = self.video_capture.get(cv2.CAP_PROP_FPS)
        fps = 15
        timer_period = 1.0 / fps if fps > 0 else 0.033
        self.timer = self.create_timer(timer_period, self.process_video_frame)
        
        self.bridge = CvBridge()
        self.get_logger().info(f'YOLO视频处理节点启动，视频路径: {self.video_path}，帧率: {fps:.1f}')

    def process_video_frame(self):
        """处理视频帧并发布JHjpg格式消息"""
        # 读取视频帧
        ret, frame = self.video_capture.read()
        # 当视频读取到末尾时，停止播放
        # if not ret:
        #     self.get_logger().info("视频处理完毕或读取失败，关闭节点")
        #     self.timer.cancel()  # 停止定时器
        #     return

         # 当视频读取到末尾时，重置读取指针到开始位置
        if not ret:
            self.get_logger().info("视频播放完毕，开始循环播放")
            # 重置视频读取指针到起始位置
            self.video_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            # 重新读取第一帧
            ret, frame = self.video_capture.read()
            if not ret:
                self.get_logger().error("重置视频后仍无法读取帧，停止定时器")
                self.timer.cancel()
                return
        
        # YOLO模型推理
        results = self.model(frame, verbose=False)
        annotated_img = results[0].plot()  # 获取标注后的图像

        # ros_annotated_img = self.bridge.cv2_to_imgmsg(annotated_img, 'bgr8')
        # self.processed_pub.publish(ros_annotated_img)
        # f"发布ros_annotated_img消息"————————————————经验证可行


        # 处理图像格式（仿照publishStitchedImage函数逻辑）
        # stitched_8u = self.convert_to_8u(annotated_img)
        # if stitched_8u is None:  # 检查转换结果是否有效
        #     return
        
        # 图像缩放
        resized_img = cv2.resize(
            annotated_img,
            (int(annotated_img.shape[1] * self.SCALE), int(annotated_img.shape[0] * self.SCALE)),
            interpolation=cv2.INTER_LINEAR
        )
        
        # 构建JHjpg消息
        jh_msg = JHjpg()
        
        # 填充自定义Header
        # custom_header = Header()
        # custom_header.timestamp = self.get_clock().now().nanoseconds  # 当前时间纳秒数
        # custom_header.id = f"yolo_processed_{self.msg_index_}"  # 自定义ID
        # jh_msg.mheader = custom_header

        # 修改填充自定义Header的部分
        custom_header = Header()
        # custom_header.timestamp = self.get_clock().now().nanoseconds 
        # 获取秒级时间戳（32位整数）
        current_time = self.get_clock().now()
        # 纳秒转秒（取整）
        timestamp_32 = int(current_time.nanoseconds / 1000000000)
        custom_header.timestamp = timestamp_32
        custom_header.id = f"yolo_processed_{self.msg_index_}"  # 自定义ID
        jh_msg.mheader = custom_header
        
        # 填充序号和描述信息
        jh_msg.index = self.msg_index_
        jh_msg.message = "YOLO processed video frame"
        
        # 编码为JPEG
        # picture 的定义为 sequence<octet> picture;
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, 90]
        # cv2.imshow("resized_img",resized_img)
        # cv2.waitKey(10)
        success, jpg_data = cv2.imencode(".jpg", resized_img, [int(cv2.IMWRITE_JPEG_QUALITY),95])
        print("jpg_data type = ",type(jpg_data))
        print("jh_msg.picture = ",type(jh_msg.picture))


        # cv2.waitKey(0)
        if success:
            # 将字节数据转换为整数列表（符合sequence<octet>要求）
            # 所谓错误写法：jh_msg.picture = [jpg_data.tobytes()]
            
            # 正确写法：
            jpg_bytes = jpg_data.tobytes()
            
            # 2. 拆分为单个字节的bytes对象列表（每个元素是1字节）
            # 例如：b'abc' → [b'a', b'b', b'c']
            jh_msg.picture = [bytes([b]) for b in jpg_bytes]
    
            jh_msg.size = len(jh_msg.picture[0])


            # 1. 获取列表中元素的数量（长度）
            element_count = len(jh_msg.picture)
            print(f"picture 列表包含 {element_count} 个元素")

            # 2. 获取图片数据的实际字节大小（假设列表中第一个元素是图片的 bytes 数据）
            if jh_msg.picture:  # 先判断列表非空
                data_size = len(jh_msg.picture[0])  # 取第一个元素（bytes）的长度
                print(f"图片数据大小：{data_size} 字节\n")
            else:
                print("picture 列表为空，无数据")



            self.processed_pub.publish(jh_msg)
            self.get_logger().info(
                f"发布JHjpg消息，大小: {jh_msg.size} 字节，序号: {jh_msg.index}"
            )
            self.msg_index_ += 1
        else:
            self.get_logger().error("JPEG编码失败，无法发布消息")


    def convert_to_8u(self, image):
        """将图像转换为8位格式（仿照示例中的格式转换逻辑）"""
        # 修正：检查numpy数组是否为空的正确方式
        if image.size == 0:
            self.get_logger().error("输入图像为空，无法转换格式")
            return None
        
        if image.dtype == np.int16 and image.shape[-1] == 3:  # CV_16SC3
            img_32s = image.astype(np.int32)
            img_32u = img_32s + 32768  # 偏移量处理
            img_8u = cv2.normalize(
                img_32u, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC3
            )
        elif image.dtype == np.float32 and image.shape[-1] == 3:  # CV_32FC3
            img_8u = cv2.normalize(
                image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC3
            )
        else:
            # 其他格式直接转换为8位
            img_8u = image.astype(np.uint8) if image.dtype != np.uint8 else image
        
        return img_8u

    def destroy_node(self):
        self.video_capture.release()  # 释放视频资源
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
