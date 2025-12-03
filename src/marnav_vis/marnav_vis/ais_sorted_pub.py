# 循环收集一秒内ais_csv_pub发布的实时零散ais数据，将其整理为固定每秒一次一组AIS数据的发布脚本
# 即：接收话题ais_csv_pub_topic实时发布的单条Ais类话题，
# 将多个单条Ais类话题整理为一个pd.DataFrame(columns=['mmsi','lon', 'lat','speed','course','heading','type','timestamp'])格式的ais_data
# 每秒钟发布一次整理好的ais_data，消息体格式自定义

import rclpy
from rclpy.node import Node
import pandas as pd
from marnav_interfaces.msg import Ais, AisBatch  # 导入原始AIS消息和自定义批量消息
from builtin_interfaces.msg import Time
import time

class AisBatchPublisher(Node):
    def __init__(self):
        super().__init__('ais_batch_publisher_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                # ========= 后需要改为动态传参，目前暂时是手动定义的参数 =========
                ('ais_start_timestamp', 1654315512000),  # AIS数据集起始时间戳，单位：毫秒
                # ================================================
            ]
        )
        # 创建后续实时更新的时间戳
        self.ais_batch_microtimestamp = self.get_parameter('ais_start_timestamp').value
        self.get_logger().info(f"AIS Batch Publisher Node initialized with start timestamp: {self.ais_batch_microtimestamp}")


        
        # 订阅原始AIS单条消息
        self.subscription = self.create_subscription(
            Ais,
            '/ais_csv_topic',  # 订阅原始话题
            self.ais_callback,
            10  # 队列大小
        )
        
        # 创建批量数据发布者
        self.batch_publisher = self.create_publisher(
            AisBatch,
            '/ais_batch_topic',  # 新的批量数据话题
            10
        )
        
        # 定时器设置
        self.timer_period = 1.0  # 每秒发布一次
        # 数据缓存：存储1秒内的AIS消息
        self.ais_cache = []
        # 定时器：每1秒触发一次，发布批量数据
        self.timer = self.create_timer(self.timer_period, self.publish_batch)

        self.get_logger().info("AIS Batch Publisher Node started. Collecting data...")

    def ais_callback(self, msg):
        """接收单条AIS消息，存入缓存"""
        self.ais_cache.append(msg)
        # self.get_logger().info(f"Cached AIS message (MMSI: {msg.mmsi})")

    def publish_batch(self):
        # 更新时间戳
        self.ais_batch_microtimestamp += 1000  # 增加1000毫秒（1秒）
        
        """每1秒将缓存中的数据整理为DataFrame，发布批量消息"""
        if not self.ais_cache:
            self.get_logger().info("No AIS data in the last second.")
            return
        
        # 1. 将缓存数据转换为DataFrame（按需求格式）
        data = []
        for msg in self.ais_cache:
            # 提取消息字段，对应CSV格式的列
            row = {
                'mmsi': msg.mmsi,
                'lon': msg.lon,
                'lat': msg.lat,
                'speed': msg.speed,
                'course': msg.course,
                'heading': msg.heading,
                'type': msg.type,
                # 时间戳转换为毫秒级（与原始CSV保持一致）
                'timestamp': msg.timestamp.sec * 1000 + msg.timestamp.nanosec // 1000000
            }
            data.append(row)
        
        # 创建DataFrame，目前没啥用处，仅供参考
        ais_data = pd.DataFrame(data, columns=['mmsi', 'lon', 'lat', 'speed', 'course', 'heading', 'type', 'timestamp'])
        
        # 获取当前时间戳
        # current_time = self.get_clock().now().to_msg()
        # self.get_logger().info(f"Current Time: sec={current_time.sec}, nanosec={current_time.nanosec}")  
        # self.get_logger().info(f"Publishing batch: {len(ais_data)} records\n")

        # 2. 构造批量消息
        batch_msg = AisBatch()
        batch_msg.ais_list = self.ais_cache  # 直接复用缓存的Ais消息数组
        batch_msg.batch_time = Time()

        # 当前发布时间用数据库的时间戳
        batch_msg.batch_time.sec = self.ais_batch_microtimestamp // 1000  # 秒部分
        batch_msg.batch_time.nanosec = (self.ais_batch_microtimestamp % 1000) * 1000000  # 毫秒转纳秒
        
        # 当前发布时间用实时时间
        # batch_msg.batch_time = self.get_clock().now().to_msg()  
        # 使用毫秒级时间戳
        
        # print(f"Publishing AIS batch with {len(ais_data)} records at time: sec={batch_msg.batch_time.sec}, nanosec={batch_msg.batch_time.nanosec}\n")

        # 3. 发布批量消息
        self.batch_publisher.publish(batch_msg)

        # 4. 清空缓存，准备下一秒的数据
        self.ais_cache = []

def main(args=None):
    rclpy.init(args=args)
    node = AisBatchPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    