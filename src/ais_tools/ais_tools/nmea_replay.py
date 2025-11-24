#!/usr/bin/env python3

# 核心功能：从日志文件中回放 NMEA 数据，用于离线测试或数据复现。
# 关键逻辑：
# 读取包含时间戳和 NMEA 句子的日志文件（格式：timestring,nmea）。
# 按照指定速率（rate 参数）和时间戳关系，将 NMEA 数据封装为 nmea_msgs.msg.Sentence 消息发布到 nmea 话题。
# 可选发布 /clock 话题模拟时钟，便于同步其他依赖时间的节点。
# 作用：无需真实硬件，通过历史日志数据测试后续 AIS 解析和处理流程

# run as : ros2 run ais_tools nmea_replay --ros-args -p filename:=/home/tl/RV/src/ais_tools/logs/2019-10-15T12.57.27.338020_ais.log -p rate:=2.0 -p clock:=true
import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
from rosgraph_msgs.msg import Clock
import datetime
import time


class NMEA_ReplayNode(Node):
    def __init__(self):
        super().__init__('nmea_replay')
        
        # 声明并获取参数
        self.declare_parameter('filename', '')
        self.declare_parameter('frame_id', 'nmea') # frame_id 参数用于指定消息关联的坐标系（Coordinate Frame）
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('clock', False)
        
        input_filename = self.get_parameter('filename').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rate = self.get_parameter('rate').value
        self.publish_clock = self.get_parameter('clock').value
        
        # 验证必要参数
        if not input_filename:
            self.get_logger().fatal("必须指定输入文件: 设置 'filename' 参数")
            raise ValueError("缺少 filename 参数")
        
        # 创建发布者
        self.nmea_pub = self.create_publisher(Sentence, 'nmea', 20)
        self.clock_publisher = None
        if self.publish_clock:
            self.clock_publisher = self.create_publisher(Clock, '/clock', 5)
        
        # 初始化时间变量
        self.data_start_time = None
        self.clock_period = datetime.timedelta(seconds=0.2)
        self.wallclock_start = datetime.datetime.utcnow()
        self.next_clock_time = self.wallclock_start
        
        # 读取日志文件数据
        try:
            with open(input_filename, 'r') as f:
                self.lines = f.readlines()
            self.get_logger().info(f"已加载日志文件: {input_filename}，共 {len(self.lines)} 条记录")
        except Exception as e:
            self.get_logger().fatal(f"无法读取文件: {e}")
            raise
        
        # 启动数据处理循环
        self.process_lines()

    def process_lines(self):
        for line in self.lines:
            # 跳过空行
            if not line.strip():
                continue
            
            # 解析时间戳和NMEA句子
            try:
                timestring, nmea = line.split(',', 1)
                datatime = datetime.datetime.fromisoformat(timestring)
                nmea = nmea.strip()  # 移除换行符和空白
            except ValueError:
                self.get_logger().warn(f"无效的行格式: {line.strip()}，已跳过")
                continue
            
            # 初始化数据起始时间
            if self.data_start_time is None:
                self.data_start_time = datatime
            
            # 控制发布时机
            while True:
                now = datetime.datetime.now(datetime.timezone.utc)
                # 计算当前应对应的日志时间
                elapsed = (now - self.wallclock_start).total_seconds() * self.rate
                rosnow = self.data_start_time + datetime.timedelta(seconds=elapsed)
                
                # 发布时钟（如果启用）
                if self.publish_clock and self.next_clock_time <= now:
                    clock_elapsed = (self.next_clock_time - self.wallclock_start).total_seconds() * self.rate
                    rosclock_time = self.data_start_time + datetime.timedelta(seconds=clock_elapsed)
                    clock_msg = Clock()
                    clock_msg.clock.sec = int(rosclock_time.timestamp())
                    clock_msg.clock.nanosec = int((rosclock_time.timestamp() - clock_msg.clock.sec) * 1e9)
                    self.clock_publisher.publish(clock_msg)
                    self.next_clock_time += self.clock_period
                
                # 检查是否到达发布时间
                if datatime <= rosnow:
                    # 发布NMEA消息
                    sentence = Sentence()
                    sentence.header.frame_id = self.frame_id
                    sentence.header.stamp.sec = int(datatime.timestamp())
                    sentence.header.stamp.nanosec = int((datatime.timestamp() - sentence.header.stamp.sec) * 1e9)
                    sentence.sentence = nmea
                    self.nmea_pub.publish(sentence)
                    break
                
                # 计算休眠时间
                time_diff = (datatime - rosnow).total_seconds() / self.rate
                if self.publish_clock:
                    clock_diff = (self.next_clock_time - now).total_seconds()
                    time_diff = min(time_diff, clock_diff)
                
                # 避免负向休眠
                if time_diff > 0:
                    time.sleep(time_diff)
                else:
                    time.sleep(0.001)  # 最小休眠时间，防止CPU占用过高

def main(args=None):
    rclpy.init(args=args)
    try:
        node = NMEA_ReplayNode()
        # 处理完所有数据后销毁节点
        node.destroy_node()
    except Exception as e:
        rclpy.shutdown()
        raise e
    rclpy.shutdown()

if __name__ == '__main__':
    main()
