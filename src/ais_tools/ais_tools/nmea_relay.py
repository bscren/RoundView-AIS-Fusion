#!/usr/bin/env python3
# 核心功能：接收原始 NMEA 格式的 AIS 数据（支持串口、TCP、UDP 三种输入方式），转发至 ROS 话题，并可选日志记录和 UDP 转发。
# 关键逻辑：
# 根据配置的 input_type（serial/tcp/udp）初始化对应的数据读取器（SerialReader/TCPReader/UDPReader）。
# 将读取到的 NMEA 句子封装为 nmea_msgs.msg.Sentence 消息，发布到 nmea 话题。
# 支持将数据写入日志文件（按 UTC 时间命名）和转发到指定 UDP 端口。
# 作用：作为数据入口，将外部硬件或网络传来的原始 AIS 数据接入 ROS 系统。

# running as: ros2 run ais_tools nmea_relay --ros-args -p input_type:=serial -p input_address:=/dev/ttyUSB0 -p input_speed:=38400

import serial
import socket
import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
import datetime
import os

class SerialReader:
    def __init__(self, address, speed):
        self.serial_in = serial.Serial(address, speed)

    def readlines(self):
        try:
            nmea_in = self.serial_in.readline()
        # return (nmea_in,)
        # 解码并处理空白字符
            return (nmea_in.decode('utf-8', errors='ignore').strip(),)
        except Exception as e:
            print(f"发生错误: {e}")
            return []

class UDPReader:
    def __init__(self, port):
        self.udp_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_in.settimeout(0.1)
        self.udp_in.bind(('', port))

    def readlines(self):
        try:
            nmea_in = self.udp_in.recv(2048)
            nmea_ins = nmea_in.decode('utf-8').split('\n')
        except socket.timeout:
            return []
        ret = []
        for n in nmea_ins:
            ret.append(n.strip())
        return ret

class TCPReader:
    def __init__(self, address, port):
        self.tcp_in = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_in.settimeout(0.1)
        self.tcp_in.connect((address, port))
        self.leftovers = ''

    def readlines(self):
        try:
            nmea_in = self.tcp_in.recv(256)
            nmea_ins = (self.leftovers+nmea_in.decode('utf-8')).split('\n')
            if len(nmea_ins):
                self.leftovers = nmea_ins[-1]
                ret = []
                for n in nmea_ins[:-1]:
                    ret.append(n.strip())
                return ret
        except socket.timeout:
            pass
        return []










class NMEARelayNode(Node):
    def __init__(self):
        super().__init__('nmea_relay')
        
        # 声明ROS 2参数（带默认值）
        self.declare_parameter('input_type', 'serial')
        self.declare_parameter('input_address', '/dev/ttyUSB0')
        self.declare_parameter('input_speed', 38400)
        self.declare_parameter('input_port', 0)
        self.declare_parameter('output', 0)
        self.declare_parameter('output_address', '<broadcast>')
        self.declare_parameter('frame_id', 'nmea')
        self.declare_parameter('log_directory', '')
        
        # 获取参数值
        input_type = self.get_parameter('input_type').value
        self.input_address = self.get_parameter('input_address').value
        input_speed = self.get_parameter('input_speed').value
        self.input_port = self.get_parameter('input_port').value
        self.output_port = self.get_parameter('output').value
        self.output_address = self.get_parameter('output_address').value
        self.frame_id = self.get_parameter('frame_id').value
        logdir = self.get_parameter('log_directory').value
        
        # 创建NMEA消息发布者
        self.nmea_pub = self.create_publisher(Sentence, 'nmea', 20)
        
        # 初始化日志文件
        self.logfile = None
        if logdir and logdir != "":
            try:
                os.makedirs(logdir, exist_ok=True)  # 确保日志目录存在
                # timestamp = datetime.datetime.utcnow().isoformat().replace(':', '.')
                # 兼容Python 3.10的写法（无警告且功能等价）
                timestamp = datetime.datetime.now(datetime.timezone.utc).isoformat().replace(':', '.')
                log_filename = f'{timestamp}.log'
                self.logfile = open(os.path.join(logdir, log_filename), 'w')
                self.get_logger().info(f"日志文件已创建: {os.path.join(logdir, log_filename)}")
            except Exception as e:
                self.get_logger().error(f"无法创建日志文件: {e}")
        
        # 初始化输入读取器
        try:
            if input_type == 'serial':
                self.reader = SerialReader(self.input_address, input_speed)
                self.get_logger().info(f"已连接串口: {self.input_address} (波特率: {input_speed})")
            elif input_type == 'tcp':
                self.reader = TCPReader(self.input_address, self.input_port)
                self.get_logger().info(f"已连接TCP服务器: {self.input_address}:{self.input_port}")
            else:  # UDP模式
                self.reader = UDPReader(self.input_port)
                self.get_logger().info(f"已启动UDP监听: 端口 {self.input_port}")
        except Exception as e:
            self.get_logger().fatal(f"初始化读取器失败: {e}")
            raise
        
        # 初始化UDP输出
        self.udp_out = None
        if self.output_port > 0:
            try:
                self.udp_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                self.udp_out.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                self.get_logger().info(f"已启动UDP转发: {self.output_address}:{self.output_port}")
            except Exception as e:
                self.get_logger().error(f"初始化UDP输出失败: {e}")
        
        # 创建定时器（10ms周期）用于读取和发布数据
        self.timer = self.create_timer(0.01, self.read_and_publish)

    def read_and_publish(self):
        if not rclpy.ok():  # 检查ROS 2是否正常运行
            return
            
        nmea_ins = self.reader.readlines()
        now = self.get_clock().now()  # 获取ROS 2当前时间

        for nmea in nmea_ins:
            # 转发到UDP
            if self.udp_out is not None:
                try:
                    self.udp_out.sendto(nmea.encode('utf-8'), (self.output_address, self.output_port))
                except Exception as e:
                    self.get_logger().warn(f"UDP转发失败: {e}")

            # 写入日志
            if self.logfile is not None:
                try:
                    timestamp = datetime.datetime.fromtimestamp(now.nanoseconds / 1e9).isoformat()
                    self.logfile.write(f"{timestamp},{nmea}\n")
                    self.logfile.flush()
                except Exception as e:
                    self.get_logger().warn(f"日志写入失败: {e}")

            # 发布到ROS话题
            if nmea:
                sentence = Sentence()
                sentence.header.stamp = now.to_msg()  # 转换为ROS消息时间格式
                sentence.header.frame_id = self.frame_id
                sentence.sentence = nmea
                self.nmea_pub.publish(sentence)

    def destroy_node(self):
        # 资源清理
        if self.logfile:
            self.logfile.close()
        if self.udp_out:
            self.udp_out.close()
        if hasattr(self, 'reader'):
            if isinstance(self.reader, SerialReader):
                self.reader.serial_in.close()
            elif isinstance(self.reader, TCPReader):
                self.reader.tcp_in.close()
            elif isinstance(self.reader, UDPReader):
                self.reader.udp_in.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = NMEARelayNode()
    try:
        rclpy.spin(node)  # 启动节点主循环
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
