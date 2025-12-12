import pandas as pd
import os
import rclpy
from rclpy.node import Node
from marnav_interfaces.msg import Ais
from builtin_interfaces.msg import Time
import os, time, glob
from datetime import datetime,timezone,timedelta
from marnav_vis.config_loader import ConfigLoader

# AIS数据实时接收-发布脚本 2022_06_04_12_07_37
# 功能: 接收AIS数据并发布，AIS数据从指定文件夹路径读取，文件夹中存在多个AIS数据的csv文件，
# csv文件命名格式为: YYYY_MM_DD_HH_MM_SS.csv, 
# 第n秒读取一次最新的csv文件,其中有x条ais数据，随后在第n秒-第n+1秒之间，发布x次其中的AIS数据。
# 在第n+1秒时，读取下一个csv文件，以此类推。
class AisPubNode(Node):
    def __init__(self):
        super().__init__('ais_csv_publisher_node')

        # 声明配置文件参数
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        # 如果未指定配置文件，使用默认路径
        if not config_file:
            try:
                config_file = ConfigLoader.find_config_file('marnav_vis', 'track_offline_config.yaml')
                self.get_logger().info(f"未指定配置文件，使用默认路径: {config_file}")
            except Exception as e:
                self.get_logger().error(f"查找默认配置文件失败: {e}")
                raise
        
        # 加载配置
        try:
            config_loader = ConfigLoader(config_file)
            ais_config = config_loader.get_ais_config()
        except Exception as e:
            self.get_logger().fatal(f"加载配置文件失败: {e}")
            raise
        
        # 从配置中读取参数
        self.ais_csv_folder = ais_config.get('ais_csv_folder', '')
        self.ais_csv_topic = ais_config.get('ais_csv_topic', '/ais_csv_topic')
        
        if not self.ais_csv_folder:
            self.get_logger().fatal("配置文件中未定义ais_csv_folder")
            raise ValueError("未定义AIS CSV文件夹路径")
        
        self.get_logger().info("="*60)
        self.get_logger().info("📡 AIS CSV发布节点配置")
        self.get_logger().info("="*60)
        self.get_logger().info(f"配置文件: {config_file}")
        self.get_logger().info(f"AIS CSV文件夹: {self.ais_csv_folder}")
        self.get_logger().info(f"发布话题: {self.ais_csv_topic}")
        self.get_logger().info(f"发布频率: 1 Hz (固定)")
        self.get_logger().info("="*60)


        # 创建AIS消息发布者
        self.ais_publisher = self.create_publisher(
            Ais,
            self.ais_csv_topic,
            10,# 队列大小
        )
        # 创建定时器，1秒钟触发一次
        self.timer_period = 1 # 秒
        self.timer = self.create_timer(timer_period_sec=self.timer_period,callback = self.timer_callback)
        self.get_logger().info("AIS Raw Publisher Node has been started.")
        self.last_published_time = None # 上次发布的时间，用于判断是否需要读取新的CSV文件
        self.start_time = None # 启动时再赋值

        # 每个文件名称格式如下：2022_06_04_12_05_12.csv
        self.csv_files = None # 当前读取的CSV数组
        self.current_csv_file = None # 当前读取的CSV文件名称
        self.current_csv_index = 0 # 当前CSV数据的索引

        # ========================== 初始化资源==========================
        self.initialize_resources()



    # 定时器回调函数
    def timer_callback(self):
        
        # 获取当前时间戳
        current_time = self.get_clock().now().to_msg()
        # print(f"Current time: sec={current_time.sec}, nanosec={current_time.nanosec}")
        # 使用毫秒级时间戳
        current_time_sec = current_time.sec*1e3 + current_time.nanosec / 1e6 # 毫秒级时间戳
        if self.start_time is None:
            self.start_time = current_time_sec
        
        # 计算经过的秒数
        # elapsed_seconds = int((current_time_sec - self.start_time))

        # if elapsed_seconds % 1000 < self.timer_period * 1000:
        # 计算当前应读取的CSV文件名称
        self.current_csv_file = self.csv_files[self.current_csv_index]
        # 读取csv文件里的多个ais数据，并逐个分开发布，间隔时间为10毫秒
        ais_data = self.read_ais()
        # print(f"Publishing {len(ais_data)} AIS messages from file: {self.current_csv_file}")
        for index, row in ais_data.iterrows():
            ais_msg = Ais()
            ais_msg.mmsi = int(row['mmsi'])
            ais_msg.lon = row['lon']
            ais_msg.lat = row['lat']
            ais_msg.speed = row['speed']
            ais_msg.course = row['course']
            ais_msg.heading = row['heading']
            ais_msg.type = int(row['type'])
            # 提取秒和纳秒（毫秒级时间戳转换）
            total_millis = row['timestamp'] # timestamp是以毫秒为单位Unix时间戳
            ais_msg.timestamp = Time()
            ais_msg.timestamp.sec = int(total_millis // 1000)  # 秒部分
            ais_msg.timestamp.nanosec = int((total_millis % 1000) * 1e6)  # 毫秒转纳秒（1毫秒=1e6纳秒）
            self.ais_publisher.publish(ais_msg)
            time.sleep(0.01)  # 10毫秒间隔

        self.current_csv_index += 1
        if self.current_csv_index >= len(self.csv_files):
            self.current_csv_index = 0 # 循环读取
        


    # 读取并排序AIS CSV文件名称
    def read_csv_name(self):
        # print("read_ais path:", self.ais_csv_folder)

        # 获取所有csv文件
        csv_files = glob.glob(self.ais_csv_folder+'*.csv')

        # 解析文件名中的时间并排序
        sorted_files = []
        for file in csv_files:
            file_name = os.path.basename(file)
            time_str = os.path.splitext(file_name)[0]  # 去除.csv后缀，得到"2022_06_04_12_06_50"
            # print(f"Found CSV file: {file_name} with time string: {time_str}")

            try:
                # 将时间字符串转换为datetime对象（用于排序）
                # 格式对应：年_月_日_时_分_秒
                time_obj = datetime.strptime(time_str, "%Y_%m_%d_%H_%M_%S")
                # print(f"Parsed time object: {time_obj} for file: {file_name}")

                # 存储（时间对象, 文件路径），用于后续排序
                sorted_files.append( (time_obj, file) )
            except ValueError:
                # 忽略不符合命名格式的文件（如其他CSV文件）
                print(f"跳过不符合格式的文件: {file}")
                continue
        
        # 3. 按时间对象升序排序（从早到晚）
        sorted_files.sort(key=lambda x: x[0])
        
        # 4. 提取排序后的文件路径，组成字符串数组
        result = [file_path for (time_obj, file_path) in sorted_files]
        return result


        
    
    def read_ais(self):
        try:
            # 读取ais数据
            path = self.current_csv_file
            ais_data = pd.read_csv(path, usecols=[1, 2, 3, 4, 5, 6, 7, 8], header=0)
            # print(f"Reading AIS data from file: {path}, number of records: {len(ais_data)}\n")
            # self.AIS_row = self.AIS_row.append(ais_data, ignore_index=True)
        except:
            ais_data = pd.DataFrame(columns=['mmsi','lon', 'lat','speed','course','heading','type','timestamp'])
        return ais_data
    
    def initialize_resources(self):
        """初始化资源"""
        self.csv_files = self.read_csv_name() 
        # self.get_logger().info(f"Initial CSV file to read: {self.csv_name}")
    
    def run(self):
        """主循环：处理视频帧与数据融合"""
        while rclpy.ok():
            rclpy.spin_once(self)  # 处理回调函数（如定时器回调）
        self.get_logger().info("Shutting down AIS CSV Publisher Node.")
        



def main(args=None):
    # 初始化ROS 2
    rclpy.init(args=args)
    
    # 创建节点并运行
    node = AisPubNode()
    node.run()

    # 清理资源
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()