# 编写脚本,伪装gnss设备，以指定帧率发布相机参数，包括：
#       Lon	        Lat	    Horizontal	Vertical	Camera Height   //	Horizontal FoV	    Vertical FoV	fx	        fy	         u0	        v0
# 举例：114.32583	30.60139	7	        -1	        20	         //       55	            30.94	   2391.26	    2446.89	    1305.04	    855.214

#  GNSS信息
# float64 latitude                # 纬度
# float64 longitude               # 经度
# float64 horizontal_orientation  # 水平方向角，单位：度
# float64 vertical_orientation    # 垂直方向角，单位：度
# float64 camera_height           # 高度，单位：米
# builtin_interfaces/Time timestamp  # 时间戳

import rclpy
from rclpy.node import Node
import random
from marnav_interfaces.msg import Gnss  # 导入自定义消息
from builtin_interfaces.msg import Time
from marnav_vis.config_loader import ConfigLoader

class GnssPublisher(Node):
    def __init__(self):
        super().__init__('gnss_publisher_node')
        
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
            gnss_config = config_loader.get_gnss_config()
        except Exception as e:
            self.get_logger().fatal(f"加载配置文件失败: {e}")
            raise
        
        # 从配置中读取参数
        self.publish_rate = gnss_config.get('gnss_publish_rate', 5.0)
        self.gnss_pub_topic = gnss_config.get('gnss_pub_topic', '/gnss_pub_topic')
        self.timer_period = 1.0 / self.publish_rate  # 定时器周期（秒）
        
        # 读取GNSS位置参数
        camera_gnss_para = gnss_config.get('camera_gnss_para', {})
        self.base_params = {
            'lon': camera_gnss_para.get('lon', 114.32583),
            'lat': camera_gnss_para.get('lat', 30.60139),
            'horizontal_orientation': camera_gnss_para.get('horizontal_orientation', 352.0),
            'vertical_orientation': camera_gnss_para.get('vertical_orientation', -4.0),
            'camera_height': camera_gnss_para.get('camera_height', 20.0)
        }
        
        # 创建发布者
        self.publisher_ = self.create_publisher(
            Gnss,
            self.gnss_pub_topic,
            10
        )
        
        # 创建定时器
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info("="*60)
        self.get_logger().info("📡 GNSS发布节点配置")
        self.get_logger().info("="*60)
        self.get_logger().info(f"配置文件: {config_file}")
        self.get_logger().info(f"发布频率: {self.publish_rate} Hz")
        self.get_logger().info(f"发布话题: {self.gnss_pub_topic}")
        self.get_logger().info(f"经纬度: Lon={self.base_params['lon']}, Lat={self.base_params['lat']}")
        self.get_logger().info(f"朝向: 水平={self.base_params['horizontal_orientation']}°, 垂直={self.base_params['vertical_orientation']}°")
        self.get_logger().info(f"相机高度: {self.base_params['camera_height']} m")
        self.get_logger().info("="*60)

    def add_noise(self, base_value, noise_range):
        """为参数添加微小随机扰动，模拟真实设备误差"""
        return base_value + random.uniform(-noise_range, noise_range)

    def timer_callback(self):
        """定时发布相机参数消息"""
        msg = Gnss()
        

        # 填充参数 
        # msg.latitude = self.base_params['lat']  # 纬度
        # msg.longitude = self.base_params['lon']  # 经度
        # msg.horizontal_orientation = self.base_params['horizontal_orientation']  # 水平方向角
        # msg.vertical_orientation = self.base_params['vertical_orientation']  # 垂直方向
        # msg.camera_height = self.base_params['camera_height']  # 高度
        
        # # 填充参数（添加微小噪声模拟实时变化）
        msg.latitude = self.add_noise(self.base_params['lat'], 0.000002)  # 纬度误差±0.000002
        msg.longitude = self.add_noise(self.base_params['lon'], 0.000002)  # 经度误差±0.000002
        msg.horizontal_orientation = self.add_noise(self.base_params['horizontal_orientation'], 0.0002)  # 水平方向角±0.02度
        msg.vertical_orientation = self.add_noise(self.base_params['vertical_orientation'], 0.0002)  # 垂直方向角±0.02度
        msg.camera_height = self.add_noise(self.base_params['camera_height'], 0.0002)  # 高度±0.02米
        
        # 添加时间戳（当前ROS时间）
        msg.timestamp = self.get_clock().now().to_msg()
        
        # 发布消息
        self.publisher_.publish(msg)
        # 调试信息（可选）
        # self.get_logger().info(f"Published: Lon={msg.longitude:.6f}, Lat={msg.latitude:.6f}, in timestamp={msg.timestamp.sec}.{msg.timestamp.nanosec}")

def main(args=None):
    rclpy.init(args=args)
    node = GnssPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()