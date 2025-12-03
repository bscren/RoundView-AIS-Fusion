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

class GnssPublisher(Node):
    def __init__(self):
        super().__init__('gnss_publisher_node')
        
        # 1. 声明参数：发布帧率（默认10Hz）
        self.declare_parameter('publish_rate', 10.0)  # 单位：Hz
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.timer_period = 1.0 / self.publish_rate  # 定时器周期（秒）
        
        # 2. 创建发布者
        self.publisher_ = self.create_publisher(
            Gnss,
            '/gnss_topic',  # 发布话题名称
            10  # 队列大小
        )
        
        # 3. 初始化相机参数（基于示例值，可添加微小扰动模拟实时数据）
        self.base_params = {
            'lon': 114.32583,
            'lat': 30.60139,
            'horizontal_orientation': 352.0,
            'vertical_orientation': -4.0,
            'camera_height': 20.0
        }
        
        # 4. 创建定时器，按指定帧率发布数据
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info(f"GNSS Camera Publisher started. Publish rate: {self.publish_rate} Hz")

    def add_noise(self, base_value, noise_range):
        """为参数添加微小随机扰动，模拟真实设备误差"""
        return base_value + random.uniform(-noise_range, noise_range)

    def timer_callback(self):
        """定时发布相机参数消息"""
        msg = Gnss()
        

        # 填充参数 
        msg.latitude = self.base_params['lat']  # 纬度
        msg.longitude = self.base_params['lon']  # 经度
        msg.horizontal_orientation = self.base_params['horizontal_orientation']  # 水平方向角
        msg.vertical_orientation = self.base_params['vertical_orientation']  # 垂直方向
        msg.camera_height = self.base_params['camera_height']  # 高度
        
        # # 填充参数（添加微小噪声模拟实时变化）
        # msg.latitude = self.add_noise(self.base_params['lat'], 0.000002)  # 纬度误差±0.000002
        # msg.longitude = self.add_noise(self.base_params['lon'], 0.000002)  # 经度误差±0.000002
        # msg.horizontal_orientation = self.add_noise(self.base_params['horizontal_orientation'], 0.0002)  # 水平方向角±0.02度
        # msg.vertical_orientation = self.add_noise(self.base_params['vertical_orientation'], 0.0002)  # 垂直方向角±0.02度
        # msg.camera_height = self.add_noise(self.base_params['camera_height'], 0.0002)  # 高度±0.02米
        
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