import rclpy
from rclpy.node import Node
from marnav_interfaces.msg import Ais, AisVis
from builtin_interfaces.msg import Time
import pandas as pd
import numpy as np
from utils.AIS_utils import transform  # 复用原坐标转换逻辑
from utils.file_read import read_all  # 用于读取相机参数

class AisTransformNode(Node):
    def __init__(self):
        super().__init__('ais_vis_node')
        
        # 声明参数：数据路径（用于读取相机参数）
        self.declare_parameter('cameras_path', './src/marnav_vis/config/camera_0_param_location.txt,./src/marnav_vis/config/camera_1_param_location.txt,./src/marnav_vis/config/camera_2_param_location.txt') #
        cameras_path = self.get_parameter('cameras_path').get_parameter_value().string_value.split(',')
        
        # 读取相机参数（从原项目的camera_para.txt中）
        self.camera_params = []
        self.cam_names = ['rtsp_image_0', 'rtsp_image_1', 'rtsp_image_2']  # 相机名称列表
        for path in cameras_path:
            para =  read_all(path)
            self.camera_params.append(para)
            self.get_logger().info(f"Loaded camera parameters for {self.cam_names[len(self.camera_params)-1]}: {para}")
        
        # 图像尺寸（根据实际相机参数或配置设置,假设所有相机分辨率相同，取第一个相机参数）
        self.im_shape = [2*self.camera_para[0][9],2*self.camera_para[0][10]]
        self.get_logger().info(f"Image shape set to: {self.im_shape}")

        # 维护AIS数据状态（用于轨迹跟踪和转换）
        self.AIS_vis = pd.DataFrame(columns=[
            'camname','mmsi', 'lon', 'lat', 'speed', 'course', 'heading', 'type', 'timestamp', 'x', 'y'
        ])
        
        # 创建AIS消息订阅者
        self.ais_subscriber = self.create_subscription(
            Ais,
            'ais_raw_data',  # 原始AIS数据话题
            self.ais_callback,
            10  # 队列长度
        )
        
        # 创建AIS带图像坐标的发布者
        self.ais_vis_publisher = self.create_publisher(
            AisVis,
            'ais_vis_data',  # 带图像坐标的AIS话题
            10  # 队列长度
        )
        
        self.get_logger().info("AIS transform_visulazition node initialized")

    def ais_callback(self, msg: Ais):
        """处理原始AIS消息，转换为带图像坐标的消息并发布"""
        # 1. 将ROS消息转换为DataFrame（适配原transform函数输入格式）
        ais_row = {
            'mmsi': msg.mmsi,
            'lon': msg.lon,
            'lat': msg.lat,
            'speed': msg.speed,
            'course': msg.course,
            'heading': msg.heading,
            'type': msg.type,
            # 转换ROS时间戳为毫秒级时间戳（原代码使用的格式）
            'timestamp': msg.timestamp.sec * 1000 + msg.timestamp.nanosec // 1000000
        }

        # 在终端输出消息内容
        print(f"收到AIS消息：")
        print(f"  MMSI: {ais_row['mmsi']}")
        print(f"  经度: {ais_row['lon']}")
        print(f"  纬度: {ais_row['lat']}")
        print(f"  速度: {ais_row['speed']}")
        print(f"  航向: {ais_row['course']}")
        print(f"  船头方向: {ais_row['heading']}")
        print(f"  船只类型: {ais_row['type']}")
        print(f"  时间戳(ms): {ais_row['timestamp']}")
        print("----------------------------------------")  # 分隔线，便于区分多条消息
        ais_current = pd.DataFrame([ais_row])
        
        
        # 2. 调用原坐标转换函数计算图像坐标
        self.AIS_vis = transform(ais_current, self.AIS_vis, self.camera_para, self.im_shape)
        
        # 3. 提取当前处理的AIS数据对应的图像坐标
        current_vis = self.AIS_vis[
            (self.AIS_vis['mmsi'] == msg.mmsi) & 
            (self.AIS_vis['timestamp'] == ais_row['timestamp'])
        ].iloc[-1]  # 取最新一行
        
        # 4. 构造AisVis消息
        ais_vis_msg = AisVis()
        ais_vis_msg.camname = current_vis['camname']
        ais_vis_msg.mmsi = msg.mmsi
        ais_vis_msg.lon = msg.lon
        ais_vis_msg.lat = msg.lat
        ais_vis_msg.speed = msg.speed
        ais_vis_msg.course = msg.course
        ais_vis_msg.heading = msg.heading
        ais_vis_msg.type = msg.type
        ais_vis_msg.x = int(current_vis['x'])
        ais_vis_msg.y = int(current_vis['y'])
        ais_vis_msg.timestamp = msg.timestamp  # 保持原始时间戳
        
        # 5. 发布带图像坐标的AIS消息
        self.ais_vis_publisher.publish(ais_vis_msg)
        self.get_logger().debug(
            f"Transformed AIS (MMSI: {msg.mmsi}) to image coordinates: ({ais_vis_msg.x}, {ais_vis_msg.y})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = AisTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Received keyboard interrupt, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# double rmLong=report->get_lon().value().get(); //目标船舶经度
# double rmLatitude=report->get_lat().value().get(); //目标船舶纬度
# const Geodesic& geod = Geodesic::WGS84(); //使用WGS84坐标系
# double s12,a1,a2;

# geod.Inverse(localLat_,localLon_,rmLatitude,rmLong,s12,a1,a2); //// 计算本地位置（localLat, localLon）与目标船舶位置的距离和方位角

# std::cout<<"两点之间的椭球面距离"<<s12<<"米"<<", 起点->终点的正方位角"<<a1<<"度, 终点->起点的反方位角"<<a2<<"度"<<std::endl;
# // s12; //两点之间的椭球面距离,单位通常为米（m）
# // a1; //起点（本地位置）-> 终点（目标船舶位置）的正方位角,单位为弧度（rad）
# // a2; //终点（目标船舶位置）-> 起点（本地位置）的反方位角,单位为弧度（rad）

# example of output in chinese:
# // [INFO] [1756544179.842737818] [raw_ais_parser]: 3接收到数据: !AIVDM,1,1,,A,16:cCV000S8cOQPAs7:C0P0V0000,0*61
# // 解析为 VDM 句子: VDM
# // 收到 VDM 句子: 片段 1/1
# // 所有 VDM 片段已收集完成，开始解析 AIS 消息...
# // 提取并组合 AIS 有效载荷完成，开始生成 AIS 消息对象...
# // AIS 消息类型: 位置报告（Class A），开始提取具体数据...
# // AIS 位置报告: MMSI=413848472, 经度=121.349, 纬度=31.3238, 速度=3.5, 相对于地面的实际运动航向=77, 相对于磁北的航向0
# // 两点之间的椭球面距离2581.06米, 起点->终点的正方位角-114.885度, 终点->起点的反方位角-114.897度

# // [INFO] [1756544279.881303167] [raw_ais_parser]: 3接收到数据: !AIVDM,1,1,,A,16:cCV000U8cP4vAs86Bj01n0000,0*3A
# // 解析为 VDM 句子: VDM
# // 收到 VDM 句子: 片段 1/1
# // 所有 VDM 片段已收集完成，开始解析 AIS 消息...
# // 提取并组合 AIS 有效载荷完成，开始生成 AIS 消息对象...
# // AIS 消息类型: 位置报告（Class A），开始提取具体数据...
# // AIS 位置报告: MMSI=413848472, 经度=121.351, 纬度=31.3242, 速度=3.7, 相对于地面的实际运动航向=71.2, 相对于磁北的航向0
# // 两点之间的椭球面距离2399.31米, 起点->终点的正方位角-115.734度, 终点->起点的反方位角-115.746度