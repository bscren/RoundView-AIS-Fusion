#!/usr/bin/env python3
# ais_contact_tracker.py
# 核心功能：跟踪 AIS 目标（船舶或航标），维护目标状态并发布聚合后的跟踪结果。
# 关键逻辑：
# 订阅 messages 话题，接收 ais_msgs.msg.AIS 消息。
# 对船舶目标（消息 ID 1/2/3/5/9/18/19/24）：维护 contacts 字典存储目标状态，聚合静态信息（船名、尺寸）和动态信息（位置、速度），生成 AISContact 消息发布到 contacts 话题。
# 对航标目标（消息 ID 21）：提取位置信息，封装为 geographic_msgs.msg.GeoPointStamped 消息发布到 atons 话题。
# 作用：持续跟踪目标状态，为上层应用（如船舶监控、避障）提供统一的目标数据接口。

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ais_interfaces.msg import AIS, AISContact
from geometry_msgs.msg import Polygon, Point32
from geographic_msgs.msg import GeoPointStamped

import copy

class AisContactTracker(Node):
    def __init__(self):
        super().__init__('ais_contact_tracker')
        
        self.get_logger().info('AisContactTracker node started.')
        self.contacts_pub = self.create_publisher(AISContact, 'contacts', 10)
        # 用于追踪Mesobot，使用AIS信标作为航标发射
        # 将frame_id设置为id(mmsi)作为临时解决方案
        self.aton_pub = self.create_publisher(GeoPointStamped, 'atons', 10)
        
        self.contacts = {}
        
        self.ais_message_sub = self.create_subscription(
            AIS,
            'messages',
            self.ais_callback,
            10
        )

    def calculate_polygon(self, static):
        # REP 103: 相对于船体的标准坐标系：
        # x向前，y向左，z向上
        bow = Point32()
        bow.x = static.reference_to_bow_distance
        width = static.reference_to_port_distance + static.reference_to_starboard_distance
        half_width = width / 2.0
        bow.y = half_width - static.reference_to_port_distance
        length = static.reference_to_bow_distance + static.reference_to_stern_distance
        
        port_pointy_start = Point32()
        port_pointy_start.x = bow.x - (length * 0.1)
        port_pointy_start.y = static.reference_to_port_distance
        
        starboard_pointy_start = Point32()
        starboard_pointy_start.x = port_pointy_start.x
        starboard_pointy_start.y = -static.reference_to_starboard_distance
        
        aft_port = Point32()
        aft_port.x = -static.reference_to_stern_distance
        aft_port.y = static.reference_to_port_distance
        
        aft_starboard = Point32()
        aft_starboard.x = aft_port.x
        aft_starboard.y = -static.reference_to_starboard_distance

        footprint = Polygon()
        footprint.points.append(bow)
        footprint.points.append(port_pointy_start)
        footprint.points.append(aft_port)
        footprint.points.append(aft_starboard)
        footprint.points.append(starboard_pointy_start)
        footprint.points.append(bow)
        
        return footprint

    def ais_callback(self, msg):
        # 调试部分
        self.get_logger().debug(f'ais_contact_tracker ais_callback')
        # self.get_logger().debug(f'Received AIS message: ID={msg.message_id}, MMSI={msg.id}')
            # 仅处理特定的消息ID
        if msg.message_id in (1, 2, 3, 5, 9, 18, 19, 24):
            if msg.id not in self.contacts:
                self.contacts[msg.id] = AISContact()
                self.contacts[msg.id].id = msg.id
            
            if msg.message_id in (5, 24):  # 静态/航行数据
                if msg.message_id == 5:
                    self.contacts[msg.id].static_info = copy.deepcopy(msg.static_info)
                    self.contacts[msg.id].voyage = copy.deepcopy(msg.voyage)
                else:
                    if msg.class_b.part_number == 0:  # 24A
                        self.contacts[msg.id].static_info.name = msg.static_info.name
                        self.contacts[msg.id].footprint = self.calculate_polygon(self.contacts[msg.id].static_info)
                    else:  # 24B
                        self.contacts[msg.id].static_info.callsign = msg.static_info.callsign
                        self.contacts[msg.id].static_info.ship_and_cargo_type = msg.static_info.ship_and_cargo_type
                        self.contacts[msg.id].static_info.reference_to_bow_distance = msg.static_info.reference_to_bow_distance
                        self.contacts[msg.id].static_info.reference_to_stern_distance = msg.static_info.reference_to_stern_distance
                        self.contacts[msg.id].static_info.reference_to_port_distance = msg.static_info.reference_to_port_distance
                        self.contacts[msg.id].static_info.reference_to_starboard_distance = msg.static_info.reference_to_starboard_distance
                        self.contacts[msg.id].footprint = self.calculate_polygon(self.contacts[msg.id].static_info)
            
            if msg.message_id in (1, 2, 3, 9, 18, 19):
                self.contacts[msg.id].header = msg.header
                self.contacts[msg.id].position_message_id = msg.message_id
                self.contacts[msg.id].pose = msg.navigation.pose
                self.contacts[msg.id].twist.twist = msg.navigation.twist
                self.contacts[msg.id].navigational_status = msg.navigation.navigational_status
                # 待办：处理协方差
                
                self.contacts_pub.publish(self.contacts[msg.id])
        
        if msg.message_id == 21:
            # 航标信息
            aton = GeoPointStamped()
            aton.header.stamp = msg.header.stamp
            aton.header.frame_id = str(msg.id)
            aton.position = msg.navigation.pose.position
            self.aton_pub.publish(aton)

def main(args=None):
    rclpy.init(args=args)
    node = AisContactTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()