#!/usr/bin/env python3
# 核心功能：解析 nmea 话题中的 NMEA 句子，转换为结构化的 AIS 消息并发布。
# 关键逻辑：
# 订阅 nmea 话题，接收 nmea_msgs.msg.Sentence 消息，筛选出 AIS 专用的 !AIVDM 类型句子。
# 调用 ais.decoder 模块解码 NMEA payload，提取船舶位置、航向、航速、静态信息（船名、呼号等）等字段。
# 将解码结果封装为 marine_ais_msgs.msg.AIS 消息（包含导航、静态、 voyage 等子消息），发布到 messages 话题。
# 作用：实现从原始 NMEA 字符串到结构化 ROS 消息的转换，是数据处理的核心节点。

# run as: ros2 run ais_tools ais_parser

import rclpy
from rclpy.node import Node
from ais_interfaces.msg import AIS, Communication, Destination, DataLinkReservationBlock, Navigation, NavigationalStatus
from nmea_msgs.msg import Sentence

# import ais.decoder 
# # 因为decoder.py在ais_tools目录下,suoyi yao xiugaiwei
from ais_tools import decoder  # 两者都在 ais_tools 包根目录下

# import tf.transformations ROS1 version
import tf_transformations  # ROS2 version

import math
import datetime

class AISParserNode(Node):
    def __init__(self):
        super().__init__('ais_parser')
        
        # 创建发布者
        self.ais_pub = self.create_publisher(AIS, 'messages', 10)
        
        # 初始化AIS解码器
        self.ais_decoder = decoder.AISDecoder()
        
        # 创建订阅者
        self.nmea_sub = self.create_subscription(
            Sentence,
            'nmea',
            self.nmea_callback,
            10
        )
        self.get_logger().info('AIS parser node initialized')

    def nmea_callback(self, msg):
        # 输出接收到的NMEA句子（调试用）
        # self.get_logger().info(f'ais_parser nmea_callback')
        # self.get_logger().debug(f'Received NMEA sentence: {msg.sentence}')
        # 仅处理AIS相关的NMEA句子
        if msg.sentence.startswith('!AIVDM'):
            self.ais_decoder.addNMEA(msg.sentence)
            msgs = self.ais_decoder.popMessages()
            
            # 转换时间戳为UTC datetime
            receive_time = datetime.datetime.fromtimestamp(
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                tz=datetime.timezone.utc
            )
            
            for m in msgs:
                a = AIS()
                a.header = msg.header  # 直接复用原始消息的header
                
                # 填充基本信息
                a.message_id = m['message_id']
                a.repeat_indicator = m['repeat_indicator']
                a.id = m['id']

                # 处理导航信息
                if 'latitude' in m and 'longitude' in m and m['latitude'] is not None and m['longitude'] is not None:
                    a.navigation.pose.position.latitude = m['latitude']
                    a.navigation.pose.position.longitude = m['longitude']
                else:
                    a.navigation.pose.position.latitude = math.nan
                    a.navigation.pose.position.longitude = math.nan
                
                if 'altitude' in m and m['altitude'] is not None:
                    a.navigation.pose.position.altitude = m['altitude']
                else:
                    a.navigation.pose.position.altitude = math.nan

                if 'true_heading' in m and m['true_heading'] is not None:
                    yaw = math.radians(90.0 - m['true_heading'])
                    # q = tf.transformations.quaternion_from_euler(yaw, 0, 0, 'rzyx')
                    # ROS 2中tf_transformations.quaternion_from_euler参数与ROS 1完全兼容
                    q = tf_transformations.quaternion_from_euler(yaw, 0, 0, 'rzyx')
                    a.navigation.pose.orientation.x = q[0]
                    a.navigation.pose.orientation.y = q[1]
                    a.navigation.pose.orientation.z = q[2]
                    a.navigation.pose.orientation.w = q[3]

                # 处理转向率
                if 'rate_of_turn' in m:
                    if m['rate_of_turn'] is None:
                        a.navigation.twist.angular.z = math.nan
                        if 'rate_of_turn_status' in m:
                            a.navigation.rate_of_turn_status = m['rate_of_turn_status']
                        else:
                            a.navigation.rate_of_turn_status = Navigation.RATE_OF_TURN_UNAVAILABLE
                    else:
                        # 从度/分钟顺时针转换为弧度/秒逆时针
                        a.navigation.twist.angular.z = math.radians(-m['rate_of_turn'] / 60.0)
                        a.navigation.rate_of_turn_status = Navigation.RATE_OF_TURN_VALID
                else:
                    a.navigation.twist.angular.z = math.nan
                    a.navigation.rate_of_turn_status = Navigation.RATE_OF_TURN_UNAVAILABLE

                # 处理航速和航向
                if 'sog' in m and m['sog'] is not None and 'cog' in m and m['cog'] is not None:
                    sog_meters_per_second = m['sog'] * 0.514444
                    cog_ros = math.radians(90.0 - m['cog'])
                    a.navigation.twist.linear.x = math.cos(cog_ros) * sog_meters_per_second
                    a.navigation.twist.linear.y = math.sin(cog_ros) * sog_meters_per_second
                else:
                    a.navigation.twist.linear.x = math.nan
                    a.navigation.twist.linear.y = math.nan

                # 处理航行状态
                if 'navigational_status' in m:
                    a.navigation.navigational_status.status = m['navigational_status']
                else:
                    a.navigation.navigational_status.status = NavigationalStatus.NAVIGATIONAL_STATUS_UNDEFINED

                a.navigation.position_accuracy_high = 'position_accuracy' in m and m['position_accuracy'] == 1

                if 'time_stamp' in m:
                    a.navigation.time_stamp = m['time_stamp']
                else:
                    a.navigation.time_stamp = Navigation.TIME_STAMP_NOT_AVAIABLE

                if 'special_manoeuvre_indicator' in m:
                    a.navigation.navigational_status.special_manoeuvre = m['special_manoeuvre_indicator']

                a.navigation.raim_in_use = 'raim_flag' in m and m['raim_flag'] == 1

                if 'position_fixing_device_type' in m:
                    a.navigation.position_fixing_device_type = m['position_fixing_device_type']

                a.navigation.barometric_altitude = 'altitude_sensor' in m and m['altitude_sensor'] == 1

                if 'position_latency' in m:
                    a.navigation.position_latency = m['position_latency']
                a.navigation.position_latency = 1

                a.navigation.assigned_mode = 'assigned_mode_flag' in m and m['assigned_mode_flag'] == 1

                # 处理静态信息
                if 'ais_version' in m:
                    a.static_info.ais_version = m['ais_version']
                
                if 'imo_number' in m:
                    a.static_info.imo_number = m['imo_number']
                
                if 'callsign' in m:
                    a.static_info.callsign = m['callsign']
                
                if 'name' in m:
                    a.static_info.name = m['name']

                if 'ship_and_cargo_type' in m:
                    a.static_info.ship_and_cargo_type = m['ship_and_cargo_type']

                if 'reference_to_bow_distance' in m:
                    a.static_info.reference_to_bow_distance = m['reference_to_bow_distance']

                if 'reference_to_stern_distance' in m:
                    a.static_info.reference_to_stern_distance = m['reference_to_stern_distance']

                if 'reference_to_port_distance' in m:
                    a.static_info.reference_to_port_distance = m['reference_to_port_distance']

                if 'reference_to_starboard_distance' in m:
                    a.static_info.reference_to_starboard_distance = m['reference_to_starboard_distance']

                if 'maximum_present_static_draught' in m and m['maximum_present_static_draught'] is not None:
                    a.static_info.static_draught = m['maximum_present_static_draught']

                if 'dte' in m:
                    a.static_info.dte_ready = m['dte'] == 0
                else:
                    a.static_info.dte_ready = False

                # 处理航行信息
                eta_parts_available = False

                year = receive_time.year
                month = receive_time.month
                if 'eta_month' in m:
                    if m['eta_month'] > 0:
                        month = m['eta_month']  # 修复原代码中的==错误
                        if month < receive_time.month:  # 可能是下一年
                            year += 1
                        eta_parts_available = True
                day = receive_time.day
                if 'eta_day' in m:
                    if m['eta_day'] > 0:
                        day = m['eta_day']
                        eta_parts_available = True
                hour = 0
                if 'eta_hour' in m:
                    if m['eta_hour'] < 24:
                        hour = m['eta_hour']
                        eta_parts_available = True
                minute = 0
                if 'eta_minute' in m:
                    if m['eta_minute'] < 60:
                        minute = m['eta_minute']
                        eta_parts_available = True

                if eta_parts_available:
                    try:
                        eta_dt = datetime.datetime(year, month, day, hour, minute, tzinfo=datetime.timezone.utc)
                        a.voyage.estimated_time_of_arrival.sec = int(eta_dt.timestamp())
                        a.voyage.estimated_time_of_arrival.nanosec = 0
                    except ValueError:
                        self.get_logger().warn(f"无法解析ETA: {str(m)}")

                if 'destination' in m:
                    a.voyage.destination = m['destination']

                # 处理寻址信息
                if 'destination_id' in m:
                    d = Destination()
                    d.id = m['destination_id']
                    if 'sequence_number' in m:
                        d.sequence_number = m['sequence_number']
                    a.addressed.destinations.append(d)

                if 'destination_id1' in m:
                    d = Destination()
                    d.id = m['destination_id1']
                    if 'sequence_number_for_id1' in m:
                        d.sequence_number = m['sequence_number_for_id1']
                    if 'message_id1_1' in m:
                        d.requested_message_id = m['message_id1_1']
                    if 'slot_offset_1_1' in m:
                        d.slot_offset = m['slot_offset_1_1']
                    a.addressed.destinations.append(d)
                    if 'message_id1_2' in m:
                        d = Destination()
                        d.id = m['destination_id1']
                        d.requested_message_id = m['message_id1_2']
                        if 'slot_offset_1_2' in m:
                            d.slot_offset = m['slot_offset_1_2']
                        a.addressed.destinations.append(d)
                
                if 'destination_id2' in m:
                    d = Destination()
                    d.id = m['destination_id2']
                    if 'sequence_number_for_id2' in m:
                        d.sequence_number = m['sequence_number_for_id2']
                    if 'message_id2_1' in m:
                        d.requested_message_id = m['message_id2_1']
                    if 'slot_offset_2_1' in m:
                        d.slot_offset = m['slot_offset_2_1']
                    a.addressed.destinations.append(d)
                
                if 'destination_id3' in m:
                    d = Destination()
                    d.id = m['destination_id3']
                    if 'sequence_number_for_id3' in m:
                        d.sequence_number = m['sequence_number_for_id3']
                    a.addressed.destinations.append(d)

                if 'destination_id4' in m:
                    d = Destination()
                    d.id = m['destination_id4']
                    if 'sequence_number_for_id4' in m:
                        d.sequence_number = m['sequence_number_for_id4']
                    a.addressed.destinations.append(d)
                    
                if 'destination_id_a' in m:
                    d = Destination()
                    d.id = m['destination_id_a']
                    if 'offset_a' in m:
                        d.slot_offset = m['offset_a']
                    if 'increment_a' in m:
                        d.increment = m['increment_a']
                    a.addressed.destinations.append(d)
                
                if 'destination_id_b' in m:
                    d = Destination()
                    d.id = m['destination_id_b']
                    if 'offset_b' in m:
                        d.slot_offset = m['offset_b']
                    if 'increment_b' in m:
                        d.increment = m['increment_b']
                    a.addressed.destinations.append(d)

                a.addressed.retransmitted = 'retransmit_flag' in m and m['retransmit_flag'] == 1

                # 处理二进制数据
                if 'binary_data' in m:
                    b = m['binary_data']
                    if 'application_identifier' in b:
                        ai = b['application_identifier']
                        if 'designated_area_code' in ai:
                            a.binary.designated_area_code = ai['designated_area_code']
                        if 'function_identifier' in ai:
                            a.binary.function_identifier = ai['function_identifier']
                    if 'unstructured_data' in b:
                        a.binary.unstructured = True
                        a.binary.data = b['unstructured_data']
                    if 'differential_correction_data' in b:
                        a.binary.data = b['differential_correction_data']
                    if 'application_data' in b:
                        a.binary.data = b['application_data']

                # 处理B类信息
                a.class_b.cs_unit = 'class_b_unit_flag' in m and m['class_b_unit_flag'] == 1

                a.class_b.display_equipped = 'class_b_display_flag' in m and m['class_b_display_flag'] == 1

                a.class_b.dsc_equipped = 'class_b_dsc_flag' in m and m['class_b_dsc_flag'] == 1

                a.class_b.whole_band = 'class_b_band_flag' in m and m['class_b_band_flag'] == 1

                a.class_b.message_22_frequency_management = 'class_b_message_22_flag' in m and m['class_b_message_22_flag'] == 1

                if 'part_number' in m:
                    a.class_b.part_number = m['part_number']
                
                if 'vendor_id' in m:
                    vid = m['vendor_id']
                    if 'manufacturers_id' in vid:
                        a.class_b.manufacturer_id = vid['manufacturers_id']
                    if 'unit_model_code' in vid:
                        a.class_b.unit_model_code = vid['unit_model_code']
                    if 'unit_serial_number' in vid:
                        a.class_b.unit_serial_number = vid['unit_serial_number']

                # 处理数据链路预留块
                for n in ('1', '2', '3', '4'):
                    if f'offset_number_{n}' in m:
                        dlrb = DataLinkReservationBlock()
                        dlrb.offset_number = m[f'offset_number_{n}']

                        if f'number_of_slots_{n}' in m:
                            dlrb.number_of_slots = m[f'number_of_slots_{n}']
                        if f'timeout_{n}' in m:
                            dlrb.timeout.sec = 60 * m[f'timeout_{n}']
                        if f'increment_{n}' in m:
                            dlrb.increment = m[f'increment_{n}']
                        a.data_link_reservation_blocks.append(dlrb)

                # 处理航标信息
                if 'type_of_aids_to_navigation' in m:
                    a.aton.type = m['type_of_aids_to_navigation']
                
                if 'name_of_aids_navigation' in m:
                    a.static_info.name = m['name_of_aids_navigation']

                if 'name_of_aids_to_navigation_extension' in m:
                    a.static_info.name += m['name_of_aids_to_navigation_extension']

                if 'off_position_indicator' in m:
                    a.aton.off_position = m['off_position_indicator'] == 1

                if 'aton_status' in m:
                    a.aton.status = m['aton_status']

                if 'virtual_aton_flag' in m:
                    a.aton.isvirtual = m['virtual_aton_flag'] == 1

                # 处理通信信息
                if m['message_id'] in (1, 2, 4, 11):
                    a.communication.state = Communication.COMMUNICATION_STATE_SOTDMA
                elif m['message_id'] == 3:
                    a.communication.state = Communication.COMMUNICATION_STATE_ITDMA
                elif 'communication_state_selector_flag' in m:
                    a.communication.state = m['communication_state_selector_flag']
                
                if 'sotdma_sync_state' in m:
                    a.communication.sync_state = m['sotdma_sync_state']

                if 'sotdma_slot_timeout' in m:
                    a.communication.sotdma_slot_timeout = m['sotdma_slot_timeout']
                
                if 'sotdma_received_stations' in m:
                    a.communication.sotdma_received_stations = m['sotdma_received_stations']

                if 'sotdma_slot_number' in m:
                    a.communication.sotdma_slot_number = m['sotdma_slot_number']

                if 'sotdma_utc_hour' in m:
                    a.communication.sotdma_utc_hour = m['sotdma_utc_hour']

                if 'sotdma_utc_minute' in m:
                    a.communication.sotdma_utc_minute = m['sotdma_utc_minute']

                if 'sotdma_slot_offset' in m:
                    a.communication.sotdma_slot_offset = m['sotdma_slot_offset']

                if 'itdma_sync_state' in m:
                    a.communication.sync_state = m['itdma_sync_state']

                if 'itdma_slot_increment' in m:
                    a.communication.itdma_slot_increment = m['itdma_slot_increment']

                if 'itdma_number_of_slots' in m:
                    a.communication.itdma_number_of_slots = m['itdma_number_of_slots']

                if 'itdma_keep_flag' in m:
                    a.communication.itdma_keep = m['itdma_keep_flag'] == 1

                # 处理频道管理
                if 'utc_time' in m and m['utc_time'] is not None:
                    try:
                        a.utc_time.sec = int(m['utc_time'].timestamp())
                        a.utc_time.nanosec = 0
                    except TypeError:
                        self.get_logger().error(f'无效的utc_time: {str(m["utc_time"])}')

                a.long_range_transmission_control = 'long_range_transmission_control' in m and m['long_range_transmission_control'] == 1

                if 'safety_related_text' in m:
                    a.safety_related_text = m['safety_related_text']

                if 'channel' in m:
                    a.channel = m['channel']

                if 'nmea_payload' in m:
                    a.nmea_payload = m['nmea_payload']

                # 发布消息
                try:
                    self.ais_pub.publish(a)
                except Exception as e:  # ROS 2中序列化异常类型已改变
                    self.get_logger().error(f'发布消息失败: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = AISParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()