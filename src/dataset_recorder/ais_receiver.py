#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AIS数据接收模块
负责从串口/TCP/UDP接收并解析AIS NMEA报文
"""

import serial
import socket
import threading
import queue
import time
import logging
import csv
from datetime import datetime, timezone
from typing import Dict, Optional, List
from pathlib import Path
import re
from dataclasses import dataclass
import math
from abc import ABC, abstractmethod
from pyais import decode

logger = logging.getLogger(__name__)


@dataclass
class AISData:
    """AIS数据结构"""
    timestamp: float  # 系统时间戳
    utc_time: Optional[datetime]  # UTC时间（from GPRMC）
    mmsi: Optional[int]  # 船舶MMSI
    latitude: Optional[float]  # 纬度
    longitude: Optional[float]  # 经度
    speed: Optional[float]  # 速度（节）
    course: Optional[float]  # 航向（度）
    heading: Optional[float]  # 头向（度）
    message_type: str  # 消息类型（AIVDM/GPRMC/GPGGA）
    raw_message: str  # 原始NMEA句子
    valid: bool  # 数据是否有效



class CommInterface(ABC):
    """通信接口抽象基类"""
    
    @abstractmethod
    def connect(self) -> bool:
        """建立连接"""
        pass
    
    @abstractmethod
    def disconnect(self):
        """断开连接"""
        pass
    
    @abstractmethod
    def is_connected(self) -> bool:
        """检查连接状态"""
        pass
    
    @abstractmethod
    def read_data(self) -> str:
        """读取数据"""
        pass


class SerialComm(CommInterface):
    """串口通信实现"""
    
    def __init__(self, config: Dict):
        """
        初始化串口通信
        
        Args:
            config: 串口配置字典
        """
        self.config = config
        self.serial_port = None
        
    def connect(self) -> bool:
        """连接串口"""
        try:
            self.serial_port = serial.Serial(
                port=self.config['port'],
                baudrate=self.config.get('baud_rate', 38400),
                parity=self.config.get('parity', 'N'),
                stopbits=self.config.get('stopbits', 1),
                bytesize=self.config.get('bytesize', 8),
                timeout=self.config.get('timeout', 1.0)
            )
            
            if self.serial_port.is_open:
                logger.info(f"AIS串口连接成功: {self.config['port']}")
                return True
            else:
                logger.error(f"AIS串口打开失败: {self.config['port']}")
                return False
                
        except serial.SerialException as e:
            logger.error(f"AIS串口连接失败: {e}")
            return False
        except Exception as e:
            logger.error(f"AIS连接异常: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                logger.info("AIS串口已关闭")
            except Exception as e:
                logger.warning(f"关闭AIS串口异常: {e}")
    
    def is_connected(self) -> bool:
        """检查串口连接状态"""
        return self.serial_port is not None and self.serial_port.is_open
    
    def read_data(self) -> str:
        """从串口读取数据"""
        try:
            if self.serial_port and self.serial_port.in_waiting > 0:
                line = self.serial_port.readline()
                return line.decode('utf-8', errors='ignore').strip()
            return ""
        except Exception as e:
            logger.error(f"串口读取错误: {e}")
            return ""


class TcpComm(CommInterface):
    """TCP通信实现"""
    
    def __init__(self, config: Dict):
        """
        初始化TCP通信
        
        Args:
            config: TCP配置字典
        """
        self.config = config
        self.socket = None
        self.buffer = ""
        
    def connect(self) -> bool:
        """连接TCP服务器"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.config.get('timeout', 1.0))
            
            ip = self.config.get('tcp_ip', '127.0.0.1')
            port = self.config.get('tcp_port', 5000)
            
            self.socket.connect((ip, port))
            logger.info(f"AIS TCP连接成功: {ip}:{port}")
            return True
            
        except socket.error as e:
            logger.error(f"AIS TCP连接失败: {e}")
            return False
        except Exception as e:
            logger.error(f"AIS TCP连接异常: {e}")
            return False
    
    def disconnect(self):
        """断开TCP连接"""
        if self.socket:
            try:
                self.socket.close()
                logger.info("AIS TCP连接已关闭")
            except Exception as e:
                logger.warning(f"关闭AIS TCP连接异常: {e}")
            finally:
                self.socket = None
    
    def is_connected(self) -> bool:
        """检查TCP连接状态"""
        return self.socket is not None
    
    def read_data(self) -> str:
        """从TCP读取数据（按行）"""
        try:
            if not self.socket:
                return ""
            
            # 接收数据
            data = self.socket.recv(4096)
            if not data:
                return ""
            
            # 解码并添加到缓冲区
            self.buffer += data.decode('utf-8', errors='ignore')
            
            # 提取完整的一行（以换行符分隔）
            if '\n' in self.buffer:
                line, self.buffer = self.buffer.split('\n', 1)
                return line.strip()
            
            return ""
            
        except socket.timeout:
            return ""
        except socket.error as e:
            logger.error(f"TCP读取错误: {e}")
            return ""
        except Exception as e:
            logger.error(f"TCP读取异常: {e}")
            return ""


class UdpComm(CommInterface):
    """UDP通信实现"""
    
    def __init__(self, config: Dict):
        """
        初始化UDP通信
        
        Args:
            config: UDP配置字典
        """
        self.config = config
        self.socket = None
        
    def connect(self) -> bool:
        """绑定UDP端口"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(self.config.get('timeout', 1.0))
            
            bind_ip = self.config.get('bind_address', '0.0.0.0')
            port = self.config.get('udp_port', 1800)
            
            self.socket.bind((bind_ip, port))
            logger.info(f"AIS UDP端口绑定成功: {bind_ip}:{port}")
            return True
            
        except socket.error as e:
            logger.error(f"AIS UDP端口绑定失败: {e}")
            return False
        except Exception as e:
            logger.error(f"AIS UDP连接异常: {e}")
            return False
    
    def disconnect(self):
        """关闭UDP套接字"""
        if self.socket:
            try:
                self.socket.close()
                logger.info("AIS UDP套接字已关闭")
            except Exception as e:
                logger.warning(f"关闭AIS UDP套接字异常: {e}")
            finally:
                self.socket = None
    
    def is_connected(self) -> bool:
        """检查UDP套接字状态"""
        return self.socket is not None
    
    def read_data(self) -> str:
        """从UDP读取数据"""
        try:
            if not self.socket:
                return ""
            
            data, addr = self.socket.recvfrom(4096)
            if data:
                return data.decode('utf-8', errors='ignore').strip()
            return ""
            
        except socket.timeout:
            return ""
        except socket.error as e:
            logger.error(f"UDP读取错误: {e}")
            return ""
        except Exception as e:
            logger.error(f"UDP读取异常: {e}")
            return ""


class AISReceiver:
    """AIS数据接收器"""
    
    def __init__(self, config: Dict, save_path: Optional[str] = None):
        """
        初始化AIS接收器
        
        Args:
            config: AIS配置字典
            save_path: CSV文件保存路径（如果为None则不保存）
        """
        self.config = config
        self.comm_interface = None
        self.running = False
        self.thread = None
        
        # 数据队列（线程安全）
        self.data_queue = queue.Queue(maxsize=config.get('max_queue_size', 100))
        
        # VDM片段缓存（用于多片段AIS消息）
        self.vdm_fragments = []
        self.vdm_lock = threading.Lock()
        
        # CSV保存相关
        self.save_path = Path(save_path) if save_path else None
        self.save_enabled = save_path is not None
        self.current_second_data = []  # 当前秒的数据缓存
        self.data_cache_lock = threading.Lock()
        self.save_thread = None
        self.first_data_time = None  # 记录第一条数据的UTC时间
        self.start_saving_event = threading.Event()  # 等待相机就绪后开始保存
        
        # 统计信息
        self.stats = {
            'total_received': 0,
            'valid_ais': 0,
            'invalid': 0,
            'saved_count': 0,
            'csv_files': 0
        }
        self.stats_lock = threading.Lock()
        
        # 初始化通信接口
        self._init_comm_interface()
        
        # 创建保存目录
        if self.save_enabled:
            self.save_path.mkdir(parents=True, exist_ok=True)
            logger.info(f"AIS数据将保存到: {self.save_path}")
        
        logger.info(f"AIS接收器初始化，通信方式: {self.config.get('comm_type', 'serial')}")
    
    def _init_comm_interface(self):
        """初始化通信接口"""
        comm_type = self.config.get('comm_type', 'serial')
        
        if comm_type == 'serial':
            self.comm_interface = SerialComm(self.config)
            logger.info(f"选择串口通信: {self.config.get('port', 'N/A')}, 波特率: {self.config.get('baud_rate', 38400)}")
        elif comm_type == 'tcp':
            self.comm_interface = TcpComm(self.config)
            logger.info(f"选择TCP通信: {self.config.get('tcp_ip', '127.0.0.1')}:{self.config.get('tcp_port', 5000)}")
        elif comm_type == 'udp':
            self.comm_interface = UdpComm(self.config)
            logger.info(f"选择UDP通信: {self.config.get('bind_address', '0.0.0.0')}:{self.config.get('udp_port', 1800)}")
        else:
            logger.error(f"不支持的通信类型: {comm_type}")
            self.comm_interface = None
    
    def connect(self) -> bool:
        """
        建立连接
        
        Returns:
            连接是否成功
        """
        if not self.comm_interface:
            logger.error("通信接口未初始化")
            return False
        
        return self.comm_interface.connect()
    
    def disconnect(self):
        """断开连接"""
        if self.comm_interface:
            self.comm_interface.disconnect()
    
    def start(self) -> bool:
        """
        启动接收线程
        
        Returns:
            启动是否成功
        """
        if self.running:
            logger.warning("AIS接收器已在运行")
            return True
        
        if not self.comm_interface or not self.comm_interface.is_connected():
            if not self.connect():
                return False
        
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        
        # 启动CSV保存线程
        if self.save_enabled:
            self.save_thread = threading.Thread(target=self._save_loop, daemon=True)
            self.save_thread.start()
            logger.info("AIS CSV保存线程已启动")
        
        logger.info("AIS接收线程已启动")
        return True
    
    def stop(self):
        """停止接收线程"""
        if not self.running:
            return
        
        self.running = False
        
        if self.thread:
            self.thread.join(timeout=5.0)
        
        # 停止保存线程并保存剩余数据
        if self.save_thread:
            self.save_thread.join(timeout=5.0)
            self._flush_remaining_data()
        
        self.disconnect()
        logger.info("AIS接收线程已停止")
    
    def _receive_loop(self):
        """接收循环（在独立线程中运行）"""
        logger.info("AIS接收循环开始")
        
        while self.running:
            try:
                if not self.comm_interface or not self.comm_interface.is_connected():
                    logger.warning("AIS通信未连接，尝试重连...")
                    if not self.connect():
                        time.sleep(5.0)
                        continue
                
                # 读取数据
                data_str = self.comm_interface.read_data()
                
                if data_str:
                    self._process_nmea_sentence(data_str)
                else:
                    time.sleep(0.01)  # 避免CPU占用过高
                    
            except Exception as e:
                logger.error(f"AIS接收循环异常: {e}")
                time.sleep(1.0)
        
        logger.info("AIS接收循环结束")
    
    def _process_nmea_sentence(self, sentence: str):
        """
        处理NMEA句子
        
        Args:
            sentence: NMEA句子字符串
        """
        with self.stats_lock:
            self.stats['total_received'] += 1
        
        # 过滤调试信息
        if sentence.startswith('$SYDBG'):
            return
        
        # 识别消息类型
        # 仅处理AIVDM消息（其他船只的AIS信息）
        if '!AIVDM' in sentence:
            self._process_aivdm(sentence)
    
    def _process_aivdm(self, sentence: str):
        """
        处理AIVDM句子（AIS船舶信息）
        
        Args:
            sentence: AIVDM NMEA句子
        """
        try:
            # 解析VDM句子格式: !AIVDM,片段总数,片段编号,消息ID,通道,有效载荷,填充位*校验和
            parts = sentence.split(',')
            if len(parts) < 6:
                return
            
            n_fragments = int(parts[1])  # 片段总数
            fragment_num = int(parts[2])  # 当前片段编号
            message_id = parts[3] if len(parts) > 3 else ''
            channel = parts[4] if len(parts) > 4 else ''
            
            # 单片段直接处理
            if n_fragments == 1 and fragment_num == 1:
                payload = parts[5].split('*')[0]
                
                ais_data = self._decode_ais_payload(payload, sentence)
                
                if ais_data and ais_data.valid:
                    self._enqueue_data(ais_data)
                    with self.stats_lock:
                        self.stats['valid_ais'] += 1
                return
            
            # 多片段组合
            with self.vdm_lock:
                if fragment_num < 1 or fragment_num > n_fragments:
                    logger.debug(f"无效片段编号: {fragment_num}/{n_fragments}，清空缓存")
                    self.vdm_fragments.clear()
                    return
                
                # 新消息第一片清空旧缓存
                if fragment_num == 1:
                    self.vdm_fragments.clear()
                
                self.vdm_fragments.append((fragment_num, sentence))
                logger.debug(f"缓存VDM片段: {fragment_num}/{n_fragments}，已收集 {len(self.vdm_fragments)}")
                
                # 片段未收齐
                if fragment_num != n_fragments or len(self.vdm_fragments) < n_fragments:
                    return
                
                # 按片段顺序组合
                self.vdm_fragments.sort(key=lambda x: x[0])
                payload_parts = []
                fill_bits = 0
                
                for idx, frag_sentence in self.vdm_fragments:
                    frag_parts = frag_sentence.split(',')
                    if len(frag_parts) < 6:
                        logger.debug(f"片段字段不足，放弃本条AIS: {frag_sentence}")
                        self.vdm_fragments.clear()
                        return
                    
                    payload_parts.append(frag_parts[5].split('*')[0])
                    
                    # 最后一段决定fill bits
                    if idx == n_fragments and len(frag_parts) > 6:
                        try:
                            fill_bits = int(frag_parts[6].split('*')[0])
                        except Exception:
                            fill_bits = 0
                
                combined_payload = ''.join(payload_parts)
                
                # 构造单条完整VDM句子并计算校验和
                nmea_body = f"AIVDM,1,1,{message_id},{channel},{combined_payload},{fill_bits}"
                checksum = 0
                for ch in nmea_body:
                    checksum ^= ord(ch)
                combined_sentence = f"!{nmea_body}*{checksum:02X}"
                
                # 清空缓存准备下一条
                self.vdm_fragments.clear()
            
            ais_data = self._decode_ais_payload(combined_payload, combined_sentence)
            
            if ais_data and ais_data.valid:
                self._enqueue_data(ais_data)
                with self.stats_lock:
                    self.stats['valid_ais'] += 1
                
        except Exception as e:
            logger.debug(f"AIVDM解析失败: {e}")
            with self.stats_lock:
                self.stats['invalid'] += 1
    
    def _decode_ais_payload(self, payload: str, raw_sentence: str) -> Optional[AISData]:
        """
        解码AIS有效载荷
        
        Args:
            payload: AIS有效载荷
            raw_sentence: 原始NMEA句子
            
        Returns:
            AIS数据对象
        """
        # 这里返回一个基础的AIS数据结构
        try:
            decoded_data = decode(raw_sentence)

            # 如果decoded_data.msg_type不为1、2、3，则返回None
            if decoded_data.msg_type not in [1, 2, 3]:
                logger.debug(f"该报文的子类型不需要解码: {decoded_data.msg_type}")
                return None
            
            # MessageType1(msg_type=1, repeat=0, mmsi=367380120, status=<NavigationStatus.UnderWayUsingEngine: 0>, turn=None, speed=0.1, accuracy=False, lon=-122.404333, lat=37.806948, course=245.2, heading=511, second=59, maneuver=0, spare_1=b'\x00', raim=True, radio=34958)
            current_utc = datetime.now(timezone.utc)
            
            ais_data = AISData(
                timestamp=time.time(),
                utc_time=current_utc,
                mmsi=decoded_data.mmsi,
                latitude=decoded_data.lat,
                longitude=decoded_data.lon,
                speed=decoded_data.speed,
                course=decoded_data.course,
                heading=decoded_data.heading,
                message_type='AIVDM',
                raw_message=raw_sentence,
                valid=True
            )
            
            # 记录第一条数据的UTC时间
            if self.first_data_time is None:
                self.first_data_time = current_utc
                logger.info(f"AIS首条数据时间: {current_utc.strftime('%Y_%m_%d_%H_%M_%S')}")
            
            # 添加到保存缓存
            if self.save_enabled:
                with self.data_cache_lock:
                    self.current_second_data.append(ais_data)
            
            return ais_data
        except Exception as e:
            logger.debug(f"AIS解码失败，或该报文的子类型不需要解码: {e}")
            return None
    
    def _parse_utc_time(self, time_str: str, date_str: Optional[str] = None) -> Optional[datetime]:
        """解析UTC时间"""
        try:
            if not time_str:
                return None
            
            # 时间格式: HHMMSS.sss
            hour = int(time_str[0:2])
            minute = int(time_str[2:4])
            second = int(time_str[4:6])
            microsecond = 0
            
            if '.' in time_str:
                frac = time_str.split('.')[1]
                microsecond = int(frac.ljust(6, '0')[:6])
            
            # 日期格式: DDMMYY
            if date_str and len(date_str) == 6:
                day = int(date_str[0:2])
                month = int(date_str[2:4])
                year = 2000 + int(date_str[4:6])
            else:
                # 使用当前日期
                now = datetime.now(timezone.utc)
                day = now.day
                month = now.month
                year = now.year
            
            return datetime(year, month, day, hour, minute, second, microsecond)
            
        except Exception as e:
            logger.debug(f"UTC时间解析失败: {e}")
            return None
    
    def _parse_latitude(self, lat_str: str, direction: str) -> Optional[float]:
        """解析纬度（DDMM.mmmm格式）"""
        try:
            if not lat_str:
                return None
            
            # 格式: DDMM.mmmm
            degrees = int(lat_str[:2])
            minutes = float(lat_str[2:])
            
            latitude = degrees + minutes / 60.0
            
            if direction == 'S':
                latitude = -latitude
            
            return latitude
            
        except Exception as e:
            logger.debug(f"纬度解析失败: {e}")
            return None
    
    def _parse_longitude(self, lon_str: str, direction: str) -> Optional[float]:
        """解析经度（DDDMM.mmmm格式）"""
        try:
            if not lon_str:
                return None
            
            # 格式: DDDMM.mmmm
            degrees = int(lon_str[:3])
            minutes = float(lon_str[3:])
            
            longitude = degrees + minutes / 60.0
            
            if direction == 'W':
                longitude = -longitude
            
            return longitude
            
        except Exception as e:
            logger.debug(f"经度解析失败: {e}")
            return None
    
    
    def _enqueue_data(self, ais_data: AISData):
        """将数据加入队列"""
        try:
            self.data_queue.put(ais_data, block=False)
        except queue.Full:
            logger.warning("AIS数据队列已满，丢弃旧数据")
            try:
                self.data_queue.get_nowait()  # 移除最旧的数据
                self.data_queue.put(ais_data, block=False)
            except queue.Empty:
                pass
    
    def get_data(self, timeout: Optional[float] = None) -> Optional[AISData]:
        """
        从队列获取数据
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            AIS数据对象
        """
        try:
            return self.data_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_statistics(self) -> Dict:
        """获取统计信息"""
        with self.stats_lock:
            return self.stats.copy()
    
    def clear_queue(self):
        """清空数据队列"""
        while not self.data_queue.empty():
            try:
                self.data_queue.get_nowait()
            except queue.Empty:
                break
    
    def _save_loop(self):
        """
        CSV保存循环（每秒触发一次）
        """
        logger.info("AIS CSV保存线程已启动，等待相机就绪...")
        
        # 等待相机开始录制
        self.start_saving_event.wait()
        logger.info("AIS开始保存数据")
        
        while self.running:
            try:
                time.sleep(1.0)  # 每秒触发一次
                
                # 获取当前秒的数据
                with self.data_cache_lock:
                    if not self.current_second_data:
                        continue
                    
                    data_to_save = self.current_second_data.copy()
                    self.current_second_data = []
                
                # 保存到CSV
                if data_to_save:
                    self._save_to_csv(data_to_save)
                    
            except Exception as e:
                logger.error(f"AIS CSV保存循环异常: {e}")
                time.sleep(1.0)
        
        logger.info("AIS CSV保存循环结束")
    
    def _save_to_csv(self, data_list: List[AISData]):
        """
        将数据列表保存为CSV文件
        
        Args:
            data_list: AIS数据列表
        """
        if not data_list or not self.save_path:
            return
        
        try:
            # 使用第一条数据的UTC时间作为文件名
            first_data = data_list[0]
            if first_data.utc_time:
                time_str = first_data.utc_time.strftime('%Y_%m_%d_%H_%M_%S')
            else:
                time_str = datetime.now(timezone.utc).strftime('%Y_%m_%d_%H_%M_%S')
            
            filename = f"ais_{time_str}.csv"
            filepath = self.save_path / filename
            
            # 检查文件是否存在
            file_exists = filepath.exists()
            
            # 写入CSV
            with open(filepath, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                
                # 如果是新文件，写入表头
                if not file_exists:
                    writer.writerow([
                        'timestamp', 'utc_time', 'mmsi', 'latitude', 'longitude',
                        'speed', 'course', 'message_type', 'raw_message'
                    ])
                    with self.stats_lock:
                        self.stats['csv_files'] += 1
                
                # 写入数据行
                for ais_data in data_list:
                    writer.writerow([
                        f"{ais_data.timestamp:.6f}",
                        ais_data.utc_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] if ais_data.utc_time else '',
                        ais_data.mmsi if ais_data.mmsi else '',
                        f"{ais_data.latitude:.8f}" if ais_data.latitude else '',
                        f"{ais_data.longitude:.8f}" if ais_data.longitude else '',
                        f"{ais_data.speed:.2f}" if ais_data.speed else '',
                        f"{ais_data.course:.2f}" if ais_data.course else '',
                        ais_data.message_type,
                        ais_data.raw_message
                    ])
            
            with self.stats_lock:
                self.stats['saved_count'] += len(data_list)
            
            logger.debug(f"保存AIS数据: {len(data_list)}条 -> {filename}")
            
        except Exception as e:
            logger.error(f"保存AIS CSV失败: {e}")
    
    def _flush_remaining_data(self):
        """
        保存剩余的数据（程序停止时调用）
        """
        with self.data_cache_lock:
            if self.current_second_data:
                logger.info(f"刷新剩余AIS数据: {len(self.current_second_data)}条")
                self._save_to_csv(self.current_second_data)
                self.current_second_data = []


# 测试代码
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 测试配置 - 串口
    test_config_serial = {
        'comm_type': 'serial',
        'port': '/dev/ttyS3',
        'baud_rate': 38400,
        'parity': 'N',
        'stopbits': 1,
        'bytesize': 8,
        'timeout': 1.0,
        'max_distance_km': 10.0,
        'max_queue_size': 100
    }
    
    # 测试配置 - TCP
    test_config_tcp = {
        'comm_type': 'tcp',
        'tcp_ip': '127.0.0.1',
        'tcp_port': 5000,
        'timeout': 1.0,
        'max_distance_km': 10.0,
        'max_queue_size': 100
    }
    
    # 测试配置 - UDP
    test_config_udp = {
        'comm_type': 'udp',
        'bind_address': '0.0.0.0',
        'udp_port': 1800,
        'timeout': 1.0,
        'max_distance_km': 10.0,
        'max_queue_size': 100
    }
    
    # 选择测试配置（根据实际情况修改）
    test_config = test_config_udp  # 或 test_config_tcp 或 test_config_udp
    
    receiver = AISReceiver(test_config)
    
    if receiver.start():
        print(f"AIS接收器已启动（{test_config['comm_type']}），按Ctrl+C停止...")
        
        try:
            while True:
                data = receiver.get_data(timeout=1.0)
                if data:
                    print(f"收到数据: {data.message_type} - {data.raw_message[:50]}...")
                
                # 每5秒打印统计
                stats = receiver.get_statistics()
                if stats['total_received'] % 50 == 0 and stats['total_received'] > 0:
                    print(f"统计: {stats}")
                    
        except KeyboardInterrupt:
            print("\n停止接收...")
            receiver.stop()
            
            # 打印最终统计
            print(f"\n最终统计: {receiver.get_statistics()}")
    else:
        print("AIS接收器启动失败")

