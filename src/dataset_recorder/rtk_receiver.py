#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RTK数据接收模块
负责从UDP端口接收并解析RTK AGRICA报文
"""

import socket
import threading
import queue
import time
import logging
import csv
from datetime import datetime, timezone
from typing import Dict, Optional, List
from dataclasses import dataclass
from pathlib import Path
import re

logger = logging.getLogger(__name__)


@dataclass
class RTKData:
    """RTK数据结构"""
    timestamp: float  # 系统时间戳
    utc_time: Optional[datetime]  # UTC时间（from AGRICA）
    latitude: Optional[float]  # 纬度（度）
    longitude: Optional[float]  # 经度（度）
    altitude: Optional[float]  # 海拔（米）
    heading: Optional[float]  # 航向角（度）
    pitch: Optional[float]  # 俯仰角（度）
    roll: Optional[float]  # 横滚角（度）
    speed: Optional[float]  # 速度（m/s）
    location_quality: Optional[int]  # 定位质量（0=无效,1=单点,2=伪距差分,4=固定解,5=浮点解）
    heading_quality: Optional[int]  # 定向质量（0=无效,1=单点,2=伪距差分,4=固定解,5=浮点解）
    satellites: Optional[int]  # 卫星数量
    hdop: Optional[float]  # 水平精度因子
    raw_message: str  # 原始AGRICA句子
    valid: bool  # 数据是否有效（location_quality>=2 and heading_quality>=2）


class RTKReceiver:
    """RTK数据接收器"""
    
    def __init__(self, config: Dict, save_path: Optional[str] = None):
        """
        初始化RTK接收器
        
        Args:
            config: RTK配置字典
            save_path: CSV文件保存路径（如果为None则不保存）
        """
        self.config = config
        self.socket = None
        self.running = False
        self.thread = None
        
        # 数据队列（线程安全）
        self.data_queue = queue.Queue(maxsize=100)
        
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
            'valid_data': 0,
            'invalid_location_quality': 0,
            'invalid_heading_quality': 0,
            'parse_errors': 0,
            'time_discontinuity': 0,
            'saved_count': 0,
            'csv_files': 0
        }
        self.stats_lock = threading.Lock()
        
        # 上一次有效时间（用于连续性检查）
        self.last_valid_time = None
        self.last_timestamp = None
        
        # 期望接收频率
        self.expected_frequency = config.get('expected_frequency', 5)  # Hz
        self.expected_interval = 1.0 / self.expected_frequency  # 秒
        
        # 创建保存目录
        if self.save_enabled:
            self.save_path.mkdir(parents=True, exist_ok=True)
            logger.info(f"RTK数据将保存到: {self.save_path}")
        
    def connect(self) -> bool:
        """
        绑定UDP端口
        
        Returns:
            绑定是否成功
        """
        try:
            udp_port = self.config['udp_port']
            bind_address = self.config.get('bind_address', '0.0.0.0')
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((bind_address, udp_port))
            self.socket.settimeout(1.0)
            
            logger.info(f"RTK UDP端口绑定成功: {bind_address}:{udp_port}")
            return True
            
        except socket.error as e:
            logger.error(f"RTK UDP端口绑定失败: {e}")
            return False
        except Exception as e:
            logger.error(f"RTK连接异常: {e}")
            return False
    
    def disconnect(self):
        """断开UDP连接"""
        if self.socket:
            try:
                self.socket.close()
                logger.info("RTK UDP端口已关闭")
            except Exception as e:
                logger.warning(f"关闭RTK UDP端口异常: {e}")
    
    def start(self) -> bool:
        """
        启动接收线程
        
        Returns:
            启动是否成功
        """
        if self.running:
            logger.warning("RTK接收器已在运行")
            return True
        
        if not self.socket:
            if not self.connect():
                return False
        
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        
        # 启动CSV保存线程
        if self.save_enabled:
            self.save_thread = threading.Thread(target=self._save_loop, daemon=True)
            self.save_thread.start()
            logger.info("RTK CSV保存线程已启动")
        
        logger.info("RTK接收线程已启动")
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
        logger.info("RTK接收线程已停止")
    
    def _receive_loop(self):
        """接收循环（在独立线程中运行）"""
        logger.info("RTK接收循环开始")
        
        while self.running:
            try:
                if not self.socket:
                    logger.warning("RTK UDP套接字未连接，尝试重连...")
                    if not self.connect():
                        time.sleep(5.0)
                        continue
                
                # 接收UDP数据
                try:
                    data, addr = self.socket.recvfrom(4096)
                    
                    # 解码为字符串
                    data_str = data.decode('utf-8', errors='ignore').strip()
                    
                    if data_str:
                        self._process_message(data_str)
                        
                except socket.timeout:
                    # 超时是正常的，继续等待
                    continue
                except socket.error as e:
                    logger.error(f"UDP接收错误: {e}")
                    time.sleep(1.0)
                    
            except Exception as e:
                logger.error(f"RTK接收循环异常: {e}")
                time.sleep(1.0)
        
        logger.info("RTK接收循环结束")
    
    def _process_message(self, message: str):
        """
        处理接收到的消息
        这里支持混合多种#开头报文，自动捕获#AGRICA完整消息
        """
        import re
        with self.stats_lock:
            self.stats['total_received'] += 1

        # 从原始消息中提取第一个#AGRICA...*消息
        agrica_match = re.search(r'#AGRICA[^*]*\*', message)
        if not agrica_match:
            logger.debug(f"未检测到#AGRICA报文（可能是其他类型消息），忽略该条。")
            return

        agrica_msg = agrica_match.group(0)

        # 解析AGRICA报文
        rtk_data = self._parse_agrica(agrica_msg)

        if not rtk_data:
            with self.stats_lock:
                self.stats['parse_errors'] += 1
            return

        # 验证定位质量和定向质量
        quality_threshold = self.config.get('quality_threshold', 1)
        
        # 检查定位质量
        if rtk_data.location_quality is None or rtk_data.location_quality < quality_threshold:
            with self.stats_lock:
                self.stats['invalid_location_quality'] += 1
                # 前5次失败时输出详细信息，之后只debug
                if self.stats['invalid_location_quality'] <= 5:
                    logger.warning(f"RTK定位质量不足: {rtk_data.location_quality} < {quality_threshold} "
                                 f"(阈值={quality_threshold}, 已过滤{self.stats['invalid_location_quality']}条)")
                else:
                    logger.debug(f"RTK定位质量不足: {rtk_data.location_quality} < {quality_threshold}")
            return
        
        # 检查定向质量
        if rtk_data.heading_quality is None or rtk_data.heading_quality < quality_threshold:
            with self.stats_lock:
                self.stats['invalid_heading_quality'] += 1
                # 前5次失败时输出详细信息，之后只debug
                if self.stats['invalid_heading_quality'] <= 5:
                    logger.warning(f"RTK定向质量不足: {rtk_data.heading_quality} < {quality_threshold} "
                                 f"(阈值={quality_threshold}, 已过滤{self.stats['invalid_heading_quality']}条)")
                else:
                    logger.debug(f"RTK定向质量不足: {rtk_data.heading_quality} < {quality_threshold}")
            return

        # 验证时间连续性
        if not self._validate_time_continuity(rtk_data):
            with self.stats_lock:
                self.stats['time_discontinuity'] += 1
            logger.warning("RTK时间不连续")

        # 记录第一条数据的UTC时间
        if self.first_data_time is None and rtk_data.utc_time:
            self.first_data_time = rtk_data.utc_time
            logger.info(f"RTK首条数据时间: {self.first_data_time.strftime('%Y_%m_%d_%H_%M_%S')}")
        
        # 添加到保存缓存
        if self.save_enabled:
            with self.data_cache_lock:
                self.current_second_data.append(rtk_data)

        # 入队
        self._enqueue_data(rtk_data)
        with self.stats_lock:
            self.stats['valid_data'] += 1

    def _parse_agrica(self, agrica_msg: str) -> Optional[RTKData]:
        """
        解析#AGRICA原始报文内容，字段提取方式参考rtk_parser_example.py
        
        字段位置参考：
        - [2-7]: 年月日时分秒
        - [8]: 定位质量
        - [10-12]: GPS/BDS/GLO卫星数
        - [19]: Heading（航向）
        - [20]: Pitch（俯仰角）
        - [21]: Roll（横滚角）
        - [22]: Speed（速度）
        - [29-31]: 纬度/经度/高程
        """
        import time
        try:
            # 必须有分号分隔符，否则格式错误
            if ';' not in agrica_msg:
                logger.debug("AGRICA报文格式错误，缺少';'")
                return None
            header_part, data_part = agrica_msg.split(';', 1)
            # 移除校验和
            if '*' in data_part:
                data_part = data_part.split('*', 1)[0]
            data_fields = data_part.split(',')
            if len(data_fields) < 32:  # 至少需要32个字段（到altitude）
                logger.debug(f"AGRICA报文字段数不足: {len(data_fields)}")
                return None
            
            # 时间相关字段解析
            try:
                year = int(data_fields[2])
                month = int(data_fields[3])
                day = int(data_fields[4])
                hour = int(data_fields[5])
                minute = int(data_fields[6])
                second = int(data_fields[7])
                utc_time = datetime(year, month, day, hour, minute, second)
            except Exception as e:
                logger.debug(f"UTC时间解析失败: {e}")
                utc_time = None
            
            # 位置信息（纬度/经度/高程）
            try:
                latitude = float(data_fields[29])
                longitude = float(data_fields[30])
                altitude = float(data_fields[31])
            except Exception as e:
                logger.debug(f"位置信息解析失败: {e}")
                latitude = longitude = altitude = None
            
            # 姿态信息（航向/俯仰/横滚）
            try:
                heading = float(data_fields[19]) if data_fields[19] else None
            except Exception:
                heading = None
            
            try:
                pitch = float(data_fields[20]) if data_fields[20] else None
            except Exception:
                pitch = None
            
            try:
                roll = float(data_fields[21]) if data_fields[21] else None
            except Exception:
                roll = None
            
            # 速度信息
            try:
                speed = float(data_fields[22]) if data_fields[22] else None
            except Exception:
                speed = None
            
            # 定位质量
            try:
                location_quality = int(data_fields[8]) if data_fields[8] else None
            except Exception:
                location_quality = None

            #定向质量
            try:
                heading_quality = int(data_fields[9]) if data_fields[9] else None
            except Exception:
                heading_quality = None

            # 卫星数量（GPS + BDS + GLO）
            try:
                gps_sats = int(data_fields[10]) if data_fields[10] else 0
                bds_sats = int(data_fields[11]) if data_fields[11] else 0
                glo_sats = int(data_fields[12]) if data_fields[12] else 0
                satellites = gps_sats + bds_sats + glo_sats
            except Exception:
                satellites = None
            
            # HDOP（这里用Baseline_NStd作为近似）
            try:
                hdop = float(data_fields[16]) if data_fields[16] else None
            except Exception:
                hdop = None
            
            rtk_data = RTKData(
                timestamp=time.time(),
                utc_time=utc_time,
                latitude=latitude,
                longitude=longitude,
                altitude=altitude,
                heading=heading,
                pitch=pitch,
                roll=roll,
                speed=speed,
                location_quality=location_quality,
                heading_quality=heading_quality,
                satellites=satellites,
                hdop=hdop,
                raw_message=agrica_msg,
                valid=(location_quality is not None and location_quality >= 1 and heading_quality is not None and heading_quality >= 1)
            )
            return rtk_data
        except Exception as e:
            logger.debug(f"AGRICA解析失败: {e}")
            return None
    
    def _parse_utc_time(self, time_str: str) -> Optional[datetime]:
        """解析UTC时间"""
        try:
            if not time_str or len(time_str) < 6:
                return None
            
            # 时间格式: HHMMSS.ss
            hour = int(time_str[0:2])
            minute = int(time_str[2:4])
            second = int(time_str[4:6])
            microsecond = 0
            
            if '.' in time_str:
                frac = time_str.split('.')[1]
                microsecond = int(frac.ljust(6, '0')[:6])
            
            # 使用当前日期
            now = datetime.now(timezone.utc)
            utc_time = datetime(
                now.year, now.month, now.day,
                hour, minute, second, microsecond
            )
            
            return utc_time
            
        except Exception as e:
            logger.debug(f"UTC时间解析失败: {e}")
            return None
    
    def _parse_latitude(self, lat_str: str, direction: str) -> Optional[float]:
        """解析纬度（DDMM.mmmmmm格式）"""
        try:
            if not lat_str:
                return None
            
            # 格式: DDMM.mmmmmm
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
        """解析经度（DDDMM.mmmmmm格式）"""
        try:
            if not lon_str:
                return None
            
            # 格式: DDDMM.mmmmmm
            degrees = int(lon_str[:3])
            minutes = float(lon_str[3:])
            
            longitude = degrees + minutes / 60.0
            
            if direction == 'W':
                longitude = -longitude
            
            return longitude
            
        except Exception as e:
            logger.debug(f"经度解析失败: {e}")
            return None
    
    def _validate_time_continuity(self, rtk_data: RTKData) -> bool:
        """
        验证时间连续性
        
        Args:
            rtk_data: RTK数据
            
        Returns:
            时间是否连续
        """
        if not self.last_timestamp or not rtk_data.timestamp:
            self.last_timestamp = rtk_data.timestamp
            self.last_valid_time = rtk_data.utc_time
            return True
        
        # 计算时间差
        time_diff = rtk_data.timestamp - self.last_timestamp
        
        # 期望时间间隔（例如5Hz = 0.2秒）
        # 允许±50%的误差
        expected = self.expected_interval
        min_interval = expected * 0.5
        max_interval = expected * 1.5
        
        # 检查是否在合理范围内
        is_continuous = (min_interval <= time_diff <= max_interval)
        
        # 更新上次时间
        self.last_timestamp = rtk_data.timestamp
        if rtk_data.valid:
            self.last_valid_time = rtk_data.utc_time
        
        return is_continuous
    
    def _enqueue_data(self, rtk_data: RTKData):
        """将数据加入队列"""
        try:
            self.data_queue.put(rtk_data, block=False)
        except queue.Full:
            logger.warning("RTK数据队列已满，丢弃旧数据")
            try:
                self.data_queue.get_nowait()
                self.data_queue.put(rtk_data, block=False)
            except queue.Empty:
                pass
    
    def get_data(self, timeout: Optional[float] = None) -> Optional[RTKData]:
        """
        从队列获取数据
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            RTK数据对象
        """
        try:
            return self.data_queue.get(timeout=timeout)
        except queue.Empty:
            # 只在debug级别输出，避免日志过多
            logger.debug("RTK数据队列为空，返回None")
            return None
        except Exception as e:
            logger.error(f"RTK数据队列获取数据异常: {e}")
            return None
    
    def get_statistics(self) -> Dict:
        """获取统计信息"""
        with self.stats_lock:
            return self.stats.copy()
    
    def get_last_valid_time(self) -> Optional[datetime]:
        """获取上次有效UTC时间"""
        return self.last_valid_time
    
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
        logger.info("RTK CSV保存线程已启动，等待相机就绪...")
        
        # 等待相机开始录制
        self.start_saving_event.wait()
        logger.info("RTK开始保存数据")
        
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
                logger.error(f"RTK CSV保存循环异常: {e}")
                time.sleep(1.0)
        
        logger.info("RTK CSV保存循环结束")
    
    def _save_to_csv(self, data_list: List[RTKData]):
        """
        将数据列表保存为CSV文件
        
        Args:
            data_list: RTK数据列表
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
            
            filename = f"rtk_{time_str}.csv"
            filepath = self.save_path / filename
            
            # 检查文件是否存在
            file_exists = filepath.exists()
            
            # 写入CSV
            with open(filepath, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                
                # 如果是新文件，写入表头
                if not file_exists:
                    writer.writerow([
                        'timestamp', 'utc_time', 'latitude', 'longitude',
                        'heading', 'pitch', 'roll', 'altitude', 'speed',
                        'location_quality', 'heading_quality', 'satellites', 'hdop', 'raw_message'
                    ])
                    with self.stats_lock:
                        self.stats['csv_files'] += 1
                
                # 写入数据行
                for rtk_data in data_list:
                    writer.writerow([
                        f"{rtk_data.timestamp:.6f}",
                        rtk_data.utc_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] if rtk_data.utc_time else '',
                        f"{rtk_data.latitude:.8f}" if rtk_data.latitude is not None else '',
                        f"{rtk_data.longitude:.8f}" if rtk_data.longitude is not None else '',
                        f"{rtk_data.heading:.2f}" if rtk_data.heading is not None else '',
                        f"{rtk_data.pitch:.2f}" if rtk_data.pitch is not None else '',
                        f"{rtk_data.roll:.2f}" if rtk_data.roll is not None else '',
                        f"{rtk_data.altitude:.3f}" if rtk_data.altitude is not None else '',
                        f"{rtk_data.speed:.2f}" if rtk_data.speed is not None else '',
                        rtk_data.location_quality if rtk_data.location_quality is not None else '',
                        rtk_data.heading_quality if rtk_data.heading_quality is not None else '',
                        rtk_data.satellites if rtk_data.satellites is not None else '',
                        f"{rtk_data.hdop:.2f}" if rtk_data.hdop is not None else '',
                        rtk_data.raw_message
                    ])
            
            with self.stats_lock:
                self.stats['saved_count'] += len(data_list)
            
            logger.debug(f"保存RTK数据: {len(data_list)}条 -> {filename}")
            
        except Exception as e:
            logger.error(f"保存RTK CSV失败: {e}")
    
    def _flush_remaining_data(self):
        """
        保存剩余的数据（程序停止时调用）
        """
        with self.data_cache_lock:
            if self.current_second_data:
                logger.info(f"刷新剩余RTK数据: {len(self.current_second_data)}条")
                self._save_to_csv(self.current_second_data)
                self.current_second_data = []


# 测试代码
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 测试配置
    test_config = {
        'udp_port': 8010,
        'bind_address': '0.0.0.0',
        'expected_frequency': 5,
        'quality_threshold': 1
    }
    
    receiver = RTKReceiver(test_config)
    
    if receiver.start():
        print("RTK接收器已启动，按Ctrl+C停止...")
        
        try:
            while True:
                data = receiver.get_data(timeout=1.0)
                if data:
                    print(f"收到RTK数据:")
                    print(f"  UTC时间: {data.utc_time}")
                    print(f"  位置: Lat={data.latitude:.6f}°, Lon={data.longitude:.6f}°, Alt={data.altitude:.2f}m")
                    print(f"  姿态: Heading={data.heading:.2f}°, Pitch={data.pitch:.2f}°, Roll={data.roll:.2f}°")
                    print(f"  速度: {data.speed:.2f}m/s")
                    print(f"  质量: LocationQuality={data.location_quality}, HeadingQuality={data.heading_quality}, Sats={data.satellites}")
                    print()
                
                # 每5秒打印统计
                stats = receiver.get_statistics()
                if stats['total_received'] % 25 == 0 and stats['total_received'] > 0:
                    print(f"统计: {stats}")
                    
        except KeyboardInterrupt:
            print("\n停止接收...")
            receiver.stop()
            
            # 打印最终统计
            print(f"\n最终统计: {receiver.get_statistics()}")
    else:
        print("RTK接收器启动失败")

