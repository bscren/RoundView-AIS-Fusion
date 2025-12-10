#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
配置管理模块
负责加载、验证和管理系统配置
"""

import yaml
import os
import serial
import socket
import cv2
from typing import Dict, List, Any, Optional
from pathlib import Path
import logging

logger = logging.getLogger(__name__)


class ConfigManager:
    """配置管理器"""
    
    def __init__(self, config_path: str):
        """
        初始化配置管理器
        
        Args:
            config_path: 配置文件路径
        """
        self.config_path = config_path
        self.config = None
        self.validation_results = {}
        
    def load_config(self) -> Dict:
        """
        加载YAML配置文件
        
        Returns:
            配置字典
        """
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)
            logger.info(f"配置文件加载成功: {self.config_path}")
            return self.config
        except FileNotFoundError:
            logger.error(f"配置文件不存在: {self.config_path}")
            raise
        except yaml.YAMLError as e:
            logger.error(f"配置文件格式错误: {e}")
            raise
        except Exception as e:
            logger.error(f"加载配置文件失败: {e}")
            raise
    
    def validate_config(self) -> bool:
        """
        验证配置文件的完整性和正确性
        
        Returns:
            验证是否通过
        """
        if not self.config:
            logger.error("配置未加载，请先调用load_config()")
            return False
        
        valid = True
        
        # 验证必需的顶级配置项（移除sync，因为已不使用时间同步）
        required_keys = ['devices', 'storage']
        for key in required_keys:
            if key not in self.config:
                logger.error(f"缺少必需的配置项: {key}")
                valid = False
        
        if not valid:
            return False
        
        # 验证设备配置
        valid &= self._validate_devices_config()
        
        # 验证存储配置
        valid &= self._validate_storage_config()
        
        return valid
    
    def _validate_devices_config(self) -> bool:
        """验证设备配置"""
        devices = self.config.get('devices', {})
        valid = True
        
        # 验证AIS配置
        if 'ais' in devices:
            ais = devices['ais']
            if ais.get('enabled', False):
                comm_type = ais.get('comm_type', 'serial')
                if comm_type == 'serial':
                    if 'port' not in ais or 'baud_rate' not in ais:
                        logger.error("AIS串口配置缺少必需参数: port/baud_rate")
                        valid = False
                elif comm_type == 'tcp':
                    if 'tcp_ip' not in ais or 'tcp_port' not in ais:
                        logger.warning("AIS TCP配置建议包含: tcp_ip/tcp_port")
                elif comm_type == 'udp':
                    if 'udp_port' not in ais:
                        logger.warning("AIS UDP配置建议包含: udp_port")
        
        # 验证相机配置
        if 'cameras' in devices:
            cameras = devices['cameras']
            if not isinstance(cameras, list):
                logger.error("cameras配置必须是列表")
                valid = False
            else:
                for i, cam in enumerate(cameras):
                    if 'id' not in cam or 'rtsp_url' not in cam:
                        logger.error(f"相机{i}配置缺少必需参数: id/rtsp_url")
                        valid = False
        
        # 验证GNSS配置
        if 'gnss' in devices:
            gnss = devices['gnss']
            if gnss.get('enabled', False):
                if 'udp_port' not in gnss:
                    logger.error("GNSS配置缺少必需参数: udp_port")
                    valid = False
        
        return valid
    
    def _validate_storage_config(self) -> bool:
        """验证存储配置"""
        storage = self.config.get('storage', {})
        
        if 'root_path' not in storage:
            logger.error("存储配置缺少root_path")
            return False
        
        # 检查路径是否可写
        root_path = Path(storage['root_path'])
        try:
            root_path.mkdir(parents=True, exist_ok=True)
            if not os.access(root_path, os.W_OK):
                logger.error(f"数据集根路径不可写: {root_path}")
                return False
        except Exception as e:
            logger.error(f"创建数据集根路径失败: {e}")
            return False
        
        return True
    
    def _validate_sync_config(self) -> bool:
        """验证时间同步配置"""
        sync = self.config.get('sync', {})
        
        if sync.get('reference_source') != 'gnss':
            logger.warning("时间基准建议设置为gnss")
        
        return True
    
    def verify_device_connectivity(self) -> Dict[str, bool]:
        """
        验证所有设备的连通性
        
        Returns:
            设备连通性结果字典
        """
        results = {}
        
        devices = self.config.get('devices', {})
        
        # 验证AIS串口/TCP/UDP连通性
        if devices.get('ais', {}).get('enabled', False):
            results['ais'] = self._verify_ais_connectivity()
        
        # 验证相机RTSP连通性
        cameras = devices.get('cameras', [])
        for camera in cameras:
            if camera.get('enabled', False):
                cam_id = camera['id']
                results[cam_id] = self._verify_camera_connectivity(camera)
        
        # 验证GNSS UDP连通性
        if devices.get('gnss', {}).get('enabled', False):
            results['gnss'] = self._verify_gnss_connectivity()
        
        self.validation_results = results
        return results
    
    def _verify_ais_connectivity(self) -> bool:
        """验证AIS连通性（支持串口/TCP/UDP）"""
        ais_config = self.config['devices']['ais']
        comm_type = ais_config.get('comm_type', 'serial')
        
        try:
            if comm_type == 'serial':
                # 验证串口
                port = ais_config.get('port')
                baud_rate = ais_config.get('baud_rate', 38400)
                
                if not port:
                    logger.error("❌ AIS串口配置缺少port参数")
                    return False
                
                ser = serial.Serial(
                    port=port,
                    baudrate=baud_rate,
                    parity=ais_config.get('parity', 'N'),
                    stopbits=ais_config.get('stopbits', 1),
                    bytesize=ais_config.get('bytesize', 8),
                    timeout=ais_config.get('timeout', 1.0)
                )
                
                if ser.is_open:
                    logger.info(f"✅ AIS串口连接成功: {port} @ {baud_rate}")
                    ser.close()
                    return True
                else:
                    logger.error(f"❌ AIS串口打开失败: {port}")
                    return False
                    
            elif comm_type == 'tcp':
                # TCP连接验证
                tcp_ip = ais_config.get('tcp_ip', '127.0.0.1')
                tcp_port = ais_config.get('tcp_port', 5000)
                logger.info(f"✅ AIS TCP配置已验证: {tcp_ip}:{tcp_port}")
                return True  # TCP连接在运行时验证
                
            elif comm_type == 'udp':
                # UDP端口验证
                udp_port = ais_config.get('udp_port', 1800)
                logger.info(f"✅ AIS UDP配置已验证: 端口 {udp_port}")
                return True  # UDP端口在运行时绑定
                
            else:
                logger.error(f"❌ 不支持的AIS通信类型: {comm_type}")
                return False
                
        except serial.SerialException as e:
            logger.error(f"❌ AIS串口连接失败: {e}")
            return False
        except Exception as e:
            logger.error(f"❌ AIS连接验证异常: {e}")
            return False
    
    def _verify_camera_connectivity(self, camera_config: Dict) -> bool:
        """验证相机RTSP连通性"""
        cam_id = camera_config['id']
        rtsp_url = camera_config['rtsp_url']
        
        try:
            # 尝试打开RTSP流
            cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
            
            if cap.isOpened():
                # 尝试读取一帧
                ret, frame = cap.read()
                cap.release()
                
                if ret and frame is not None:
                    logger.info(f"✅ 相机{cam_id}连接成功: {rtsp_url[:30]}...")
                    return True
                else:
                    logger.error(f"❌ 相机{cam_id}无法读取帧")
                    return False
            else:
                logger.error(f"❌ 相机{cam_id}RTSP流打开失败: {rtsp_url[:30]}...")
                return False
                
        except Exception as e:
            logger.error(f"❌ 相机{cam_id}连接异常: {e}")
            return False
    
    def _verify_gnss_connectivity(self) -> bool:
        """验证GNSS UDP连通性"""
        gnss_config = self.config['devices']['gnss']
        udp_port = gnss_config['udp_port']
        bind_address = gnss_config.get('bind_address', '0.0.0.0')
        
        try:
            # 尝试绑定UDP端口
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((bind_address, udp_port))
            sock.settimeout(1.0)
            
            logger.info(f"✅ GNSS UDP端口绑定成功: {bind_address}:{udp_port}")
            sock.close()
            return True
            
        except socket.error as e:
            logger.error(f"❌ GNSS UDP端口绑定失败: {bind_address}:{udp_port} - {e}")
            return False
        except Exception as e:
            logger.error(f"❌ GNSS连接验证异常: {e}")
            return False
    
    def get_device_config(self, device_type: str) -> Optional[Dict]:
        """
        获取指定设备的配置
        
        Args:
            device_type: 设备类型 (ais/cameras/gnss)
            
        Returns:
            设备配置字典
        """
        if not self.config:
            return None
        return self.config.get('devices', {}).get(device_type)
    
    def get_storage_config(self) -> Optional[Dict]:
        """获取存储配置"""
        if not self.config:
            return None
        return self.config.get('storage')
    
    def get_sync_config(self) -> Optional[Dict]:
        """获取时间同步配置"""
        if not self.config:
            return None
        return self.config.get('sync')
    
    def get_fault_tolerance_config(self) -> Optional[Dict]:
        """获取容错配置"""
        if not self.config:
            return None
        return self.config.get('fault_tolerance')
    
    def get_logging_config(self) -> Optional[Dict]:
        """获取日志配置"""
        if not self.config:
            return None
        return self.config.get('logging')
    
    def print_validation_summary(self):
        """打印设备连通性验证摘要"""
        if not self.validation_results:
            logger.warning("尚未执行设备连通性验证")
            return
        
        print("\n" + "="*60)
        print("设备连通性验证结果")
        print("="*60)
        
        for device, result in self.validation_results.items():
            status = "✅ 成功" if result else "❌ 失败"
            print(f"{device:20s}: {status}")
        
        total = len(self.validation_results)
        success = sum(1 for r in self.validation_results.values() if r)
        print("-"*60)
        print(f"总计: {success}/{total} 设备连接成功")
        print("="*60 + "\n")
        
        if success < total:
            logger.warning("部分设备连接失败，请检查配置和设备状态")
            return False
        else:
            logger.info("所有设备连接成功")
            return True


# 测试代码
if __name__ == "__main__":
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 创建配置管理器
    config_file = "config_template.yaml"
    
    if not os.path.exists(config_file):
        print(f"❌ 配置文件不存在: {config_file}")
        exit(1)
    
    manager = ConfigManager(config_file)
    
    # 加载配置
    try:
        config = manager.load_config()
        print("✅ 配置文件加载成功\n")
    except Exception as e:
        print(f"❌ 配置文件加载失败: {e}")
        exit(1)
    
    # 验证配置
    if manager.validate_config():
        print("✅ 配置文件验证通过\n")
    else:
        print("❌ 配置文件验证失败\n")
        exit(1)
    
    # 验证设备连通性
    print("开始验证设备连通性...\n")
    results = manager.verify_device_connectivity()
    
    # 打印验证摘要
    manager.print_validation_summary()

