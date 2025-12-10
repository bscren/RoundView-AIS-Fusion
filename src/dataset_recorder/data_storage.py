#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据存储模块
负责结构化存储AIS、GNSS、相机数据
"""

import os
import cv2
import json
import csv
import threading
import queue
import time
import logging
from pathlib import Path
from datetime import datetime, timezone
from typing import Dict, List, Optional
from dataclasses import asdict
import numpy as np

logger = logging.getLogger(__name__)


class DataStorage:
    """数据存储管理器"""
    
    def __init__(self, config: Dict):
        """
        初始化数据存储管理器
        
        Args:
            config: 存储配置字典
        """
        self.config = config
        
        # 存储路径
        self.root_path = Path(config['root_path'])
        self.current_clip_path = None
        
        # 图像配置
        self.image_format = config.get('image_format', 'jpg')
        self.image_quality = config.get('image_quality', 95)
        self.sample_rate = config.get('sample_rate', 1)
        
        # 缓存配置
        cache_config = config.get('cache', {})
        self.max_queue_size = cache_config.get('max_queue_size', 100)
        self.batch_write_size = cache_config.get('batch_write_size', 50)
        self.batch_write_interval = cache_config.get('batch_write_interval', 5.0)
        self.expire_time = cache_config.get('expire_time', 300)
        
        # 分桶缓存（按数据类型）
        self.gnss_cache = []  # GNSS数据缓存
        self.ais_cache = []  # AIS数据缓存
        self.camera_cache = {}  # 相机帧缓存 {camera_id: [frames]}
        
        # 缓存锁
        self.cache_lock = threading.Lock()
        
        # 批量写入线程
        self.running = False
        self.write_thread = None
        
        # 统计信息
        self.stats = {
            'total_clips': 0,
            'gnss_saved': 0,
            'ais_saved': 0,
            'frames_saved': 0,
            'batches_written': 0
        }
        self.stats_lock = threading.Lock()
        
        # 当前秒的数据缓存（用于每秒保存）
        self.current_second_gnss = []
        self.current_second_ais = []
        self.last_save_time = None
        
        # 采样计数器
        self.frame_counters = {}  # {camera_id: counter}
        
        logger.info(f"数据存储管理器初始化完成: {self.root_path}")
    
    def create_clip(self, timestamp: Optional[datetime] = None) -> str:
        """
        创建新的数据包目录
        
        目录结构：clip_年_月_日_时_分_秒/
        
        Args:
            timestamp: UTC时间戳（from GNSS），如果为None则使用系统时间
            
        Returns:
            clip目录路径
        """
        if timestamp is None:
            timestamp = datetime.now(timezone.utc)
        
        # 生成clip目录名：clip_年_月_日_时_分_秒
        clip_name = f"clip_{timestamp.strftime('%Y_%m_%d_%H_%M_%S')}"
        clip_path = self.root_path / clip_name
        
        # 创建子目录结构
        subdirs = ['gnss', 'ais', 'camera', 'metadata']
        
        try:
            for subdir in subdirs:
                (clip_path / subdir).mkdir(parents=True, exist_ok=True)
            
            self.current_clip_path = clip_path
            
            logger.info(f"✅ 创建数据包目录: {clip_path}")
            
            with self.stats_lock:
                self.stats['total_clips'] += 1
            
            return str(clip_path)
            
        except Exception as e:
            logger.error(f"❌ 创建数据包目录失败: {e}")
            raise
    
    def create_camera_subdirs(self, camera_ids: List[str]):
        """
        为每个相机创建子目录
        
        Args:
            camera_ids: 相机ID列表
        """
        if not self.current_clip_path:
            logger.error("尚未创建clip目录")
            return
        
        camera_dir = self.current_clip_path / 'camera'
        
        for cam_id in camera_ids:
            cam_path = camera_dir / cam_id
            cam_path.mkdir(parents=True, exist_ok=True)
            
            # 初始化采样计数器
            self.frame_counters[cam_id] = 0
        
        logger.info(f"为{len(camera_ids)}个相机创建子目录")
    
    def add_gnss_data(self, gnss_data):
        """
        添加GNSS数据到缓存
        
        Args:
            gnss_data: GNSS数据对象
        """
        with self.cache_lock:
            self.current_second_gnss.append(gnss_data)
    
    def add_ais_data(self, ais_data):
        """
        添加AIS数据到缓存
        
        Args:
            ais_data: AIS数据对象
        """
        with self.cache_lock:
            self.current_second_ais.append(ais_data)
    
    def add_camera_frame(self, camera_frame):
        """
        添加相机帧到缓存
        
        Args:
            camera_frame: CameraFrame对象
        """
        cam_id = camera_frame.camera_id
        
        # 检查采样率
        if cam_id not in self.frame_counters:
            self.frame_counters[cam_id] = 0
        
        self.frame_counters[cam_id] += 1
        
        # 根据采样率决定是否保存
        if self.frame_counters[cam_id] % self.sample_rate != 0:
            return
        
        with self.cache_lock:
            if cam_id not in self.camera_cache:
                self.camera_cache[cam_id] = []
            self.camera_cache[cam_id].append(camera_frame)
    
    def save_gnss_batch(self, gnss_list: List, utc_time: datetime):
        """
        批量保存GNSS数据到CSV
        
        命名：gnss_年_月_日_时_分_秒.csv（优先使用数据列表第一条的UTC时间）
        
        Args:
            gnss_list: GNSS数据列表
            utc_time: 备用UTC时间（当数据中无时间时使用）
        """
        if not gnss_list or not self.current_clip_path:
            return
        
        try:
            # 优先使用数据本身的时间
            if gnss_list and gnss_list[0].utc_time:
                target_time = gnss_list[0].utc_time
            else:
                target_time = utc_time

            # 生成文件名
            filename = f"gnss_{target_time.strftime('%Y_%m_%d_%H_%M_%S')}.csv"
            filepath = self.current_clip_path / 'gnss' / filename
            
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
                
                # 写入数据行
                for gnss_data in gnss_list:
                    writer.writerow([
                        f"{gnss_data.timestamp:.6f}",
                        gnss_data.utc_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] if gnss_data.utc_time else '',
                        f"{gnss_data.latitude:.8f}" if gnss_data.latitude is not None else '',
                        f"{gnss_data.longitude:.8f}" if gnss_data.longitude is not None else '',
                        f"{gnss_data.heading:.2f}" if gnss_data.heading is not None else '',
                        f"{gnss_data.pitch:.2f}" if gnss_data.pitch is not None else '',
                        f"{gnss_data.roll:.2f}" if gnss_data.roll is not None else '',
                        f"{gnss_data.altitude:.3f}" if gnss_data.altitude is not None else '',
                        f"{gnss_data.speed:.2f}" if gnss_data.speed is not None else '',
                        gnss_data.location_quality if gnss_data.location_quality is not None else '',
                        gnss_data.heading_quality if gnss_data.heading_quality is not None else '',
                        gnss_data.satellites if gnss_data.satellites is not None else '',
                        f"{gnss_data.hdop:.2f}" if gnss_data.hdop is not None else '',
                        gnss_data.raw_message
                    ])
            
            with self.stats_lock:
                self.stats['gnss_saved'] += len(gnss_list)
            
            logger.debug(f"保存GNSS数据: {len(gnss_list)}条 -> {filename}")
            
        except Exception as e:
            logger.error(f"保存GNSS数据失败: {e}")
    
    def save_ais_batch(self, ais_list: List, utc_time: datetime):
        """
        批量保存AIS数据到CSV
        
        命名：ais_年_月_日_时_分_秒.csv（优先使用数据列表第一条的UTC时间）
        
        Args:
            ais_list: AIS数据列表
            utc_time: 备用UTC时间
        """
        if not ais_list or not self.current_clip_path:
            return
        
        try:
            # 优先使用数据本身的时间
            if ais_list and ais_list[0].utc_time:
                target_time = ais_list[0].utc_time
            else:
                target_time = utc_time

            # 生成文件名
            filename = f"ais_{target_time.strftime('%Y_%m_%d_%H_%M_%S')}.csv"
            filepath = self.current_clip_path / 'ais' / filename
            
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
                
                # 写入数据行
                for ais_data in ais_list:
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
                self.stats['ais_saved'] += len(ais_list)
            
            logger.debug(f"保存AIS数据: {len(ais_list)}条 -> {filename}")
            
        except Exception as e:
            logger.error(f"保存AIS数据失败: {e}")
    
    def save_camera_frame(self, camera_frame, utc_time: datetime, index: int = 0):
        """
        保存相机帧为JPG图像
        
        命名：{相机ID}_年_月_日_时_分_秒_毫秒_索引.jpg
        
        Args:
            camera_frame: CameraFrame对象
            utc_time: GNSS UTC时间
            index: 帧索引（用于区分同一秒内的多帧）
        """
        if not self.current_clip_path:
            logger.error("尚未创建clip目录")
            return False
        
        try:
            cam_id = camera_frame.camera_id
            
            # 生成文件名，增加index防止覆盖
            millisec = int(utc_time.microsecond / 1000)
            filename = f"{cam_id}_{utc_time.strftime('%Y_%m_%d_%H_%M_%S')}_{millisec:03d}_{index:02d}.{self.image_format}"
            
            filepath = self.current_clip_path / 'camera' / cam_id / filename
            
            # 保存图像
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.image_quality]
            success = cv2.imwrite(str(filepath), camera_frame.frame, encode_params)
            
            if success:
                with self.stats_lock:
                    self.stats['frames_saved'] += 1
                logger.debug(f"保存相机帧: {filename}")
                return True
            else:
                logger.error(f"图像保存失败: {filename}")
                return False
                
        except Exception as e:
            logger.error(f"保存相机帧异常: {e}")
            return False
    
    def save_metadata(self, metadata: Dict):
        """
        保存元数据为JSON
        
        Args:
            metadata: 元数据字典
        """
        if not self.current_clip_path:
            logger.error("尚未创建clip目录")
            return
        
        try:
            filepath = self.current_clip_path / 'metadata' / 'metadata.json'
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(metadata, f, indent=2, ensure_ascii=False, default=str)
            
            logger.info(f"✅ 元数据已保存: {filepath}")
            
        except Exception as e:
            logger.error(f"保存元数据失败: {e}")
    
    def start_batch_writer(self):
        """启动批量写入线程"""
        if self.running:
            logger.warning("批量写入线程已在运行")
            return
        
        self.running = True
        self.write_thread = threading.Thread(
            target=self._batch_write_loop,
            name="BatchWriter",
            daemon=True
        )
        self.write_thread.start()
        
        logger.info("批量写入线程已启动")
    
    def stop_batch_writer(self):
        """停止批量写入线程"""
        if not self.running:
            return
        
        self.running = False
        
        if self.write_thread:
            self.write_thread.join(timeout=5.0)
        
        # 写入剩余数据
        self._flush_all_caches()
        
        logger.info("批量写入线程已停止")
    
    def _batch_write_loop(self):
        """批量写入循环（每秒触发一次）"""
        logger.info("批量写入循环开始")
        
        while self.running:
            current_time = time.time()
            
            # 每秒检查一次是否需要写入
            self._check_and_write_per_second()
            
            # 清理过期数据
            self._cleanup_expired_data()
            
            time.sleep(1.0)
        
        logger.info("批量写入循环结束")
    
    def _check_and_write_per_second(self):
        """
        每秒检查并写入数据
        
        按需求：每秒保存一次当前一秒内接收到的所有过滤后的#AIVDM和#AGRICA数据
        """
        current_utc = datetime.now(timezone.utc)
        
        # 检查是否到了新的一秒
        if self.last_save_time:
            time_diff = (current_utc - self.last_save_time).total_seconds()
            if time_diff < 1.0:
                return
        
        with self.cache_lock:
            # 保存GNSS数据
            if self.current_second_gnss:
                self.save_gnss_batch(self.current_second_gnss, current_utc)
                self.current_second_gnss = []
            
            # 保存AIS数据
            if self.current_second_ais:
                self.save_ais_batch(self.current_second_ais, current_utc)
                self.current_second_ais = []
            
            # 保存相机帧
            for cam_id, frames in self.camera_cache.items():
                if frames:
                    for i, frame in enumerate(frames):
                        self.save_camera_frame(frame, current_utc, i)
                    self.camera_cache[cam_id] = []
            
            with self.stats_lock:
                self.stats['batches_written'] += 1
        
        self.last_save_time = current_utc
    
    def _cleanup_expired_data(self):
        """清理过期数据（超过5分钟）"""
        current_time = time.time()
        expire_threshold = self.expire_time
        
        with self.cache_lock:
            # 清理GNSS缓存
            self.current_second_gnss = [
                d for d in self.current_second_gnss
                if current_time - d.timestamp < expire_threshold
            ]
            
            # 清理AIS缓存
            self.current_second_ais = [
                d for d in self.current_second_ais
                if current_time - d.timestamp < expire_threshold
            ]
            
            # 清理相机缓存
            for cam_id in list(self.camera_cache.keys()):
                self.camera_cache[cam_id] = [
                    f for f in self.camera_cache[cam_id]
                    if current_time - f.timestamp < expire_threshold
                ]
    
    def _flush_all_caches(self):
        """刷新所有缓存（程序结束时调用）"""
        logger.info("刷新所有缓存...")
        
        current_utc = datetime.now(timezone.utc)
        
        with self.cache_lock:
            # 保存GNSS数据
            if self.current_second_gnss:
                self.save_gnss_batch(self.current_second_gnss, current_utc)
                logger.info(f"刷新GNSS数据: {len(self.current_second_gnss)}条")
                self.current_second_gnss = []
            
            # 保存AIS数据
            if self.current_second_ais:
                self.save_ais_batch(self.current_second_ais, current_utc)
                logger.info(f"刷新AIS数据: {len(self.current_second_ais)}条")
                self.current_second_ais = []
            
            # 保存相机帧
            total_frames = 0
            for cam_id, frames in self.camera_cache.items():
                if frames:
                    for i, frame in enumerate(frames):
                        self.save_camera_frame(frame, current_utc, i)
                    total_frames += len(frames)
                    self.camera_cache[cam_id] = []
            
            if total_frames > 0:
                logger.info(f"刷新相机帧: {total_frames}帧")
    
    def generate_clip_metadata(self, start_time: datetime, end_time: datetime,
                               camera_ids: List[str], sync_stats: Dict) -> Dict:
        """
        生成数据包元数据
        
        Args:
            start_time: 开始时间
            end_time: 结束时间
            camera_ids: 相机ID列表
            sync_stats: 时间同步统计信息
            
        Returns:
            元数据字典
        """
        metadata = {
            'clip_id': self.current_clip_path.name if self.current_clip_path else '',
            'start_time': start_time.isoformat() + 'Z',
            'end_time': end_time.isoformat() + 'Z',
            'duration_seconds': (end_time - start_time).total_seconds(),
            'cameras': camera_ids,
            'camera_count': len(camera_ids),
            'storage_stats': self.get_statistics(),
            'sync_error_stats': {
                'mean_ms': sync_stats.get('mean_sync_error', 0.0),
                'max_ms': sync_stats.get('max_sync_error', 0.0),
                'total_aligned': sync_stats.get('total_aligned', 0)
            },
            'generation_time': datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')
        }
        
        return metadata
    
    def get_statistics(self) -> Dict:
        """获取统计信息"""
        with self.stats_lock:
            return self.stats.copy()
    
    def print_statistics(self):
        """打印统计信息"""
        stats = self.get_statistics()
        
        print("\n" + "="*60)
        print("数据存储统计")
        print("="*60)
        print(f"数据包数量: {stats['total_clips']}")
        print(f"GNSS数据: {stats['gnss_saved']}条")
        print(f"AIS数据: {stats['ais_saved']}条")
        print(f"相机帧: {stats['frames_saved']}帧")
        print(f"批次数: {stats['batches_written']}")
        print("="*60 + "\n")


# 测试代码
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 测试配置
    test_config = {
        'root_path': '/tmp/test_dataset',
        'image_format': 'jpg',
        'image_quality': 95,
        'sample_rate': 1,
        'cache': {
            'max_queue_size': 100,
            'batch_write_size': 50,
            'batch_write_interval': 5.0,
            'expire_time': 300
        }
    }
    
    storage = DataStorage(test_config)
    
    # 创建clip目录
    clip_path = storage.create_clip()
    print(f"Clip路径: {clip_path}")
    
    # 创建相机子目录
    camera_ids = ['camera_01', 'camera_02', 'camera_03']
    storage.create_camera_subdirs(camera_ids)
    
    # 启动批量写入线程
    storage.start_batch_writer()
    
    print("数据存储管理器测试完成")
    print("按Ctrl+C停止...")
    
    try:
        time.sleep(5)
    except KeyboardInterrupt:
        pass
    finally:
        storage.stop_batch_writer()
        storage.print_statistics()

