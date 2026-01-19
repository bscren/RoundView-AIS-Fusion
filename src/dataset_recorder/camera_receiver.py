#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多相机RTSP接收模块
负责从RTSP流接收视频帧并提取时间戳
"""

import cv2
import threading
import queue
import time
import logging
import numpy as np
from datetime import datetime
from typing import Dict, Optional, List, Tuple
from dataclasses import dataclass
from pathlib import Path

logger = logging.getLogger(__name__)


@dataclass
class CameraFrame:
    """相机帧数据结构"""
    camera_id: str  # 相机ID
    timestamp: float  # 系统时间戳
    pts: Optional[float]  # PTS时间戳（from PyAV）
    frame: np.ndarray  # 图像帧
    frame_number: int  # 帧编号
    width: int  # 图像宽度
    height: int  # 图像高度
    valid: bool  # 帧是否有效


class SingleCameraReceiver:
    """单个相机接收器"""
    
    def __init__(self, camera_config: Dict, save_path: Optional[str] = None, video_filename: Optional[str] = None):
        """
        初始化单相机接收器
        
        Args:
            camera_config: 相机配置字典
            save_path: 视频文件保存路径（如果为None则不保存）
            video_filename: 视频文件名（不含扩展名）
        """
        self.config = camera_config
        self.camera_id = camera_config['id']
        self.rtsp_url = camera_config['rtsp_url']
        
        self.cap = None
        self.running = False
        self.thread = None
        
        # 数据队列
        self.frame_queue = queue.Queue(maxsize=10)
        
        # 视频保存相关
        self.save_path = Path(save_path) if save_path else None
        self.save_enabled = save_path is not None
        self.video_filename = video_filename
        self.video_writer = None
        self.video_filepath = None
        self.first_frame_saved = False
        self.recording_started = False  # 标记是否已开始录制
        
        # 统计信息
        self.stats = {
            'total_frames': 0,
            'valid_frames': 0,
            'dropped_frames': 0,
            'black_frames': 0,
            'connection_lost': 0,
            'saved_frames': 0
        }
        self.stats_lock = threading.Lock()
        
        # 连接状态
        self.connected = False
        self.last_frame_time = 0
        
        # 创建保存目录
        if self.save_enabled:
            self.save_path.mkdir(parents=True, exist_ok=True)
            logger.info(f"{self.camera_id}视频将保存到: {self.save_path}")
        
    def connect(self) -> bool:
        """
        连接RTSP流
        
        Returns:
            连接是否成功
        """
        try:
            # 使用FFMPEG后端（更可靠）
            self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
            
            if self.cap.isOpened():
                # 设置缓冲区
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                # 预热：读取几帧
                for i in range(3):
                    ret, _ = self.cap.read()
                    if not ret:
                        logger.warning(f"{self.camera_id}: 预热帧{i+1}读取失败")
                
                self.connected = True
                logger.info(f"✅ {self.camera_id} RTSP连接成功")
                return True
            else:
                logger.error(f"❌ {self.camera_id} RTSP流打开失败")
                return False
                
        except Exception as e:
            logger.error(f"❌ {self.camera_id} 连接异常: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.cap:
            try:
                self.cap.release()
                self.connected = False
                logger.info(f"{self.camera_id} 已断开连接")
            except Exception as e:
                logger.warning(f"{self.camera_id} 断开连接异常: {e}")
    
    def start(self) -> bool:
        """启动接收线程"""
        if self.running:
            logger.warning(f"{self.camera_id} 接收器已在运行")
            return True
        
        if not self.connected:
            if not self.connect():
                return False
        
        self.running = True
        self.thread = threading.Thread(
            target=self._receive_loop,
            name=f"Camera-{self.camera_id}",
            daemon=True
        )
        self.thread.start()
        
        logger.info(f"{self.camera_id} 接收线程已启动")
        return True
    
    def stop(self):
        """停止接收线程"""
        if not self.running:
            return
        
        self.running = False
        
        if self.thread:
            self.thread.join(timeout=5.0)
        
        # 关闭视频写入器
        if self.video_writer:
            self.video_writer.release()
            logger.info(f"{self.camera_id}视频已保存: {self.video_filepath}")
            self.video_writer = None
        
        self.disconnect()
        logger.info(f"{self.camera_id} 接收线程已停止")
    
    def _receive_loop(self):
        """接收循环"""
        logger.info(f"{self.camera_id} 接收循环开始")
        
        frame_number = 0
        consecutive_failures = 0
        max_failures = 10
        
        while self.running:
            try:
                # 检查连接
                if not self.cap or not self.cap.isOpened():
                    logger.warning(f"{self.camera_id} 连接丢失，尝试重连...")
                    with self.stats_lock:
                        self.stats['connection_lost'] += 1
                    
                    self.disconnect()
                    if not self.connect():
                        time.sleep(5.0)
                        continue
                    
                    consecutive_failures = 0
                
                # 读取帧
                ret, frame = self.cap.read()
                
                if not ret or frame is None:
                    consecutive_failures += 1
                    logger.debug(f"{self.camera_id} 读取帧失败 ({consecutive_failures}/{max_failures})")
                    
                    if consecutive_failures >= max_failures:
                        logger.error(f"{self.camera_id} 连续读取失败，断开连接")
                        self.disconnect()
                        consecutive_failures = 0
                    
                    time.sleep(0.1)
                    continue
                
                consecutive_failures = 0
                frame_number += 1
                
                # 获取时间戳
                timestamp = time.time()
                self.last_frame_time = timestamp
                
                # 获取PTS（from OpenCV，可能不准确）
                pts = self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0 if self.cap else None
                
                # 检查帧有效性
                valid = self._validate_frame(frame)
                
                if not valid:
                    with self.stats_lock:
                        if self._is_black_frame(frame):
                            self.stats['black_frames'] += 1
                        self.stats['dropped_frames'] += 1
                    continue
                
                # 创建帧数据
                camera_frame = CameraFrame(
                    camera_id=self.camera_id,
                    timestamp=timestamp,
                    pts=pts,
                    frame=frame,
                    frame_number=frame_number,
                    width=frame.shape[1],
                    height=frame.shape[0],
                    valid=valid
                )
                
                # 保存到视频
                if self.save_enabled:
                    self._write_frame_to_video(frame)
                
                # 入队
                self._enqueue_frame(camera_frame)
                
                with self.stats_lock:
                    self.stats['total_frames'] += 1
                    self.stats['valid_frames'] += 1
                
                # 控制帧率
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"{self.camera_id} 接收循环异常: {e}")
                time.sleep(1.0)
        
        logger.info(f"{self.camera_id} 接收循环结束")
    
    def _validate_frame(self, frame: np.ndarray) -> bool:
        """验证帧有效性"""
        if frame is None:
            return False
        
        # 检查尺寸
        if frame.shape[0] < 100 or frame.shape[1] < 100:
            return False
        
        # 检查是否全黑
        if self._is_black_frame(frame):
            return False
        
        return True
    
    def _is_black_frame(self, frame: np.ndarray, threshold: float = 10.0) -> bool:
        """检查是否为全黑帧"""
        mean_value = np.mean(frame)
        return mean_value < threshold
    
    def _enqueue_frame(self, camera_frame: CameraFrame):
        """将帧加入队列"""
        try:
            self.frame_queue.put(camera_frame, block=False)
        except queue.Full:
            # 队列满，移除最旧的帧
            try:
                self.frame_queue.get_nowait()
                self.frame_queue.put(camera_frame, block=False)
                with self.stats_lock:
                    self.stats['dropped_frames'] += 1
            except queue.Empty:
                pass
    
    def get_frame(self, timeout: Optional[float] = None) -> Optional[CameraFrame]:
        """获取帧"""
        try:
            return self.frame_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_statistics(self) -> Dict:
        """获取统计信息"""
        with self.stats_lock:
            return self.stats.copy()
    
    def is_connected(self) -> bool:
        """检查是否连接"""
        return self.connected and self.cap is not None and self.cap.isOpened()
    
    def _write_frame_to_video(self, frame: np.ndarray):
        """
        将帧写入视频文件
        
        Args:
            frame: 图像帧
        """
        try:
            # 初始化视频写入器（首次写入时）
            if self.video_writer is None:
                if not self.video_filename:
                    logger.error(f"{self.camera_id}未设置视频文件名，无法保存")
                    return
                
                # 生成视频文件路径
                video_file = f"{self.video_filename}.mp4"
                self.video_filepath = self.save_path / video_file
                
                # 获取帧尺寸和帧率
                height, width = frame.shape[:2]
                fps = self.config.get('fps')
                
                
                # 创建视频写入器
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 使用mp4v编码
                self.video_writer = cv2.VideoWriter(
                    str(self.video_filepath),
                    fourcc,
                    fps,
                    (width, height)
                )
                
                if not self.video_writer.isOpened():
                    logger.error(f"{self.camera_id}视频写入器初始化失败")
                    self.video_writer = None
                    return
                
                logger.info(f"{self.camera_id}视频写入器已初始化: {self.video_filepath} ({width}x{height}@{fps}fps)")
                self.first_frame_saved = True
                self.recording_started = True  # 标记已开始录制
            
            # 写入帧
            if self.video_writer and self.video_writer.isOpened():
                self.video_writer.write(frame)
                with self.stats_lock:
                    self.stats['saved_frames'] += 1
                    
        except Exception as e:
            logger.error(f"{self.camera_id}写入视频帧失败: {e}")
    
    def set_video_filename(self, filename: str):
        """
        设置视频文件名（不含扩展名）
        
        Args:
            filename: 文件名
        """
        self.video_filename = filename


class MultiCameraReceiver:
    """多相机接收器管理器"""
    
    def __init__(self, cameras_config: List[Dict], save_path: Optional[str] = None):
        """
        初始化多相机接收器
        
        Args:
            cameras_config: 相机配置列表
            save_path: 视频保存根路径（每个相机会在此路径下创建子目录）
        """
        self.cameras_config = cameras_config
        self.receivers = {}
        self.save_path = Path(save_path) if save_path else None
        
        # 创建各相机接收器
        for cam_config in cameras_config:
            if cam_config.get('enabled', True):
                cam_id = cam_config['id']
                # 为每个相机创建独立的保存路径
                cam_save_path = None
                if self.save_path:
                    cam_save_path = str(self.save_path / cam_id)
                self.receivers[cam_id] = SingleCameraReceiver(cam_config, save_path=cam_save_path)
        
        logger.info(f"多相机接收器初始化完成，共{len(self.receivers)}个相机")
    
    def connect_all(self) -> Dict[str, bool]:
        """
        连接所有相机
        
        Returns:
            各相机连接结果
        """
        results = {}
        
        for cam_id, receiver in self.receivers.items():
            results[cam_id] = receiver.connect()
        
        success_count = sum(1 for r in results.values() if r)
        logger.info(f"相机连接完成: {success_count}/{len(results)} 成功")
        
        return results
    
    def start_all(self) -> Dict[str, bool]:
        """
        启动所有相机接收线程
        
        Returns:
            各相机启动结果
        """
        results = {}
        
        for cam_id, receiver in self.receivers.items():
            results[cam_id] = receiver.start()
        
        success_count = sum(1 for r in results.values() if r)
        logger.info(f"相机接收启动完成: {success_count}/{len(results)} 成功")
        
        return results
    
    def stop_all(self):
        """停止所有相机接收"""
        for cam_id, receiver in self.receivers.items():
            receiver.stop()
        
        logger.info("所有相机接收已停止")
    
    def get_frame_from_camera(self, camera_id: str, 
                             timeout: Optional[float] = None) -> Optional[CameraFrame]:
        """
        从指定相机获取帧
        
        Args:
            camera_id: 相机ID
            timeout: 超时时间
            
        Returns:
            相机帧数据
        """
        if camera_id not in self.receivers:
            return None
        
        return self.receivers[camera_id].get_frame(timeout)
    
    def get_all_frames(self, timeout: float = 1.0) -> Dict[str, Optional[CameraFrame]]:
        """
        从所有相机获取帧
        
        Args:
            timeout: 超时时间
            
        Returns:
            各相机帧数据字典
        """
        frames = {}
        
        for cam_id in self.receivers.keys():
            frames[cam_id] = self.get_frame_from_camera(cam_id, timeout)
        
        return frames
    
    def get_all_statistics(self) -> Dict[str, Dict]:
        """获取所有相机统计信息"""
        stats = {}
        
        for cam_id, receiver in self.receivers.items():
            stats[cam_id] = receiver.get_statistics()
        
        return stats
    
    def get_connection_status(self) -> Dict[str, bool]:
        """获取所有相机连接状态"""
        status = {}
        
        for cam_id, receiver in self.receivers.items():
            status[cam_id] = receiver.is_connected()
        
        return status
    
    def clear_all_queues(self):
        """清空所有相机的帧队列"""
        for cam_id, receiver in self.receivers.items():
            # 清空该相机的帧队列
            while not receiver.frame_queue.empty():
                try:
                    receiver.frame_queue.get_nowait()
                except:
                    break
        
        logger.info("已清空所有相机队列")
    
    def set_video_filename(self, filename: str):
        """
        为所有相机设置视频文件名（不含扩展名）
        
        Args:
            filename: 文件名基础（每个相机会添加自己的ID作为前缀）
        """
        for cam_id, receiver in self.receivers.items():
            video_filename = f"{cam_id}_{filename}"
            receiver.set_video_filename(video_filename)
        
        logger.info(f"已为所有相机设置视频文件名: {filename}")
    
    def all_cameras_recording(self) -> bool:
        """
        检查是否所有相机都已开始录制
        
        Returns:
            如果所有相机都已开始录制则返回True
        """
        if not self.receivers:
            return False
        
        for cam_id, receiver in self.receivers.items():
            if not receiver.recording_started:
                return False
        
        return True
    
    def get_recording_status(self) -> Dict[str, bool]:
        """
        获取所有相机的录制状态
        
        Returns:
            各相机录制状态字典
        """
        status = {}
        for cam_id, receiver in self.receivers.items():
            status[cam_id] = receiver.recording_started
        return status
    
    def print_status_summary(self):
        """打印状态摘要"""
        print("\n" + "="*60)
        print("多相机接收状态")
        print("="*60)
        
        status = self.get_connection_status()
        stats = self.get_all_statistics()
        
        for cam_id in sorted(self.receivers.keys()):
            conn = "✅ 已连接" if status.get(cam_id, False) else "❌ 未连接"
            st = stats.get(cam_id, {})
            
            print(f"{cam_id}:")
            print(f"  状态: {conn}")
            print(f"  总帧数: {st.get('total_frames', 0)}")
            print(f"  有效帧: {st.get('valid_frames', 0)}")
            print(f"  丢弃帧: {st.get('dropped_frames', 0)}")
            print(f"  黑帧数: {st.get('black_frames', 0)}")
            print(f"  断连次数: {st.get('connection_lost', 0)}")
        
        print("="*60 + "\n")


# 测试代码
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 测试配置
    test_cameras = [
        {
            'id': 'camera_01',
            'rtsp_url': 'rtsp://admin:juhai001@192.168.1.1:554/Streaming/Channels/2',
            'enabled': True
        },
        {
            'id': 'camera_02',
            'rtsp_url': 'rtsp://admin:juhai002@192.168.1.2:554/Streaming/Channels/2',
            'enabled': True
        },
        {
            'id': 'camera_03',
            'rtsp_url': 'rtsp://admin:juhai003@192.168.1.3:554/Streaming/Channels/2',
            'enabled': True
        }
    ]
    
    manager = MultiCameraReceiver(test_cameras)
    
    # 连接所有相机
    results = manager.connect_all()
    print(f"连接结果: {results}")
    
    # 启动接收
    if all(results.values()):
        manager.start_all()
        
        print("接收已启动，按Ctrl+C停止...")
        
        try:
            while True:
                # 获取所有相机的帧
                frames = manager.get_all_frames(timeout=1.0)
                
                for cam_id, frame in frames.items():
                    if frame:
                        print(f"{cam_id}: 帧{frame.frame_number}, "
                              f"尺寸{frame.width}x{frame.height}, "
                              f"时间戳{frame.timestamp:.3f}")
                        
                        # 显示图像
                        cv2.imshow(cam_id, frame.frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                # 每5秒打印状态
                time.sleep(5.0)
                manager.print_status_summary()
                
        except KeyboardInterrupt:
            print("\n停止接收...")
        finally:
            manager.stop_all()
            cv2.destroyAllWindows()
            
            # 打印最终统计
            print("\n最终统计:")
            manager.print_status_summary()
    else:
        print("部分相机连接失败")

