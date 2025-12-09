#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多相机-AIS-RTK数据集录制工具 - 主程序
整合所有模块，实现完整的数据采集流程
"""
import os
import sys
import time
import signal
import logging
import argparse
from pathlib import Path
from datetime import datetime, timezone
from typing import Optional
import threading

# 导入自定义模块
from config_manager import ConfigManager
from ais_receiver import AISReceiver
from camera_receiver import MultiCameraReceiver
from rtk_receiver import RTKReceiver

logger = logging.getLogger(__name__)


class DatasetRecorder:
    """数据集录制器主类"""
    
    def __init__(self, config_path: str):
        """
        初始化数据集录制器
        
        Args:
            config_path: 配置文件路径
        """
        self.config_path = config_path
        self.config_manager = None
        self.config = None
        
        # 各模块实例
        self.ais_receiver = None
        self.camera_manager = None
        self.rtk_receiver = None
        
        # 数据保存路径
        self.dataset_root = None
        self.current_session_path = None
        
        # 运行状态
        self.running = False
        self.start_time = None
        
        logger.info("="*60)
        logger.info("多相机-AIS-RTK数据集录制工具")
        logger.info("="*60)
    
    def load_and_validate_config(self) -> bool:
        """
        加载和验证配置文件
        
        Returns:
            是否成功
        """
        logger.info("\n步骤1: 加载配置文件")
        logger.info("-"*60)
        
        try:
            self.config_manager = ConfigManager(self.config_path)
            self.config = self.config_manager.load_config()
            logger.info(f"✅ 配置文件加载成功: {self.config_path}")
            
            # 验证配置
            if not self.config_manager.validate_config():
                logger.error("❌ 配置文件验证失败")
                return False
            
            logger.info("✅ 配置文件验证通过")
            return True
            
        except Exception as e:
            logger.error(f"❌ 加载配置失败: {e}")
            return False
    
    def verify_device_connectivity(self) -> bool:
        """
        验证设备连通性
        
        Returns:
            是否所有设备连接成功
        """
        logger.info("\n步骤2: 验证设备连通性")
        logger.info("-"*60)
        
        results = self.config_manager.verify_device_connectivity()
        success = self.config_manager.print_validation_summary()
        
        if not success:
            logger.error("❌ 部分设备连接失败")
            logger.info("\n请检查：")
            for device, result in results.items():
                if not result:
                    logger.error(f"  - {device}: 连接失败")
            return False
        
        logger.info("✅ 所有设备连接成功")
        return True
    
    def initialize_modules(self) -> bool:
        """
        初始化所有模块
        
        Returns:
            是否成功
        """
        logger.info("\n步骤3: 初始化模块")
        logger.info("-"*60)
        
        try:
            devices = self.config['devices']
            storage_config = self.config['storage']
            
            # 创建会话目录
            self.dataset_root = Path(storage_config['root_path'])
            session_name = f"session_{datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S')}"
            self.current_session_path = self.dataset_root / session_name
            self.current_session_path.mkdir(parents=True, exist_ok=True)
            
            logger.info(f"会话目录: {self.current_session_path}")
            
            # 创建子目录
            rtk_path = self.current_session_path / 'rtk'
            ais_path = self.current_session_path / 'ais'
            camera_path = self.current_session_path / 'camera'
            
            rtk_path.mkdir(exist_ok=True)
            ais_path.mkdir(exist_ok=True)
            camera_path.mkdir(exist_ok=True)
            
            # 初始化RTK接收器
            if devices.get('rtk', {}).get('enabled', False):
                logger.info("初始化RTK接收器...")
                self.rtk_receiver = RTKReceiver(devices['rtk'], save_path=str(rtk_path))
                if not self.rtk_receiver.start():
                    logger.error("❌ RTK接收器启动失败")
                    return False
                logger.info("✅ RTK接收器已启动")
            else:
                logger.warning("⚠️  RTK未启用")
            
            # 初始化AIS接收器
            if devices.get('ais', {}).get('enabled', False):
                logger.info("初始化AIS接收器...")
                self.ais_receiver = AISReceiver(devices['ais'], save_path=str(ais_path))
                if not self.ais_receiver.start():
                    logger.warning("⚠️  AIS接收器启动失败")
                else:
                    logger.info("✅ AIS接收器已启动")
            
            # 初始化多相机接收器
            cameras = devices.get('cameras', [])
            enabled_cameras = [c for c in cameras if c.get('enabled', True)]
            
            if enabled_cameras:
                logger.info(f"初始化{len(enabled_cameras)}个相机...")
                self.camera_manager = MultiCameraReceiver(enabled_cameras, save_path=str(camera_path))
                results = self.camera_manager.start_all()
                
                success_count = sum(1 for r in results.values() if r)
                logger.info(f"✅ 相机启动: {success_count}/{len(results)}")
                
                if success_count == 0:
                    logger.error("❌ 所有相机启动失败")
                    return False
            else:
                logger.warning("⚠️  没有启用的相机")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ 模块初始化失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def setup_video_filenames(self) -> bool:
        """
        设置视频文件名（使用第一条数据的UTC时间）
        
        Returns:
            是否成功
        """
        logger.info("\n步骤4: 等待首条数据并设置文件名")
        logger.info("-"*60)
        
        try:
            # 等待第一条RTK数据（最多等待10秒）
            first_time = None
            for i in range(100):
                if self.rtk_receiver and self.rtk_receiver.first_data_time:
                    first_time = self.rtk_receiver.first_data_time
                    break
                elif self.ais_receiver and self.ais_receiver.first_data_time:
                    first_time = self.ais_receiver.first_data_time
                    break
                time.sleep(0.1)
            
            if not first_time:
                logger.warning("未等到首条数据，使用当前UTC时间")
                first_time = datetime.utcnow()
            
            # 生成文件名时间戳
            time_str = first_time.strftime('%Y_%m_%d_%H_%M_%S')
            logger.info(f"使用时间戳: {time_str}")
            
            # 设置相机视频文件名
            if self.camera_manager:
                self.camera_manager.set_video_filename(time_str)
                logger.info("✅ 相机视频文件名已设置")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ 设置文件名异常: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def wait_for_cameras_ready(self) -> bool:
        """
        等待所有相机开始录制视频
        
        Returns:
            是否成功
        """
        logger.info("\n步骤5: 等待相机开始录制")
        logger.info("-"*60)
        
        try:
            if not self.camera_manager:
                logger.warning("没有相机管理器，跳过等待")
                # 如果没有相机，直接触发保存事件
                if self.rtk_receiver:
                    self.rtk_receiver.start_saving_event.set()
                if self.ais_receiver:
                    self.ais_receiver.start_saving_event.set()
                return True
            
            # 等待所有相机开始录制（最多等待30秒）
            max_wait_time = 30
            wait_interval = 0.5
            elapsed = 0
            
            logger.info("等待相机开始录制视频...")
            
            while elapsed < max_wait_time:
                recording_status = self.camera_manager.get_recording_status()
                
                # 打印当前状态
                if elapsed % 5 < wait_interval:  # 每5秒打印一次
                    ready_count = sum(1 for status in recording_status.values() if status)
                    total_count = len(recording_status)
                    logger.info(f"  相机录制状态: {ready_count}/{total_count} 已就绪")
                    for cam_id, status in recording_status.items():
                        status_text = "✅" if status else "⏳"
                        logger.info(f"    {cam_id}: {status_text}")
                
                # 检查是否所有相机都已开始录制
                if self.camera_manager.all_cameras_recording():
                    logger.info("✅ 所有相机已开始录制")
                    
                    # 触发RTK和AIS开始保存
                    if self.rtk_receiver:
                        self.rtk_receiver.start_saving_event.set()
                        logger.info("✅ 已触发RTK开始保存")
                    
                    if self.ais_receiver:
                        self.ais_receiver.start_saving_event.set()
                        logger.info("✅ 已触发AIS开始保存")
                    
                    return True
                
                time.sleep(wait_interval)
                elapsed += wait_interval
            
            # 超时处理
            logger.warning(f"等待相机就绪超时（{max_wait_time}秒）")
            ready_count = sum(1 for status in recording_status.values() if status)
            total_count = len(recording_status)
            logger.warning(f"当前状态: {ready_count}/{total_count} 相机已就绪")
            
            # 即使超时，也触发保存事件
            if self.rtk_receiver:
                self.rtk_receiver.start_saving_event.set()
            if self.ais_receiver:
                self.ais_receiver.start_saving_event.set()
            
            return ready_count > 0  # 至少有一个相机就绪就算成功
            
        except Exception as e:
            logger.error(f"❌ 等待相机就绪异常: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def start_recording(self) -> bool:
        """
        启动数据录制
        
        Returns:
            是否成功启动
        """
        logger.info("\n步骤6: 开始录制")
        logger.info("-"*60)
        
        try:
            # 记录开始时间
            self.start_time = datetime.utcnow()
            self.running = True
            
            logger.info("✅ 数据录制已启动")
            logger.info(f"   - RTK数据保存到: {self.current_session_path / 'rtk'}")
            logger.info(f"   - AIS数据保存到: {self.current_session_path / 'ais'}")
            logger.info(f"   - 相机视频保存到: {self.current_session_path / 'camera'}")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ 启动录制失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    
    def _print_status(self):
        """打印实时状态"""
        print("\n" + "="*60)
        print(f"运行状态 - {datetime.now().strftime('%H:%M:%S')}")
        print("="*60)
        
        # RTK状态
        if self.rtk_receiver:
            rtk_stats = self.rtk_receiver.get_statistics()
            print(f"RTK: 有效={rtk_stats['valid_data']}, "
                  f"总接收={rtk_stats['total_received']}, "
                  f"已保存={rtk_stats.get('saved_count', 0)}, "
                  f"CSV文件={rtk_stats.get('csv_files', 0)}")
        
        # AIS状态
        if self.ais_receiver:
            ais_stats = self.ais_receiver.get_statistics()
            print(f"AIS: 有效={ais_stats.get('valid_ais', 0)}, "
                  f"总接收={ais_stats.get('total_received', 0)}, "
                  f"已保存={ais_stats.get('saved_count', 0)}, "
                  f"CSV文件={ais_stats.get('csv_files', 0)}")
        
        # 相机状态
        if self.camera_manager:
            cam_stats = self.camera_manager.get_all_statistics()
            for cam_id, stats in sorted(cam_stats.items()):
                print(f"{cam_id}: 有效帧={stats['valid_frames']}, "
                      f"已保存={stats.get('saved_frames', 0)}, "
                      f"丢弃={stats['dropped_frames']}")
        
        print("="*60 + "\n")
    
    def stop_recording(self):
        """停止数据录制并优雅关闭"""
        if not self.running:
            return
        
        logger.info("\n停止录制...")
        logger.info("="*60)
        
        self.running = False
        
        # 停止所有接收器
        if self.rtk_receiver:
            logger.info("停止RTK接收...")
            self.rtk_receiver.stop()
        
        if self.ais_receiver:
            logger.info("停止AIS接收...")
            self.ais_receiver.stop()
        
        if self.camera_manager:
            logger.info("停止相机接收...")
            self.camera_manager.stop_all()
        
        # 生成最终报告
        self._generate_final_report()
        
        logger.info("="*60)
        logger.info("✅ 录制已停止")
        logger.info("="*60)
    
    def _generate_final_report(self):
        """生成最终报告和元数据"""
        logger.info("\n生成最终报告...")
        
        try:
            # 收集统计信息
            end_time = datetime.utcnow()
            duration = (end_time - self.start_time).total_seconds() if self.start_time else 0
            
            # 打印最终统计
            print("\n" + "="*60)
            print("最终统计报告")
            print("="*60)
            print(f"会话路径: {self.current_session_path}")
            print(f"开始时间: {self.start_time.strftime('%Y-%m-%d %H:%M:%S UTC') if self.start_time else 'N/A'}")
            print(f"结束时间: {end_time.strftime('%Y-%m-%d %H:%M:%S UTC')}")
            print(f"总时长: {duration:.1f}秒 ({duration/60:.1f}分钟)")
            print()
            
            # RTK统计
            if self.rtk_receiver:
                rtk_stats = self.rtk_receiver.get_statistics()
                print(f"RTK接收:")
                print(f"  总接收: {rtk_stats['total_received']}条")
                print(f"  有效数据: {rtk_stats['valid_data']}条")
                print(f"  已保存: {rtk_stats.get('saved_count', 0)}条")
                print(f"  CSV文件: {rtk_stats.get('csv_files', 0)}个")
                print()
            
            # AIS统计
            if self.ais_receiver:
                ais_stats = self.ais_receiver.get_statistics()
                print(f"AIS接收:")
                print(f"  总接收: {ais_stats.get('total_received', 0)}条")
                print(f"  AIVDM: {ais_stats.get('valid_ais', 0)}条")
                print(f"  已保存: {ais_stats.get('saved_count', 0)}条")
                print(f"  CSV文件: {ais_stats.get('csv_files', 0)}个")
                print()
            
            # 相机统计
            if self.camera_manager:
                print(f"相机接收:")
                cam_stats = self.camera_manager.get_all_statistics()
                for cam_id, stats in sorted(cam_stats.items()):
                    print(f"  {cam_id}:")
                    print(f"    总帧数: {stats['total_frames']}")
                    print(f"    有效帧: {stats['valid_frames']}")
                    print(f"    已保存帧: {stats.get('saved_frames', 0)}")
                print()
            
            print("="*60)
            
            # 生成并保存元数据
            import json
            metadata = {
                'session_id': self.current_session_path.name,
                'start_time': self.start_time.isoformat() + 'Z' if self.start_time else None,
                'end_time': end_time.isoformat() + 'Z',
                'duration_seconds': duration,
                'rtk_stats': self.rtk_receiver.get_statistics() if self.rtk_receiver else {},
                'ais_stats': self.ais_receiver.get_statistics() if self.ais_receiver else {},
                'camera_stats': self.camera_manager.get_all_statistics() if self.camera_manager else {}
            }
            
            metadata_path = self.current_session_path / 'metadata.json'
            with open(metadata_path, 'w', encoding='utf-8') as f:
                json.dump(metadata, f, indent=2, ensure_ascii=False, default=str)
            
            logger.info(f"✅ 元数据已保存: {metadata_path}")
            logger.info("✅ 最终报告生成完成")
            
        except Exception as e:
            logger.error(f"生成最终报告失败: {e}")
    
    def run(self):
        """
        运行主流程
        
        完整流程：
        1. 加载和验证配置
        2. 验证设备连通性
        3. 初始化所有模块
        4. 等待首条数据并设置文件名
        5. 等待相机开始录制
        6. 启动录制（RTK和AIS开始保存）
        7. 等待用户中断
        8. 停止录制并生成报告
        """
        # 步骤1-2: 配置和设备验证
        if not self.load_and_validate_config():
            return False
        
        if not self.verify_device_connectivity():
            return False
        
        # 步骤3: 初始化模块
        if not self.initialize_modules():
            logger.error("模块初始化失败，退出")
            return False
        
        # 步骤4: 设置视频文件名（等待首条数据）
        if not self.setup_video_filenames():
            logger.error("设置文件名失败，退出")
            return False
        
        # 步骤5: 等待相机开始录制
        if not self.wait_for_cameras_ready():
            logger.error("等待相机就绪失败，退出")
            return False
        
        # 步骤6: 启动录制
        if not self.start_recording():
            logger.error("启动录制失败，退出")
            return False
        
        # 等待用户中断并定期打印状态
        logger.info("\n数据录制进行中...")
        logger.info("按 Ctrl+C 停止录制\n")
        
        last_status_time = time.time()
        status_interval = 10.0  # 每10秒打印一次状态
        
        try:
            while self.running:
                time.sleep(1.0)
                
                # 定期打印状态
                current_time = time.time()
                if current_time - last_status_time >= status_interval:
                    self._print_status()
                    last_status_time = current_time
                    
        except KeyboardInterrupt:
            logger.info("\n\n收到中断信号...")
        
        # 步骤6: 停止录制
        self.stop_recording()
        
        return True


def setup_logging(log_level: str = "INFO"):
    """
    配置日志系统
    
    Args:
        log_level: 日志级别
    """
    level = getattr(logging, log_level.upper(), logging.INFO)
    
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # 设置第三方库的日志级别
    logging.getLogger('PIL').setLevel(logging.WARNING)


def main():
    """主函数（CLI入口）"""
    parser = argparse.ArgumentParser(
        description='多相机-AIS-RTK数据集录制工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  python3 main.py --config config.yaml
  python3 main.py --config config.yaml --log-level DEBUG

说明:
  - 配置文件：使用config_template.yaml作为模板
  - 输出路径：在配置文件中指定
  - 停止录制：按 Ctrl+C
        """
    )

    DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")

    parser.add_argument(
        '--config',
        type=str,
        required=False,
        default=DEFAULT_CONFIG,
        help='配置文件路径（YAML格式）'
    )
    
    parser.add_argument(
        '--log-level',
        type=str,
        default='INFO',
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
        help='日志级别（默认: INFO）'
    )
    
    args = parser.parse_args()
    
    # 配置日志
    setup_logging(args.log_level)
    
    # 检查配置文件是否存在
    if not Path(args.config).exists():
        print(f"❌ 配置文件不存在: {args.config}")
        print(f"\n提示: 使用config_template.yaml创建配置文件")
        print(f"  cp config_template.yaml config.yaml")
        print(f"  vim config.yaml")
        return 1
    
    # 创建录制器
    recorder = DatasetRecorder(args.config)
    
    # 设置信号处理（优雅关闭）
    def signal_handler(sig, frame):
        logger.info("\n收到停止信号")
        recorder.stop_recording()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 运行录制器
    success = recorder.run()
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())

