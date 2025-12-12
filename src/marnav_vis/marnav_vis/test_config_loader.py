#!/usr/bin/env python3
"""
配置加载器测试脚本
用于验证YAML配置文件能否正确加载
"""

import sys
from pathlib import Path

# 添加marnav_vis模块到路径
sys.path.insert(0, str(Path(__file__).parent / 'marnav_vis'))

from config_loader import ConfigLoader


def test_config_file(config_file_path):
    """测试配置文件加载"""
    print("="*70)
    print(f"测试配置文件: {config_file_path}")
    print("="*70)
    
    try:
        # 加载配置
        config_loader = ConfigLoader(config_file_path)
        print("✅ 配置文件加载成功\n")
        
        # 测试相机配置
        print("📹 相机配置:")
        camera_config = config_loader.get_camera_config()
        print(f"  - 图像尺寸: {camera_config.get('width_height', 'N/A')}")
        print(f"  - 发布频率: {camera_config.get('camera_publish_fps', 'N/A')} Hz")
        
        if 'video_path' in camera_config:
            print(f"  - 视频路径: {camera_config.get('video_path')}")
        
        cameras = camera_config.get('cameras', [])
        print(f"  - 相机数量: {len(cameras)}")
        for i, cam in enumerate(cameras):
            print(f"    相机{i}: {cam.get('camera_name')} -> {cam.get('topic_name')}")
        print()
        
        # 测试GNSS配置
        print("📡 GNSS配置:")
        gnss_config = config_loader.get_gnss_config()
        print(f"  - 发布频率: {gnss_config.get('gnss_publish_rate', 'N/A')} Hz")
        print(f"  - 发布话题: {gnss_config.get('gnss_pub_topic', 'N/A')}")
        
        if 'camera_gnss_para' in gnss_config:
            gnss_para = gnss_config['camera_gnss_para']
            print(f"  - 经纬度: Lon={gnss_para.get('lon')}, Lat={gnss_para.get('lat')}")
            print(f"  - 朝向: 水平={gnss_para.get('horizontal_orientation')}°, 垂直={gnss_para.get('vertical_orientation')}°")
            print(f"  - 相机高度: {gnss_para.get('camera_height')} m")
        print()
        
        # 测试AIS配置
        print("🚢 AIS配置:")
        ais_config = config_loader.get_ais_config()
        if 'ais_csv_folder' in ais_config:
            print(f"  - CSV文件夹: {ais_config.get('ais_csv_folder')}")
        if 'ais_start_timestamp' in ais_config:
            print(f"  - 起始时间戳: {ais_config.get('ais_start_timestamp')} ms")
        print(f"  - CSV话题: {ais_config.get('ais_csv_topic', 'N/A')}")
        print(f"  - 批量话题: {ais_config.get('ais_batch_pub_topic', 'N/A')}")
        print()
        
        # 测试DeepSORVF配置
        print("🎯 DeepSORVF配置:")
        deepsorvf_config = config_loader.get_deepsorvf_config()
        print(f"  - 输入/输出FPS: {deepsorvf_config.get('input_fps', 'N/A')}/{deepsorvf_config.get('output_fps', 'N/A')}")
        print(f"  - 处理间隔: {deepsorvf_config.get('skip_interval', 'N/A')} ms")
        print(f"  - 同步队列大小: {deepsorvf_config.get('sync_queue_size', 'N/A')}")
        print(f"  - 同步时间误差: {deepsorvf_config.get('sync_slop', 'N/A')} s")
        print(f"  - 融合轨迹话题: {deepsorvf_config.get('fus_trajectory_topic', 'N/A')}")
        print()
        
        print("="*70)
        print("✅ 所有配置项测试通过")
        print("="*70)
        return True
        
    except FileNotFoundError as e:
        print(f"❌ 错误: 配置文件不存在 - {e}")
        return False
    except Exception as e:
        print(f"❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """主函数"""
    # 获取配置文件目录
    config_dir = Path(__file__).parent / 'config'
    
    # 测试离线配置
    offline_config = config_dir / 'track_offline_config.yaml'
    if offline_config.exists():
        test_config_file(str(offline_config))
        print()
    else:
        print(f"⚠️  离线配置文件不存在: {offline_config}")
    
    # 测试实时配置
    realtime_config = config_dir / 'track_realtime_config.yaml'
    if realtime_config.exists():
        test_config_file(str(realtime_config))
        print()
    else:
        print(f"⚠️  实时配置文件不存在: {realtime_config}")
    
    print("\n测试完成！")


if __name__ == '__main__':
    main()

