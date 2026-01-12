#!/usr/bin/env python3
import yaml
import numpy as np

# 加载YAML
with open('CameraParams_SiteTest.yaml', 'r') as f:
    config = yaml.safe_load(f)

camera_idx = 0  # 调整第一个相机

# 1. 调整焦距（缩放）
config['cameras'][camera_idx]['focal'] *= 1.05  # +5%
config['cameras'][camera_idx]['K_matrix']['data'][0] = config['cameras'][camera_idx]['focal']
config['cameras'][camera_idx]['K_matrix']['data'][4] = config['cameras'][camera_idx]['focal']

# 2. 调整主点（平移）
config['cameras'][camera_idx]['ppx'] += 20  # 向右移20像素
config['cameras'][camera_idx]['K_matrix']['data'][2] = config['cameras'][camera_idx]['ppx']

# 3. 调整旋转（使用上面的欧拉角函数）
# R_old = np.array(config['cameras'][camera_idx]['R']['data']).reshape(3, 3)
# roll, pitch, yaw = rotation_matrix_to_euler_angles(R_old)
# yaw_new = yaw + 2.0  # 向右转2度
# R_new = euler_angles_to_rotation_matrix(roll, pitch, yaw_new)
# config['cameras'][camera_idx]['R']['data'] = R_new.flatten().tolist()

# 保存新配置
with open('CameraParams_SiteTest_tuned.yaml', 'w') as f:
    yaml.dump(config, f, default_flow_style=False)

print("✅ 调整后的参数已保存到 CameraParams_SiteTest_tuned.yaml")