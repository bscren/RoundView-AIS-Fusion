import numpy as np
import cv2

# === 1. 从现有 R 矩阵提取欧拉角（XYZ顺序） ===
R_old = np.array([
    [ 9.97237206e-01, 9.80765466e-03, 7.36328959e-02],
    [ 7.46568106e-03, 9.72993910e-01, -2.30710760e-01],
    [ -7.39070326e-02, 2.30623096e-01, 9.70232487e-01 ]
], dtype=np.float32)

# 提取欧拉角（弧度）
def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])  # Roll (横滚)
        y = np.arctan2(-R[2,0], sy)      # Pitch (俯仰)
        z = np.arctan2(R[1,0], R[0,0])   # Yaw (偏航)
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    
    return np.degrees([x, y, z])  # 转为角度

roll, pitch, yaw = rotation_matrix_to_euler_angles(R_old)
print(f"当前欧拉角: Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°")

# === 2. 微调欧拉角 ===
roll_new = roll         # roll 意思是 横滚角
pitch_new = pitch + 1  # pitch 意思是 俯仰角
yaw_new = yaw          # yaw 意思是 偏航角

# === 3. 重新计算 R 矩阵 ===
def euler_angles_to_rotation_matrix(roll, pitch, yaw):
    # 角度转弧度
    roll_rad = np.radians(roll)
    pitch_rad = np.radians(pitch)
    yaw_rad = np.radians(yaw)
    
    # 绕 X 轴旋转（Roll）
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll_rad), -np.sin(roll_rad)],
        [0, np.sin(roll_rad), np.cos(roll_rad)]
    ])
    
    # 绕 Y 轴旋转（Pitch）
    Ry = np.array([
        [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
        [0, 1, 0],
        [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
    ])
    
    # 绕 Z 轴旋转（Yaw）
    Rz = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0],
        [0, 0, 1]
    ])
    
    # 组合：R = Rz * Ry * Rx
    R = Rz @ Ry @ Rx
    return R.astype(np.float32)

R_new = euler_angles_to_rotation_matrix(roll_new, pitch_new, yaw_new)

# === 4. 输出新的 R 矩阵（YAML格式） ===
print("\n新的 R 矩阵:")
print("data: [", end="")
for i in range(3):
    for j in range(3):
        print(f"{R_new[i,j]:.8e}", end="")
        if i < 2 or j < 2:
            print(", ", end="")
print(" ]")