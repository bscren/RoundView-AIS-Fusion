# 该脚本用于导出 Ultralytics YOLO 系列模型（如 YOLOv11n）为 ONNX 格式
# 并测试 ONNX 模型与 PyTorch 模型的输出误差

import torch
import numpy as np
# 关键修改：从 ultralytics 库导入 YOLO 类
from ultralytics import YOLO

# -------------------------- 配置参数 --------------------------
model_path = "/home/tl/RV/src/marnav_vis/detection_yolox/model_data/Yolo11m.pt"  # 模型权重路径
output_onnx_path = "/home/tl/RV/src/marnav_vis/detection_yolox/model_data/Yolo11m.onnx"  # 导出的 ONNX 路径
opset_version = 12  # ONNX 算子集版本（建议 11+，ultralytics 推荐 12）
input_shape = [640, 640]  # 输入尺寸 (height, width)
# --------------------------------------------------------------

# 关键修改：使用 Ultralytics 的 YOLO 类加载模型
print(f"正在从 {model_path} 加载模型...")
model = YOLO(model_path)
model.fuse() # 融合 Conv2d + BatchNorm2d 层以优化推理

# 关键修改：从 export 方法中移除 input_names 和 output_names
# ultralytics 会自动处理输入输出节点名，输入名为 'images'
print(f"正在导出 ONNX 模型到 {output_onnx_path}...")
success = model.export(
    format="onnx",
    opset=opset_version,
    imgsz=input_shape,
    dynamic=False,  # 固定输入尺寸
    simplify=True,   # 简化 ONNX 模型
    exist_ok=True # 如果文件已存在，覆盖它

)

if success:
    print(f"ONNX 模型导出成功：{output_onnx_path}")
else:
    print("ONNX 模型导出失败！")
    exit()

# 验证 ONNX 模型正确性
print("\n正在验证 ONNX 模型...")
import onnxruntime as ort
import os
# 1. 使用 PyTorch 模型获取原始输出
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
input_tensor = torch.randn(1, 3, input_shape[0], input_shape[1]).to(device)
with torch.no_grad():
    # 直接调用模型的底层结构以获取原始特征图输出
    torch_outputs = model.model(input_tensor)

# 2. 使用 ONNX Runtime 获取输出
ort_session = ort.InferenceSession(output_onnx_path)
onnx_input = input_tensor.cpu().numpy()

# 关键修改：ONNX Runtime 的输入节点名必须为 'images'
onnx_outputs = ort_session.run(None, {"images": onnx_input})

# 3. 对比 PyTorch 与 ONNX 输出
print(f"\nPyTorch 输出数量: {len(torch_outputs)}")
print(f"ONNX 输出数量: {len(onnx_outputs)}")

for i, (torch_out, onnx_out) in enumerate(zip(torch_outputs, onnx_outputs)):
    print(f"\n--- 输出尺度 {i+1} ---")
    print(f"PyTorch 输出形状: {torch_out.shape}")
    print(f"ONNX 输出形状: {onnx_out.shape}")
    
    torch_out_np = torch_out.cpu().numpy()
    
    # 计算最大绝对误差
    error = np.max(np.abs(torch_out_np - onnx_out))
    print(f"最大误差 (Max Error): {error:.8f}")
    
    # 计算均方根误差 (RMSE)
    rmse = np.sqrt(np.mean((torch_out_np - onnx_out) ** 2))
    print(f"均方根误差 (RMSE): {rmse:.8f}")

print("\n验证完成。如果误差接近 0，则说明 ONNX 模型导出正确。")