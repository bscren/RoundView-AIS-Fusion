import torch
import os
import onnxruntime as ort
from ultralytics import YOLO

model_path = "/home/tl/RV/src/marnav_vis/detection_yolox/model_data/Yolo11m.pt"
output_onnx_path = "/home/tl/RV/src/marnav_vis/detection_yolox/model_data/Yolo11m.onnx"
input_shape = [640, 640]
expected_num_classes = 7

# 1. 加载模型
print(f"正在从 {model_path} 加载模型...")
model = YOLO(model_path)
print(f"模型类别数: {model.model.nc}")

# 2. 获取底层模型并验证
pytorch_model = model.model
pytorch_model.eval()

# 3. 检查并修正类别数
if pytorch_model.model[-1].nc != expected_num_classes:
    print(f"⚠️  检测到类别数不匹配: {pytorch_model.model[-1].nc} != {expected_num_classes}")
    print("尝试修正...")
    # 修改输出层的类别数
    pytorch_model.model[-1].nc = expected_num_classes
    # 重新构建输出层（如果需要）
    # 注意：这可能需要重新加载权重

print(f"修正后输出层类别数: {pytorch_model.model[-1].nc}")

# 4. 使用torch.onnx.export导出
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
dummy_input = torch.randn(1, 3, input_shape[0], input_shape[1]).to(device)
pytorch_model = pytorch_model.to(device)

print(f"\n正在导出 ONNX 模型...")
torch.onnx.export(
    pytorch_model,
    dummy_input,
    output_onnx_path,
    export_params=True,
    opset_version=12,
    do_constant_folding=True,
    input_names=['images'],
    output_names=['output0'],
    dynamic_axes={'images': {0: 'batch_size'}, 'output0': {0: 'batch_size'}}
)

# 5. 验证
print("\n正在验证 ONNX 模型...")
ort_session = ort.InferenceSession(output_onnx_path)
input_name = ort_session.get_inputs()[0].name
onnx_outputs = ort_session.run(None, {input_name: dummy_input.cpu().numpy()})

actual_channels = onnx_outputs[0].shape[1]
expected_channels = 4 + 1 + expected_num_classes

print(f"\n期望通道数: {expected_channels}, 实际通道数: {actual_channels}")
if actual_channels == expected_channels:
    print("✅ ONNX 模型导出成功！")
else:
    print("❌ 导出失败，类别数仍不匹配")