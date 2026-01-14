# 该脚本用于导出YOLOX模型的ONNX文件，并测试ONNX模型与PyTorch模型的输出误差
# 当误差小于1e-4时，认为ONNX模型与PyTorch模型输出一致

import torch
import numpy as np
from nets.yolo import YoloBody  # 导入模型结构
from utils.utils import preprocess_input  # 导入预处理函数（如需）
from ament_index_python.packages import get_package_share_directory
import os
# -------------------------- 配置参数 --------------------------
# 获取包共享目录
package_share_dir = get_package_share_directory('marnav_vis')
model_path = os.path.join(package_share_dir, 'detection_yolox', 'model_data', 'YOLOX-final.pth')
classes_path = os.path.join(package_share_dir, 'detection_yolox', 'model_data', 'ship_classes.txt')
#
# model_path = "/home/tl/RV/src/marnav_vis/detection_yolox/model_data/YOLOX-final.pth"  # 模型权重路径
# classes_path = "/home/tl/RV/src/marnav_vis/detection_yolox/model_data/ship_classes.txt"  # 类别文件
input_shape = [640, 640]  # 输入尺寸（需与模型训练时一致）
phi = "n"  # 模型版本（s/m/l/x）
ouput_onnx_path = os.path.join(package_share_dir, 'detection_yolox', 'model_data', 'yolox.onnx')
# output_onnx_path = "/home/tl/RV/src/marnav_vis/detection_yolox/model_data/yolox.onnx"  # 导出的ONNX路径
opset_version = 11  # ONNX算子集版本（建议11+，兼容多数C++部署工具）
# --------------------------------------------------------------

# 加载类别数（用于初始化模型）
def get_classes(classes_path):
    with open(classes_path, encoding='utf-8') as f:
        class_names = f.readlines()
    class_names = [c.strip() for c in class_names]
    return class_names, len(class_names)

class_names, num_classes = get_classes(classes_path)

# 初始化模型并加载权重
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = YoloBody(num_classes, phi).to(device)
model.load_state_dict(torch.load(model_path, map_location=device, weights_only=False))
model.eval()  # 切换到推理模式

# 构造输入张量（batch_size=1，3通道，640x640）
input_tensor = torch.randn(1, 3, input_shape[0], input_shape[1]).to(device)

# 导出ONNX
torch.onnx.export(
    model,  # 模型实例
    input_tensor,  # 输入张量
    output_onnx_path,  # 输出路径
    opset_version=opset_version,  # 算子集版本
    do_constant_folding=True,  # 常量折叠优化
    input_names=["input"],  # 输入节点名称
    output_names=["output0", "output1", "output2"],  # 输出节点名称（对应3个尺度的特征图）
    dynamic_axes={  # 动态维度（batch和宽高可动态变化）
        "input": {0: "batch_size", 2: "height", 3: "width"},
        "output0": {0: "batch_size", 2: "height0", 3: "width0"},
        "output1": {0: "batch_size", 2: "height1", 3: "width1"},
        "output2": {0: "batch_size", 2: "height2", 3: "width2"}
    }
)

print(f"ONNX模型导出成功：{output_onnx_path}")

# 验证ONNX模型正确性
import onnxruntime as ort
ort_session = ort.InferenceSession(output_onnx_path)
onnx_input = input_tensor.cpu().numpy()  # 转换为numpy数组
onnx_outputs = ort_session.run(None, {"input": onnx_input})

# 对比PyTorch与ONNX输出（确保误差在可接受范围）
with torch.no_grad():
    torch_outputs = model(input_tensor)
for torch_out, onnx_out in zip(torch_outputs, onnx_outputs):
    print(f"PyTorch输出形状：{torch_out.shape}，ONNX输出形状：{onnx_out.shape}")
    # 计算误差（应接近0）
    error = np.max(np.abs(torch_out.cpu().numpy() - onnx_out))
    print(f"最大误差：{error}")