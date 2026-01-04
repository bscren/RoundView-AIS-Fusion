# 该脚本用于导出DeepSORT特征提取模型的ONNX文件，并测试ONNX模型与PyTorch模型的输出误差
# 当误差小于1e-4时，认为ONNX模型与PyTorch模型输出一致

import torch
import numpy as np
import sys
import os

# 添加路径以便导入模型
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

from model import Net  # 导入DeepSORT特征提取模型

# -------------------------- 配置参数 --------------------------
model_path = "/home/tl/RV/src/marnav_vis/deep_sort/deep_sort/deep/checkpoint/ckpt.t7"  # 模型权重路径
output_onnx_path = "/home/tl/RV/src/marnav_vis/deep_sort/deep_sort/deep/checkpoint/ckpt.onnx"  # 导出的ONNX路径
opset_version = 19  # ONNX算子集版本（建议11+，兼容多数C++部署工具，但是yolov11m导出时采用19）
input_shape = [128, 64]  # 输入尺寸 [高, 宽]（需与模型训练时一致，对应3x128x64）
batch_size = 1  # 默认batch_size（用于导出，实际运行时支持动态batch）
# --------------------------------------------------------------

# 检查模型文件是否存在
if not os.path.exists(model_path):
    raise FileNotFoundError(f"模型文件不存在: {model_path}")

# 初始化模型（使用reid=True模式，用于特征提取）
print("="*60)
print("正在加载PyTorch模型...")
print("="*60)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"使用设备: {device}")

model = Net(reid=True).to(device)
model.eval()  # 切换到推理模式

# 加载权重
print(f"正在从 {model_path} 加载权重...")
checkpoint = torch.load(model_path, map_location=device)
if 'net_dict' in checkpoint:
    model.load_state_dict(checkpoint['net_dict'])
    print("✅ 权重加载成功（使用'net_dict'键）")
else:
    # 尝试直接加载state_dict（某些版本可能没有嵌套在字典中）
    try:
        model.load_state_dict(checkpoint)
        print("✅ 权重加载成功（直接加载state_dict）")
    except Exception as e:
        raise ValueError(f"无法加载模型权重，请检查checkpoint文件格式: {e}")

print(f"模型参数数量: {sum(p.numel() for p in model.parameters())}")

# 构造输入张量（batch_size=1，3通道，128x64，注意：高x宽）
# DeepSORT输入格式：batch_size x 3 x height(128) x width(64)
print("\n" + "="*60)
print("构造测试输入...")
print("="*60)
dummy_input = torch.randn(batch_size, 3, input_shape[0], input_shape[1]).to(device)
print(f"输入张量形状: {dummy_input.shape} (batch_size x channels x height x width)")

# 先测试一下PyTorch模型输出
with torch.no_grad():
    torch_output = model(dummy_input)
    print(f"PyTorch模型输出形状: {torch_output.shape}")
    print(f"输出特征维度: {torch_output.shape[1]} (应为512)")

# 导出ONNX
print("\n" + "="*60)
print("正在导出ONNX模型...")
print("="*60)
try:
    torch.onnx.export(
        model,  # 模型实例
        dummy_input,  # 输入张量
        output_onnx_path,  # 输出路径
        opset_version=opset_version,  # 算子集版本
        do_constant_folding=True,  # 常量折叠优化
        input_names=["input"],  # 输入节点名称
        output_names=["output"],  # 输出节点名称（特征向量）
        dynamic_axes={  # 动态维度（batch_size可动态变化，但图像尺寸固定）
            "input": {0: "batch_size"},  # batch维度动态
            "output": {0: "batch_size"}  # batch维度动态
        },
        verbose=False  # 不打印详细信息
    )
    print(f"✅ ONNX模型导出成功: {output_onnx_path}")
except Exception as e:
    print(f"❌ ONNX导出失败: {e}")
    raise

# 验证ONNX模型正确性
print("\n" + "="*60)
print("正在验证ONNX模型...")
print("="*60)

try:
    import onnxruntime as ort
    
    # 创建ONNX Runtime会话
    ort_session = ort.InferenceSession(output_onnx_path)
    
    # 准备输入数据（转换为numpy数组）
    onnx_input = dummy_input.cpu().numpy()
    
    # 运行ONNX模型
    onnx_outputs = ort_session.run(None, {"input": onnx_input})
    onnx_output = onnx_outputs[0]
    
    print(f"ONNX模型输出形状: {onnx_output.shape}")
    
    # 对比PyTorch与ONNX输出（确保误差在可接受范围）
    torch_output_np = torch_output.cpu().numpy()
    
    # 计算各种误差指标
    max_error = np.max(np.abs(torch_output_np - onnx_output))
    mean_error = np.mean(np.abs(torch_output_np - onnx_output))
    relative_error = np.max(np.abs(torch_output_np - onnx_output) / (np.abs(torch_output_np) + 1e-8))
    
    print(f"\n误差统计:")
    print(f"  最大绝对误差: {max_error:.2e}")
    print(f"  平均绝对误差: {mean_error:.2e}")
    print(f"  最大相对误差: {relative_error:.2e}")
    
    # 验证输出是否归一化（L2范数应接近1）
    onnx_norm = np.linalg.norm(onnx_output, axis=1)
    torch_norm = np.linalg.norm(torch_output_np, axis=1)
    print(f"\n特征向量归一化检查:")
    print(f"  ONNX输出L2范数 (应接近1.0): {onnx_norm}")
    print(f"  PyTorch输出L2范数 (应接近1.0): {torch_norm}")
    
    # 判断是否通过验证
    error_threshold = 1e-4
    if max_error < error_threshold:
        print(f"\n✅ 验证通过！最大误差 {max_error:.2e} < 阈值 {error_threshold}")
        print(f"✅ ONNX模型与PyTorch模型输出一致")
    else:
        print(f"\n⚠️  警告：最大误差 {max_error:.2e} >= 阈值 {error_threshold}")
        print(f"   模型输出可能存在差异，建议检查")
        
except ImportError:
    print("⚠️  onnxruntime未安装，跳过ONNX验证")
    print("   可以运行: pip install onnxruntime 来安装")
except Exception as e:
    print(f"❌ ONNX验证失败: {e}")
    raise

# 测试不同batch_size（如果支持动态batch）
print("\n" + "="*60)
print("测试不同batch_size...")
print("="*60)
try:
    import onnxruntime as ort
    ort_session = ort.InferenceSession(output_onnx_path)
    
    for test_batch_size in [1, 2, 4]:
        test_input = torch.randn(test_batch_size, 3, input_shape[0], input_shape[1])
        test_input_np = test_input.numpy()
        
        onnx_test_output = ort_session.run(None, {"input": test_input_np})[0]
        print(f"  batch_size={test_batch_size}: 输出形状 {onnx_test_output.shape} ✅")
        
except Exception as e:
    print(f"  动态batch测试失败: {e}")

print("\n" + "="*60)
print("导出完成！")
print("="*60)
print(f"ONNX模型文件: {output_onnx_path}")
print(f"输入格式: batch_size x 3 x 128 x 64 (batch_size可动态)")
print(f"输出格式: batch_size x 512 (归一化的特征向量)")
print("\n使用说明:")
print("  1. ONNX模型可以在C++中使用ONNX Runtime进行推理")
print("  2. 输入图像需要预处理:")
print("     - 调整尺寸到 128x64 (高x宽)")
print("     - 归一化到[0,1]并转换为RGB")
print("     - 使用ImageNet均值和标准差归一化: mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225]")
print("     - 转换为NCHW格式 (batch_size x 3 x 128 x 64)")
print("  3. 输出是L2归一化的512维特征向量，可用于计算余弦相似度")

