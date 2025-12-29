import torch
import torch.nn as nn
import os
import onnxruntime as ort
from ultralytics import YOLO
import copy


model_path = "/home/jh-train/JHYoloTrain/train/runs/detect/train21/weights/best.pt"
output_onnx_path = "/home/jh-train/JHYoloTrain/train/runs/detect/train21/weights/Yolo11n.onnx"
input_shape = [640, 640]
expected_num_classes = 7


# 1. 加载模型
print(f"正在从 {model_path} 加载模型...")
model = YOLO(model_path)
print(f"模型类别数: {model.model.nc}")


# 2. 获取底层模型并验证
pytorch_model = model.model
pytorch_model.eval()


# 3. 诊断模型结构
print("\n=== 模型结构诊断 ===")
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
test_input = torch.randn(1, 3, input_shape[0], input_shape[1]).to(device)
pytorch_model = pytorch_model.to(device)

with torch.no_grad():
    test_output = pytorch_model(test_input)
    if isinstance(test_output, (list, tuple)):
        test_output = test_output[0]
    print(f"模型输出形状: {test_output.shape}")
    current_output_channels = test_output.shape[1]

# 计算期望的输出通道数
expected_channels = 4 + 1 + expected_num_classes  # bbox(4) + objectness(1) + classes(7)
print(f"期望输出通道数: {expected_channels}")
print(f"当前输出通道数: {current_output_channels}")

# 检查检测层结构
detect_layer = pytorch_model.model[-1]
print(f"\n检测层类型: {type(detect_layer).__name__}")
print(f"检测层属性: {[attr for attr in dir(detect_layer) if not attr.startswith('_')]}")

# 打印检测层的所有卷积层
print("\n检测层中的卷积层:")
for name, module in detect_layer.named_modules():
    if isinstance(module, nn.Conv2d):
        print(f"  {name}: in_channels={module.in_channels}, out_channels={module.out_channels}")


# 4. 修正输出层
if current_output_channels != expected_channels:
    print(f"\n⚠️  需要修正输出通道数: {current_output_channels} -> {expected_channels}")
    
    # 计算需要修改的通道数
    channels_diff = expected_num_classes - (current_output_channels - 5)  # 5 = 4(bbox) + 1(objectness)
    print(f"类别数差异: {channels_diff}")
    
    # 修改类别数属性
    detect_layer.nc = expected_num_classes
    
    # 查找并修改输出卷积层
    def find_and_modify_output_conv(module, parent_name="", depth=0):
        """递归查找并修改输出卷积层"""
        modified = False
        
        for name, child in list(module.named_children()):
            full_name = f"{parent_name}.{name}" if parent_name else name
            
            if isinstance(child, nn.Conv2d):
                # 检查是否是输出层
                # 输出层的out_channels应该是 (4 + 1 + num_classes) 的倍数
                total_per_output = current_output_channels
                if child.out_channels == total_per_output:
                    print(f"\n找到输出卷积层: {full_name}")
                    print(f"  当前: in={child.in_channels}, out={child.out_channels}")
                    
                    # 创建新的卷积层
                    new_conv = nn.Conv2d(
                        child.in_channels,
                        expected_channels,
                        child.kernel_size,
                        child.stride,
                        child.padding,
                        child.dilation,
                        child.groups,
                        child.bias is not None
                    )
                    
                    # 复制权重
                    with torch.no_grad():
                        # 复制bbox权重 (前4个通道)
                        new_conv.weight.data[:4] = child.weight.data[:4]
                        # 复制objectness权重 (第5个通道)
                        new_conv.weight.data[4:5] = child.weight.data[4:5]
                        # 复制类别权重
                        old_num_classes = current_output_channels - 5
                        copy_classes = min(old_num_classes, expected_num_classes)
                        new_conv.weight.data[5:5+copy_classes] = child.weight.data[5:5+copy_classes]
                        # 如果新类别数更多，初始化新类别权重为0
                        if expected_num_classes > old_num_classes:
                            # 使用最后几个类别的平均值初始化新类别
                            if old_num_classes > 0:
                                avg_weight = child.weight.data[5:5+old_num_classes].mean(dim=0, keepdim=True)
                                new_conv.weight.data[5+copy_classes:] = avg_weight.repeat(expected_num_classes - copy_classes, 1, 1, 1)
                        
                        # 复制bias
                        if child.bias is not None:
                            new_conv.bias.data[:4] = child.bias.data[:4]
                            new_conv.bias.data[4:5] = child.bias.data[4:5]
                            new_conv.bias.data[5:5+copy_classes] = child.bias.data[5:5+copy_classes]
                            if expected_num_classes > old_num_classes:
                                if old_num_classes > 0:
                                    avg_bias = child.bias.data[5:5+old_num_classes].mean()
                                    new_conv.bias.data[5+copy_classes:] = avg_bias
                    
                    # 替换模块
                    if parent_name:
                        parent = module
                        for part in parent_name.split('.'):
                            parent = getattr(parent, part)
                        setattr(parent, name, new_conv)
                    else:
                        setattr(module, name, new_conv)
                    
                    print(f"  修改后: in={new_conv.in_channels}, out={new_conv.out_channels}")
                    modified = True
                    break
            
            # 递归处理子模块
            if isinstance(child, nn.Module):
                if find_and_modify_output_conv(child, full_name, depth+1):
                    modified = True
                    break
        
        return modified
    
    # 执行修改
    if find_and_modify_output_conv(detect_layer):
        print("✅ 输出层修改成功")
        
        # 验证修改后的输出
        with torch.no_grad():
            test_output_new = pytorch_model(test_input)
            if isinstance(test_output_new, (list, tuple)):
                test_output_new = test_output_new[0]
            new_output_channels = test_output_new.shape[1]
            print(f"\n修正后输出形状: {test_output_new.shape}")
            
            if new_output_channels == expected_channels:
                print(f"✅ 输出通道数修正成功: {new_output_channels}")
            else:
                print(f"❌ 输出通道数仍不匹配: {new_output_channels} != {expected_channels}")
                raise ValueError("无法修正输出通道数")
    else:
        print("❌ 未找到输出卷积层，尝试其他方法...")
        raise ValueError("无法找到输出层进行修改")
else:
    print(f"\n✅ 输出通道数已匹配，无需修改")


# 5. 导出ONNX
print(f"\n=== 导出 ONNX 模型 ===")
dummy_input = torch.randn(1, 3, input_shape[0], input_shape[1]).to(device)

try:
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
    print(f"✅ ONNX 导出完成: {output_onnx_path}")
except Exception as e:
    print(f"❌ ONNX 导出失败: {e}")
    raise


# 6. 验证ONNX模型
print(f"\n=== 验证 ONNX 模型 ===")
try:
    ort_session = ort.InferenceSession(output_onnx_path)
    input_name = ort_session.get_inputs()[0].name
    onnx_outputs = ort_session.run(None, {input_name: dummy_input.cpu().numpy()})
    
    actual_channels = onnx_outputs[0].shape[1]
    
    print(f"期望通道数: {expected_channels}")
    print(f"实际通道数: {actual_channels}")
    print(f"ONNX模型输出形状: {onnx_outputs[0].shape}")
    
    if actual_channels == expected_channels:
        print("\n✅✅✅ ONNX 模型导出成功！类别数正确！")
    else:
        print(f"\n❌ 导出失败，类别数仍不匹配")
        print(f"   差异: {expected_channels - actual_channels} 个通道")
        raise ValueError("ONNX模型类别数验证失败")
except Exception as e:
    print(f"❌ ONNX 模型验证失败: {e}")
    raise
