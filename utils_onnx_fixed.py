import torch
import torch.nn as nn
import os
import onnxruntime as ort
from ultralytics import YOLO


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


# 3. 检查并修正类别数
current_nc = pytorch_model.model[-1].nc
print(f"当前模型类别数: {current_nc}")

# 先测试一下当前模型的输出形状
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
test_input = torch.randn(1, 3, input_shape[0], input_shape[1]).to(device)
pytorch_model = pytorch_model.to(device)

with torch.no_grad():
    test_output = pytorch_model(test_input)
    if isinstance(test_output, (list, tuple)):
        test_output = test_output[0]
    print(f"当前模型输出形状: {test_output.shape}")
    current_output_channels = test_output.shape[1]

# 计算期望的输出通道数
# YOLO输出格式: [batch, num_outputs, H, W]
# 其中 num_outputs = (4 + 1 + num_classes) * num_detection_layers
# 对于单层检测: num_outputs = 4 + 1 + num_classes
expected_channels = 4 + 1 + expected_num_classes
print(f"期望输出通道数: {expected_channels}, 当前输出通道数: {current_output_channels}")

if current_output_channels != expected_channels:
    print(f"⚠️  检测到输出通道数不匹配，需要修正...")
    
    # 获取检测头（Detect层）
    detect_layer = pytorch_model.model[-1]
    
    # 修改类别数属性
    detect_layer.nc = expected_num_classes
    
    # 对于YOLOv11，检测层结构可能是RTDETRDecoder或类似的
    # 需要找到并修改所有输出相关的卷积层
    
    def modify_output_layers(module, old_nc, new_nc):
        """递归修改输出层的通道数"""
        modified = False
        
        # 计算通道数变化
        channels_diff = new_nc - old_nc
        
        # 遍历所有子模块
        for name, child in list(module.named_children()):
            if isinstance(child, nn.Conv2d):
                # 检查是否是输出层（输出通道数匹配旧格式）
                # YOLO输出: (4 + 1 + nc) * num_layers
                # 对于单层: 4 + 1 + nc
                old_total_channels = 4 + 1 + old_nc
                if child.out_channels == old_total_channels or child.out_channels % old_total_channels == 0:
                    num_layers = child.out_channels // old_total_channels if child.out_channels % old_total_channels == 0 else 1
                    new_out_channels = (4 + 1 + new_nc) * num_layers
                    
                    print(f"  找到输出卷积层 '{name}': {child.out_channels} -> {new_out_channels}")
                    
                    # 创建新的卷积层
                    new_conv = nn.Conv2d(
                        child.in_channels,
                        new_out_channels,
                        child.kernel_size,
                        child.stride,
                        child.padding,
                        child.dilation,
                        child.groups,
                        child.bias is not None
                    )
                    
                    # 复制权重
                    with torch.no_grad():
                        # 复制前5个通道（bbox + objectness）
                        new_conv.weight.data[:4*num_layers] = child.weight.data[:4*num_layers]
                        new_conv.weight.data[4*num_layers:5*num_layers] = child.weight.data[4*num_layers:5*num_layers]
                        # 复制类别权重（只复制前min(old_nc, new_nc)个）
                        copy_classes = min(old_nc, new_nc)
                        new_conv.weight.data[5*num_layers:5*num_layers+copy_classes*num_layers] = \
                            child.weight.data[5*num_layers:5*num_layers+copy_classes*num_layers]
                        
                        if child.bias is not None:
                            new_conv.bias.data[:4*num_layers] = child.bias.data[:4*num_layers]
                            new_conv.bias.data[4*num_layers:5*num_layers] = child.bias.data[4*num_layers:5*num_layers]
                            new_conv.bias.data[5*num_layers:5*num_layers+copy_classes*num_layers] = \
                                child.bias.data[5*num_layers:5*num_layers+copy_classes*num_layers]
                    
                    # 替换模块
                    setattr(module, name, new_conv)
                    modified = True
            else:
                # 递归处理子模块
                if modify_output_layers(child, old_nc, new_nc):
                    modified = True
        
        return modified
    
    # 修改检测层的输出通道
    modify_output_layers(detect_layer, current_nc, expected_num_classes)
    
    # 验证修改后的输出
    with torch.no_grad():
        test_output_new = pytorch_model(test_input)
        if isinstance(test_output_new, (list, tuple)):
            test_output_new = test_output_new[0]
        print(f"修正后模型输出形状: {test_output_new.shape}")
        new_output_channels = test_output_new.shape[1]
        
        if new_output_channels == expected_channels:
            print(f"✅ 输出通道数修正成功: {new_output_channels}")
        else:
            print(f"⚠️  输出通道数仍不匹配: {new_output_channels} != {expected_channels}")
            print("   尝试使用YOLO内置导出方法...")
            # 如果直接修改失败，尝试使用YOLO的export方法
            model.model = pytorch_model
            model.export(format='onnx', imgsz=input_shape[0], simplify=True)
            print("   使用YOLO内置导出完成")
            exit(0)
else:
    print(f"✅ 输出通道数匹配，无需修改")


# 4. 使用torch.onnx.export导出
print(f"\n正在导出 ONNX 模型...")
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
    print("✅ ONNX 导出完成")
except Exception as e:
    print(f"❌ ONNX 导出失败: {e}")
    print("   尝试使用YOLO内置导出方法作为备选...")
    # 备选方案：使用YOLO内置导出
    model.model = pytorch_model
    model.export(format='onnx', imgsz=input_shape[0], simplify=True)
    # YOLO导出会生成带时间戳的文件名，需要重命名
    import glob
    onnx_files = glob.glob(os.path.join(os.path.dirname(output_onnx_path), "*.onnx"))
    if onnx_files:
        latest_onnx = max(onnx_files, key=os.path.getctime)
        if latest_onnx != output_onnx_path:
            os.rename(latest_onnx, output_onnx_path)
            print(f"   已重命名导出文件为: {output_onnx_path}")


# 5. 验证
print("\n正在验证 ONNX 模型...")
try:
    ort_session = ort.InferenceSession(output_onnx_path)
    input_name = ort_session.get_inputs()[0].name
    onnx_outputs = ort_session.run(None, {input_name: dummy_input.cpu().numpy()})
    
    actual_channels = onnx_outputs[0].shape[1]
    expected_channels = 4 + 1 + expected_num_classes
    
    print(f"\n期望通道数: {expected_channels}, 实际通道数: {actual_channels}")
    print(f"ONNX模型输出形状: {onnx_outputs[0].shape}")
    
    if actual_channels == expected_channels:
        print("✅ ONNX 模型导出成功！类别数正确！")
    else:
        print("❌ 导出失败，类别数仍不匹配")
        print(f"   差异: {expected_channels - actual_channels} 个通道")
except Exception as e:
    print(f"❌ ONNX 模型验证失败: {e}")
    raise
