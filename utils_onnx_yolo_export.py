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


# 2. 诊断当前模型输出
print("\n=== 诊断模型输出 ===")
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
pytorch_model = model.model.eval().to(device)
test_input = torch.randn(1, 3, input_shape[0], input_shape[1]).to(device)

with torch.no_grad():
    test_output = pytorch_model(test_input)
    if isinstance(test_output, (list, tuple)):
        test_output = test_output[0]
    print(f"PyTorch模型输出形状: {test_output.shape}")
    current_channels = test_output.shape[1]

expected_channels = 4 + 1 + expected_num_classes
print(f"期望通道数: {expected_channels}, 当前通道数: {current_channels}")


# 3. 如果通道数不匹配，修正模型
if current_channels != expected_channels:
    print(f"\n⚠️  检测到通道数不匹配，开始修正...")
    
    detect_layer = pytorch_model.model[-1]
    old_nc = detect_layer.nc
    
    print(f"检测层类型: {type(detect_layer).__name__}")
    print(f"当前nc属性: {old_nc}")
    
    # 修改nc属性
    detect_layer.nc = expected_num_classes
    
    # 查找输出卷积层并修改
    def modify_output_conv(module):
        """修改输出卷积层的通道数"""
        for name, child in module.named_children():
            if isinstance(child, nn.Conv2d):
                # 检查是否是输出层
                if child.out_channels == current_channels:
                    print(f"\n找到输出卷积层: {name}")
                    print(f"  当前: in={child.in_channels}, out={child.out_channels}")
                    
                    # 创建新卷积层
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
                        # bbox (4) + objectness (1) + classes
                        new_conv.weight.data[:5] = child.weight.data[:5]
                        old_class_channels = current_channels - 5
                        copy_classes = min(old_class_channels, expected_num_classes)
                        if copy_classes > 0:
                            new_conv.weight.data[5:5+copy_classes] = child.weight.data[5:5+copy_classes]
                        # 如果新类别更多，用平均值初始化
                        if expected_num_classes > old_class_channels and old_class_channels > 0:
                            avg = child.weight.data[5:5+old_class_channels].mean(dim=0, keepdim=True)
                            new_conv.weight.data[5+copy_classes:] = avg.repeat(expected_num_classes - copy_classes, 1, 1, 1)
                        
                        if child.bias is not None:
                            new_conv.bias.data[:5] = child.bias.data[:5]
                            if copy_classes > 0:
                                new_conv.bias.data[5:5+copy_classes] = child.bias.data[5:5+copy_classes]
                            if expected_num_classes > old_class_channels and old_class_channels > 0:
                                avg_bias = child.bias.data[5:5+old_class_channels].mean()
                                new_conv.bias.data[5+copy_classes:] = avg_bias
                    
                    # 替换
                    setattr(module, name, new_conv)
                    print(f"  修改后: in={new_conv.in_channels}, out={new_conv.out_channels}")
                    return True
            
            # 递归处理
            if isinstance(child, nn.Module):
                if modify_output_conv(child):
                    return True
        return False
    
    if modify_output_conv(detect_layer):
        # 验证修改
        with torch.no_grad():
            test_output_new = pytorch_model(test_input)
            if isinstance(test_output_new, (list, tuple)):
                test_output_new = test_output_new[0]
            new_channels = test_output_new.shape[1]
            print(f"\n修正后输出形状: {test_output_new.shape}")
            
            if new_channels == expected_channels:
                print(f"✅ 通道数修正成功!")
            else:
                raise ValueError(f"修正失败: {new_channels} != {expected_channels}")
    else:
        raise ValueError("未找到输出卷积层")


# 4. 使用YOLO内置导出（更可靠）
print(f"\n=== 导出 ONNX 模型 ===")
# 更新model对象中的模型
model.model = pytorch_model

# 使用YOLO的export方法
try:
    # YOLO的export会生成带时间戳的文件名，我们需要指定输出路径
    # 但YOLO export不支持直接指定输出文件名，所以先用临时文件名
    temp_onnx = output_onnx_path.replace('.onnx', '_temp.onnx')
    
    # 使用export方法
    model.export(
        format='onnx',
        imgsz=input_shape[0],
        simplify=True,
        opset=12,
        dynamic=False,  # 使用固定batch size
    )
    
    # YOLO会在模型目录生成onnx文件，找到最新的
    import glob
    model_dir = os.path.dirname(model_path)
    onnx_files = glob.glob(os.path.join(model_dir, "*.onnx"))
    if onnx_files:
        # 找到最新的onnx文件
        latest_onnx = max(onnx_files, key=os.path.getctime)
        print(f"YOLO导出的文件: {latest_onnx}")
        
        # 移动到目标位置
        if os.path.exists(output_onnx_path):
            os.remove(output_onnx_path)
        os.rename(latest_onnx, output_onnx_path)
        print(f"✅ ONNX导出完成: {output_onnx_path}")
    else:
        raise FileNotFoundError("未找到YOLO导出的ONNX文件")
        
except Exception as e:
    print(f"YOLO导出失败: {e}")
    print("尝试使用torch.onnx.export...")
    
    # 备选方案：直接使用torch.onnx.export
    dummy_input = torch.randn(1, 3, input_shape[0], input_shape[1]).to(device)
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
    print(f"✅ torch.onnx.export完成: {output_onnx_path}")


# 5. 验证ONNX模型
print(f"\n=== 验证 ONNX 模型 ===")
try:
    ort_session = ort.InferenceSession(output_onnx_path)
    input_name = ort_session.get_inputs()[0].name
    dummy_input_cpu = torch.randn(1, 3, input_shape[0], input_shape[1])
    onnx_outputs = ort_session.run(None, {input_name: dummy_input_cpu.numpy()})
    
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
