"""
检查 .pth 模型文件对应的 YOLOX 模型结构（phi 参数）
"""
import torch
import sys
import os

# 添加路径以便导入
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def check_model_structure(model_path):
    """
    检查模型文件的结构信息，推断对应的 phi 参数
    
    参数:
        model_path: .pth 模型文件路径
    
    返回:
        dict: 包含模型结构信息的字典
    """
    print(f"正在检查模型文件: {model_path}\n")
    
    # 加载模型文件
    try:
        checkpoint = torch.load(model_path, map_location='cpu', weights_only=False)
    except Exception as e:
        print(f"❌ 加载模型文件失败: {e}")
        return None
    
    # 检查是否是 state_dict 格式
    if isinstance(checkpoint, dict):
        # 如果是完整的 checkpoint（包含 'state_dict' 或其他键）
        if 'state_dict' in checkpoint:
            state_dict = checkpoint['state_dict']
            print("📦 检测到完整 checkpoint 格式（包含 state_dict）")
        elif 'model' in checkpoint:
            state_dict = checkpoint['model']
            print("📦 检测到完整 checkpoint 格式（包含 model）")
        else:
            # 直接是 state_dict
            state_dict = checkpoint
            print("📦 检测到纯 state_dict 格式")
    else:
        state_dict = checkpoint
        print("📦 检测到纯 state_dict 格式")
    
    # 获取所有层名称
    layer_names = list(state_dict.keys())
    
    # 统计参数信息
    total_params = 0
    param_info = {}
    
    print("\n" + "="*60)
    print("模型结构分析")
    print("="*60)
    
    # 分析关键层来确定模型结构
    # YOLOX 不同版本的关键区别在于 backbone 的通道数
    backbone_channels = []
    head_channels = []
    
    for name, param in state_dict.items():
        param_size = param.numel()
        total_params += param_size
        
        # 检查 backbone 相关层
        if 'backbone' in name:
            if 'stem' in name or 'conv' in name:
                if len(param.shape) >= 2:
                    # 获取输出通道数
                    out_channels = param.shape[0]
                    if out_channels not in backbone_channels:
                        backbone_channels.append(out_channels)
        
        # 检查 head 相关层
        if 'head' in name:
            if 'stems' in name or 'cls_preds' in name or 'reg_preds' in name:
                if len(param.shape) >= 2:
                    out_channels = param.shape[0]
                    if out_channels not in head_channels:
                        head_channels.append(out_channels)
    
    # 根据通道数推断 phi
    # YOLOX 的 width 参数影响通道数：
    # s: 0.50, m: 0.75, l: 1.00, x: 1.25
    # 基础通道数通常是 256, 512, 1024
    
    print(f"\n📊 参数统计:")
    print(f"   总参数量: {total_params:,} ({total_params/1e6:.2f}M)")
    print(f"   总层数: {len(layer_names)}")
    
    print(f"\n🔍 Backbone 通道数: {sorted(set(backbone_channels))[:10]}")  # 只显示前10个
    print(f"🔍 Head 通道数: {sorted(set(head_channels))[:10]}")
    
    # 尝试推断 phi
    print(f"\n" + "="*60)
    print("模型结构推断")
    print("="*60)
    
    # 方法1: 根据参数量推断
    param_ranges = {
        's': (5e6, 15e6),      # 约 5-15M 参数
        'm': (15e6, 30e6),     # 约 15-30M 参数
        'l': (30e6, 60e6),     # 约 30-60M 参数
        'x': (60e6, 150e6),    # 约 60-150M 参数
    }
    
    inferred_phi_by_params = None
    for phi, (min_params, max_params) in param_ranges.items():
        if min_params <= total_params <= max_params:
            inferred_phi_by_params = phi
            break
    
    if inferred_phi_by_params:
        print(f"✅ 根据参数量推断: phi = '{inferred_phi_by_params}'")
    else:
        print(f"⚠️  参数量 {total_params/1e6:.2f}M 不在预期范围内")
    
    # 方法2: 根据通道数推断
    if head_channels:
        # 典型的 head stem 通道数
        # s: 128 (256*0.5), m: 192 (256*0.75), l: 256 (256*1.0), x: 320 (256*1.25)
        max_head_channel = max(head_channels)
        if max_head_channel <= 140:
            inferred_phi_by_channel = 's'
        elif max_head_channel <= 200:
            inferred_phi_by_channel = 'm'
        elif max_head_channel <= 280:
            inferred_phi_by_channel = 'l'
        else:
            inferred_phi_by_channel = 'x'
        
        print(f"✅ 根据通道数推断: phi = '{inferred_phi_by_channel}'")
    
    # 方法3: 尝试加载不同 phi 值
    print(f"\n" + "="*60)
    print("尝试加载不同 phi 值")
    print("="*60)
    
    from nets.yolo import YoloBody
    
    # 从 classes_path 获取类别数（需要先知道）
    # 这里假设是 1 个类别（vessel），实际应该从 classes_path 读取
    num_classes = 1  # 默认值，实际应该从 ship_classes.txt 读取
    
    compatible_phis = []
    for phi in ['s', 'm', 'l', 'x']:
        try:
            model = YoloBody(num_classes, phi)
            model_dict = model.state_dict()
            
            # 检查键是否匹配
            model_keys = set(model_dict.keys())
            state_dict_keys = set(state_dict.keys())
            
            # 移除 'module.' 前缀（如果使用了 DataParallel）
            state_dict_keys_clean = {k.replace('module.', '') for k in state_dict_keys}
            
            # 检查形状是否匹配
            matched_keys = 0
            shape_mismatches = []
            
            for key in model_keys:
                if key in state_dict_keys_clean:
                    # 找到原始键
                    original_key = key
                    if key not in state_dict_keys:
                        original_key = 'module.' + key
                    
                    if original_key in state_dict_keys:
                        if model_dict[key].shape == state_dict[original_key].shape:
                            matched_keys += 1
                        else:
                            shape_mismatches.append((key, model_dict[key].shape, state_dict[original_key].shape))
            
            match_rate = matched_keys / len(model_keys) if len(model_keys) > 0 else 0
            
            if match_rate > 0.9:  # 90% 以上匹配
                compatible_phis.append((phi, match_rate, len(shape_mismatches)))
                print(f"✅ phi='{phi}': 匹配率 {match_rate*100:.1f}%, 形状不匹配: {len(shape_mismatches)}")
            else:
                print(f"❌ phi='{phi}': 匹配率 {match_rate*100:.1f}%, 形状不匹配: {len(shape_mismatches)}")
                
        except Exception as e:
            print(f"❌ phi='{phi}': 加载失败 - {e}")
    
    # 最终建议
    print(f"\n" + "="*60)
    print("最终建议")
    print("="*60)
    
    if compatible_phis:
        best_phi = max(compatible_phis, key=lambda x: x[1])[0]
        print(f"🎯 推荐使用: phi = '{best_phi}'")
        print(f"\n在 yolo.py 中设置:")
        print(f'    "phi": "{best_phi}",')
    else:
        print("⚠️  无法确定模型结构，请检查:")
        print("   1. 模型文件是否完整")
        print("   2. 类别数量是否正确")
        print("   3. 模型是否与代码版本匹配")
    
    return {
        'total_params': total_params,
        'num_layers': len(layer_names),
        'inferred_phi': inferred_phi_by_params,
        'compatible_phis': compatible_phis
    }


if __name__ == '__main__':
    # 默认检查当前目录的模型文件
    if len(sys.argv) > 1:
        model_path = sys.argv[1]
    else:
        model_path = 'model_data/YOLOX-final.pth'
    
    if not os.path.exists(model_path):
        print(f"❌ 模型文件不存在: {model_path}")
        print(f"\n使用方法:")
        print(f"  python check_model_structure.py <model_path>")
        sys.exit(1)
    
    check_model_structure(model_path)

