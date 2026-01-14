# 该脚本用于导出 Ultralytics YOLO 系列模型（如 YOLOv11n）为 ONNX 格式
# 并测试 ONNX 模型与 PyTorch 模型的输出误差
from ultralytics import YOLO
import os
import os.path as osp
import cv2  # 用于图片读写/显示（可选，也可只用YOLO自带方法）
import torch  # 新增：导入torch检测CUDA
from ament_index_python.packages import get_package_share_directory
import os
# ===================== 基本环境检查 =====================
print(torch.cuda.is_available())  # 输出True才表示PyTorch支持CUDA
print(torch.__version__)  # 查看PyTorch版本
# ===================== 配置项（按需修改） =====================
# 获取包共享目录
package_share_dir = get_package_share_directory('marnav_vis')
# 模型路径和ONNX输出路径
model_path = os.path.join(package_share_dir, 'detection_yolox', 'model_data', 'Yolo11m.pt')
target_onnx_path = os.path.join(package_share_dir, 'detection_yolox', 'model_data', 'Yolo11m.onnx')
# model_path = "/home/tl/RV/src/marnav_vis/detection_yolox/model_data/Yolo11m.pt"
# target_onnx_path = "/home/tl/RV/src/marnav_vis/detection_yolox/model_data/Yolo11m.onnx"
input_size = 640
inference_img_path = os.path.join(package_share_dir, 'detection_yolox', 'test', 'test.jpg')
output_img_path = os.path.join(package_share_dir, 'detection_yolox', 'test', 'test_with_box.jpg')
# inference_img_path = "/home/tl/RV/src/marnav_vis/detection_yolox/test/test.jpg"  # 你的输入图片路径
# output_img_path = "/home/tl/RV/src/marnav_vis/detection_yolox/test/test_with_box.jpg"  # 绘制框后保存的图片路径
# ==============================================================

# 新增：自动检测CUDA是否可用，避免手动设置出错
def get_available_device():
    if torch.cuda.is_available():
        print(f"✅ CUDA可用，使用GPU (device=0)")
        return 0  # 有CUDA则用第0块GPU
    else:
        print(f"❌ CUDA不可用，自动切换为CPU")
        return "cpu"  # 无CUDA则用CPU

device = get_available_device()  # 自动获取可用设备
# ==============================================================

# 1. 加载YOLO模型
print(f"正在从 {model_path} 加载模型...")
model = YOLO(model_path)
print(f"模型类别数: {model.model.nc}")

# 2. 导出ONNX（保持原有逻辑）
"""
参数	    类型	    默认值	    描述
simplify	bool	    True	使用以下方式简化模型图 onnxslim，从而可能提高性能和兼容性。
opset	    int	        None	指定 ONNX opset 版本，以便与不同的 ONNX 解析器和运行时兼容。 如果未设置，则使用最新支持的版本。
nms	        bool	    False	添加非极大值抑制 (NMS)，这对于准确高效的检测后处理至关重要。
imgsz	    int或tuple  640	    模型输入的所需图像大小。 可以是正方形图像的整数或元组 (height, width) 用于指定特定维度。
half	    bool	    False	启用 FP16（半精度）量化，从而减小模型大小并可能加快受支持硬件上的推理速度。
format	    str	        'onnx'	导出模型的目标格式，定义与各种部署环境的兼容性。
dynamic	    bool	    False	允许动态输入大小，从而增强了处理不同图像尺寸的灵活性。
device	    str	        None	指定导出设备：GPU (device=0），CPU（device=cpu），适用于 Apple 芯片的 MPS（device=mps）。
batch	    int	        1	    指定导出模型批处理推理大小或导出模型将并发处理的最大图像数量，在 predict 模式下。
"""
print(f"正在导出ONNX模型...")
exported_onnx_path = model.export(
    format="onnx",
    imgsz=input_size,
    opset=19,
    simplify=True,
    nms = True,
    dynamic = True,
    device = 0,
    batch = 16
)

# 3. 重命名ONNX文件（保持原有逻辑）
print(f"原始导出路径: {exported_onnx_path}")
print(f"目标路径: {target_onnx_path}")

if osp.exists(exported_onnx_path):
    # 关键修复：等待2秒，确保文件已完全写入磁盘
    import time
    time.sleep(2) 
    
    # 规范化路径（解决路径分隔符差异问题）
    exported_onnx_path_norm = osp.normpath(exported_onnx_path)
    target_onnx_path_norm = osp.normpath(target_onnx_path)
    
    # 检查两个路径是否相同
    if exported_onnx_path_norm == target_onnx_path_norm:
        print(f"✅ 导出路径与目标路径相同，无需重命名")
        print(f"✅ ONNX文件已保存至: {target_onnx_path}")
    else:
        # 如果目标文件已存在，先删除
        if osp.exists(target_onnx_path):
            os.remove(target_onnx_path)
        # 执行重命名
        os.rename(exported_onnx_path, target_onnx_path)
        print(f"✅ ONNX文件已重命名为: {target_onnx_path}")
else:
    raise FileNotFoundError(f"ONNX导出失败！未找到原始导出文件: {exported_onnx_path}")

# 4. 加载ONNX模型并推理
print(f"正在加载ONNX模型: {target_onnx_path}")
onnx_model = YOLO(target_onnx_path)

if not osp.exists(inference_img_path):
    raise FileNotFoundError(f"推理图片不存在！路径: {inference_img_path}")

print(f"正在对 {inference_img_path} 执行推理...")
results = onnx_model.predict(
    source=inference_img_path,
    imgsz=input_size,
    # device="cpu",
    device = device, # gpu 
    verbose=False
)

# 5. 绘制检测框并保存/显示图片
print("\n推理完成，开始绘制检测框...")
for result in results:
    # ========== 核心：用YOLO自带方法生成带框的图片 ==========
    plotted_img = result.plot()  # 返回BGR格式的numpy数组（适配OpenCV）
    
    # ========== 保存绘制后的图片 ==========
    cv2.imwrite(output_img_path, plotted_img)
    print(f"带检测框的图片已保存至: {osp.abspath(output_img_path)}")
    
    # ========== （可选）显示图片（需有图形界面） ==========
    # cv2.imshow("YOLO Detection Result", plotted_img)
    # cv2.waitKey(0)  # 按任意键关闭窗口
    # cv2.destroyAllWindows()

# 6. 打印推理结果（保持原有逻辑）
print("\n检测结果详情：")
for result in results:
    if result.boxes is None or len(result.boxes) == 0:
        print("未检测到任何目标")
        continue
    for box in result.boxes:
        cls_name = result.names[int(box.cls)]
        conf = box.conf.item()
        xyxy = box.xyxy.tolist()[0]
        print(f"类别: {cls_name:10s} | 置信度: {conf:.4f} | 坐标(xyxy): {[round(x, 2) for x in xyxy]}")