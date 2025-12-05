#!/bin/bash
# DeepSORVF 快速测试脚本

echo "=========================================="
echo "DeepSORVF_ros_v7 快速测试"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查1：ROS环境
echo -e "${YELLOW}[1/5]${NC} 检查ROS环境..."
if [ -f "/home/tl/RV/install/setup.bash" ]; then
    echo -e "${GREEN}✓${NC} 找到ROS安装"
    source /home/tl/RV/install/setup.bash
else
    echo -e "${RED}✗${NC} 未找到ROS安装"
    exit 1
fi

# 检查2：编译状态
echo -e "${YELLOW}[2/5]${NC} 检查编译状态..."
if [ -f "/home/tl/RV/install/marnav_vis/lib/marnav_vis/DeepSORVF_JH" ]; then
    echo -e "${GREEN}✓${NC} DeepSORVF_JH 可执行文件存在"
else
    echo -e "${RED}✗${NC} DeepSORVF_JH 不存在，正在编译..."
    cd /home/tl/RV
    colcon build --packages-select marnav_vis --symlink-install
fi

# 检查3：YOLO模型
echo -e "${YELLOW}[3/5]${NC} 检查YOLO模型..."
YOLO_MODEL="/home/tl/RV/src/marnav_vis/detection_yolox/model_data/yolo_weights.pth"
if [ -f "$YOLO_MODEL" ] || [ -f "/home/tl/RV/src/marnav_vis/weights/yolov3_deep_sort.pth" ]; then
    echo -e "${GREEN}✓${NC} YOLO模型文件存在"
else
    echo -e "${YELLOW}⚠${NC} 未找到YOLO模型（可能使用默认模型）"
fi

# 检查4：测试数据
echo -e "${YELLOW}[4/5]${NC} 检查测试数据..."
if [ -d "/home/tl/RV/src/marnav_vis/clip-01" ]; then
    echo -e "${GREEN}✓${NC} 找到测试数据目录"
    VIDEO_COUNT=$(find /home/tl/RV/src/marnav_vis/clip-01 -name "*.mp4" | wc -l)
    AIS_COUNT=$(find /home/tl/RV/src/marnav_vis/clip-01/ais -name "*.csv" 2>/dev/null | wc -l)
    echo "  视频文件: ${VIDEO_COUNT} 个"
    echo "  AIS文件: ${AIS_COUNT} 个"
else
    echo -e "${YELLOW}⚠${NC} 未找到测试数据目录"
fi

# 检查5：conda环境
echo -e "${YELLOW}[5/5]${NC} 检查conda环境..."
if command -v conda &> /dev/null; then
    if conda env list | grep -q "yolov11"; then
        echo -e "${GREEN}✓${NC} yolov11 conda环境存在"
    else
        echo -e "${YELLOW}⚠${NC} yolov11 conda环境不存在"
    fi
else
    echo -e "${YELLOW}⚠${NC} conda未安装"
fi

echo ""
echo "=========================================="
echo "检查完成！"
echo "=========================================="
echo ""

# 提供测试选项
echo "请选择测试方式："
echo ""
echo "1) 运行完整系统（launch文件 - 推荐）"
echo "2) 只运行主节点（需要手动启动其他节点）"
echo "3) 运行诊断脚本（模拟测试，不需要ROS）"
echo "4) 查看代码检查报告"
echo "5) 退出"
echo ""
read -p "请输入选项 (1-5): " choice

case $choice in
    1)
        echo ""
        echo -e "${GREEN}启动完整系统...${NC}"
        echo "提示：观察终端输出，寻找以下关键信息："
        echo "  - [VIS DEBUG] 检测到 X 个目标"
        echo "  - [FUSION DEBUG] 成功融合: [ID]"
        echo "  - ⚠️ 警告信息"
        echo ""
        echo "按 Ctrl+C 停止程序"
        echo ""
        sleep 2
        cd /home/tl/RV
        ros2 launch marnav_vis Assemble_JH_launch.py
        ;;
    2)
        echo ""
        echo -e "${GREEN}启动主节点...${NC}"
        echo "注意：您需要手动启动以下节点："
        echo "  - camera_pub_temporary_Test_node"
        echo "  - gnss_pub_node"
        echo "  - ais_csv_pub_node"
        echo "  - ais_sorted_pub_node"
        echo ""
        sleep 2
        cd /home/tl/RV
        ros2 run marnav_vis DeepSORVF_JH
        ;;
    3)
        echo ""
        echo -e "${GREEN}运行诊断脚本...${NC}"
        echo ""
        echo "=== 成功案例 ==="
        conda run -n yolov11 python3 /home/tl/RV/融合诊断.py
        echo ""
        echo "=== 失败案例 ==="
        conda run -n yolov11 python3 /home/tl/RV/融合诊断_失败案例.py
        ;;
    4)
        echo ""
        if command -v less &> /dev/null; then
            less /home/tl/RV/代码检查报告.md
        elif command -v cat &> /dev/null; then
            cat /home/tl/RV/代码检查报告.md
        else
            echo "请手动查看：/home/tl/RV/代码检查报告.md"
        fi
        ;;
    5)
        echo "退出"
        exit 0
        ;;
    *)
        echo -e "${RED}无效选项${NC}"
        exit 1
        ;;
esac

