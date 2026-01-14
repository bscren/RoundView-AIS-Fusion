# 单线程的逐帧处理，适用于ROS 2环境下的快速原型演示
import os
import time
import imutils
import cv2
import pandas as pd
import numpy as np
import rclpy
from rclpy.node import Node
from utils_backup.file_read import read_all, ais_initial, update_time, time2stamp
from utils_backup.VIS_utils import VISPRO
from utils_backup.AIS_utils import AISPRO
from utils_backup.FUS_utils import FUSPRO
from utils_backup.gen_result import gen_result
from utils_backup.draw import DRAW


class AisVisNode(Node):
    def __init__(self):
        super().__init__('ais_vis_node')  # ROS 2节点名称
        
        # ========================== 1. 声明ROS 2参数（替代argparse）==========================
        self.declare_parameters(
            namespace='',
            parameters=[
                ('data_path', '/home/tl/RV/src/marnav_vis/clip-01/'),         # 数据路径
                ('result_path', './result/'),       # 结果保存路径
                ('anti', 1),                        # 抗遮挡开关
                ('anti_rate', 0)                    # 遮挡率
            ]
        )
        
        # ========================== 2. 获取参数值 ==========================
        self.data_path = self.get_parameter('data_path').value
        self.result_path = self.get_parameter('result_path').value
        self.anti = self.get_parameter('anti').value
        self.anti_rate = self.get_parameter('anti_rate').value
        
        # 从data_path和result_path解析衍生参数（原read_all逻辑）
        self.video_path, self.ais_path, self.result_video, self.result_metric, self.initial_time, self.camera_para = read_all(
            self.data_path, self.result_path
        )
        
        # 打印参数配置（类似原代码的打印逻辑）
        self.get_logger().info("\nVesselSORT")
        self.get_logger().info(f"\tdata_path: {self.data_path}")
        self.get_logger().info(f"\tresult_path: {self.result_path}")
        self.get_logger().info(f"\t软链接的 video_path: {self.video_path}")
        self.get_logger().info(f"\t软链接的 ais_path: {self.ais_path}")
        self.get_logger().info(f"\tanti: {self.anti}")
        self.get_logger().info(f"\tanti_rate: {self.anti_rate}\n")
        
        # ========================== 3. 初始化资源（原main函数初始化逻辑）==========================
        self.initialize_resources()


    def initialize_resources(self):
        """初始化视频捕获、AIS/VIS/FUS处理器等资源"""
        # AIS初始化
        self.ais_file, self.timestamp0, self.time0 = ais_initial(self.ais_path, self.initial_time)
        self.Time = self.initial_time.copy()
        
        # 视频捕获
        self.cap = cv2.VideoCapture(self.video_path)
        self.im_shape = [self.cap.get(3), self.cap.get(4)]  # 图像尺寸
        self.max_dis = min(self.im_shape) // 2
        self.fps = int(self.cap.get(5)) # 视频帧率 25fps
        self.t = int(1000 / self.fps)  # 每帧持续时间（40毫秒）
        
        # 处理器初始化
        self.AIS = AISPRO(self.ais_path, self.ais_file, self.im_shape, self.t)
        self.VIS = VISPRO(self.anti, self.anti_rate, self.t)
        self.FUS = FUSPRO(self.max_dis, self.im_shape, self.t)
        self.DRA = DRAW(self.im_shape, self.t)
        
        # 视频显示与保存配置
        self.name = 'ROS version demo'
        self.show_size = 500
        self.videoWriter = None
        self.bin_inf = pd.DataFrame(columns=['ID', 'mmsi', 'timestamp', 'match'])
        
        # 计时与计数变量
        self.times = 0
        self.time_i = 0
        self.sum_t = []
        
        # 打印启动信息
        self.get_logger().info(
            f'Start Time: {self.time0} || Stamp: {self.timestamp0} || fps: {self.fps}'
        )


    def run(self):
        """主循环：处理视频帧与数据融合"""
        while rclpy.ok():  # 兼容ROS 2退出信号
            # 逐帧读取视频
            ret, im = self.cap.read()

            # print im_shape
            # print(f"FUSION DEBUG: Current image shape: {im.shape}")
            if not ret:  # 视频读取完毕
                self.get_logger().info("Video processing completed.")
                break
            
            start = time.time()
            
            # 更新时间戳
            self.Time, timestamp, Time_name = update_time(self.Time, self.t)
            
            # 1. AIS轨迹提取
            time_start_ais = time.time()
            AIS_vis, AIS_cur = self.AIS.process(self.camera_para, timestamp, Time_name)
            time_end_ais = time.time()
            

            # 2. 视觉轨迹提取
            time_start_vis = time.time()
            Vis_tra, Vis_cur = self.VIS.feedCap(im, timestamp, AIS_vis, self.bin_inf)
            time_end_vis = time.time()

            # 3. 视觉与AIS数据融合
            time_start_fus = time.time()
            Fus_tra, self.bin_inf = self.FUS.fusion(AIS_vis, AIS_cur, Vis_tra, Vis_cur, timestamp)
            time_end_fus = time.time()
            

             # ============ 比对调试 ============
            """将AIS_vis和AIS_cur保存到本地CSV文件"""
            # AIS_cur  = pd.DataFrame(columns=['mmsi','lon','lat','speed','course','heading','type','timestamp'])
            # AIS_vis  = pd.DataFrame(columns=['mmsi','lon','lat','speed','course','heading','type','x','y','timestamp'])
            # if timestamp % 1000 < self.t:
            #     result_name_vis = f'result/Standard_AIS_vis/AIS_vis_{timestamp}.csv'
            #     result_name_cur = f'result/Standard_AIS_cur/AIS_cur_{timestamp}.csv'
            #     result_name_tra_cur = f'result/Standard_Vis_tra_cur/Vis_tra_cur_{timestamp}.csv'
            #     result_name_tra = f'result/Standard_Vis_tra/Vis_tra_{timestamp}.csv'
            #     result_name_fus_tra = f'result/Standard_Fus_tra/Fus_tra_{timestamp}.csv'
            #     AIS_vis.to_csv(result_name_vis, index=False)
            #     AIS_cur.to_csv(result_name_cur, index=False)
            #     Vis_tra.to_csv(result_name_tra, index=False)
            #     Vis_cur.to_csv(result_name_tra_cur, index=False)
            #     Fus_tra.to_csv(result_name_fus_tra, index=False)
            # ==============================
            
            
            
            # 4. 融合结果绘制
            time_start_draw = time.time()
            im = self.DRA.draw_traj(im, AIS_vis, AIS_cur, Vis_tra, Vis_cur, Fus_tra, timestamp)
            time_end_draw = time.time()
            
            # 计时与结果生成
            end = time.time() - start
            
            self.time_i += end
            if timestamp % 1000 < self.t:
                gen_result(self.times, Vis_cur, Fus_tra, self.result_metric, self.im_shape)
                self.times += 1
                self.sum_t.append(self.time_i)

                self.get_logger().info(
                    f'Time: {Time_name} || Stamp: {timestamp} || '
                    f'AIS : {time_end_ais - time_start_ais:.6f} seconds. || '
                    f'VIS : {time_end_vis - time_start_vis:.6f} seconds. || '
                    f'FUS : {time_end_fus - time_start_fus:.6f} seconds. || '
                    f'DRAW: {time_end_draw - time_start_draw:.6f} seconds. || '
                    f'Total Process: {end:.6f} || '
                    f'Average: {np.mean(self.sum_t):.6f} +- {np.std(self.sum_t):.6f}'
                )
                self.time_i = 0

            # 视频显示与保存
            result = imutils.resize(im, height=self.show_size)
            if self.videoWriter is None:
                fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
                self.videoWriter = cv2.VideoWriter(
                    self.result_video, fourcc, self.fps, (result.shape[1], result.shape[0])
                )
            self.videoWriter.write(result)
            
            cv2.imshow(self.name, result)
            cv2.waitKey(1)
            
            # 检查窗口是否关闭（用户点击x）
            if cv2.getWindowProperty(self.name, cv2.WND_PROP_AUTOSIZE) < 1:
                self.get_logger().info("User closed the window.")
                break
            
            # 处理ROS 2事件（如节点关闭信号）
            rclpy.spin_once(self, timeout_sec=0.001)
        
        # 资源释放
        self.cap.release()
        if self.videoWriter:
            self.videoWriter.release()
        cv2.destroyAllWindows()


def main(args=None):
    # 初始化ROS 2
    rclpy.init(args=args)
    
    # 创建节点并运行
    node = AisVisNode()
    node.run()
    
    # 清理资源
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
