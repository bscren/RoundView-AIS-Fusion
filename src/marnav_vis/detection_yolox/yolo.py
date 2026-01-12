import colorsys
import os
import time

import numpy as np
import torch
import torch.nn as nn
from PIL import ImageDraw, ImageFont
from ament_index_python.packages import get_package_share_directory

from .nets.yolo import YoloBody
from .utils.utils import cvtColor, get_classes, preprocess_input, resize_image
from .utils.utils_bbox import decode_outputs, non_max_suppression

from ultralytics import YOLO as UltralyticsYOLO

'''
训练自己的数据集必看注释！
'''
class YOLO(object):
    _defaults = {
        #--------------------------------------------------------------------------#
        #   使用自己训练好的模型进行预测一定要修改model_path和classes_path！
        #   model_path指向logs文件夹下的权值文件，classes_path指向model_data下的txt
        #   如果出现shape不匹配，同时要注意训练时的model_path和classes_path参数的修改
        #--------------------------------------------------------------------------#
        # "model_path"        : 'detection_yolox/model_data/YOLOX-final.pth',
        # "classes_path"      : 'detection_yolox/model_data/ship_classes.txt',
        "model_path"        : None,  # 初始化时动态设置
        "classes_path"      : None,  # 初始化时动态设置
        #---------------------------------------------------------------------#
        #   输入图片的大小，必须为32的倍数。
        #---------------------------------------------------------------------#
        "input_shape"       : [640, 640],
        #---------------------------------------------------------------------#
        #   所使用的YoloX的版本。s、m、l、x
        #---------------------------------------------------------------------#
        "phi"               : 's',
        #---------------------------------------------------------------------#
        #   只有得分大于置信度的预测框会被保留下来
        #---------------------------------------------------------------------#
        "confidence"        : 0.1,
        #---------------------------------------------------------------------#
        #   非极大抑制所用到的nms_iou大小
        #---------------------------------------------------------------------#
        "nms_iou"           : 0.5,
        #---------------------------------------------------------------------#
        #   该变量用于控制是否使用letterbox_image对输入图像进行不失真的resize，
        #   在多次测试后，发现关闭letterbox_image直接resize的效果更好
        #---------------------------------------------------------------------#
        "letterbox_image"   : True,
        #-------------------------------#
        #   是否使用Cuda
        #   没有GPU可以设置成False
        #-------------------------------#
        "cuda"              : True,
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"

    #---------------------------------------------------#
    #   初始化YOLO
    #---------------------------------------------------#
    def __init__(self, yolo_type, **kwargs):
        self.__dict__.update(self._defaults)
        for name, value in kwargs.items():
            setattr(self, name, value)
        # 清理 yolo_type 字符串，去除空格和换行符，并转换为小写
        if yolo_type is None:
            raise ValueError("yolo_type 参数不能为 None")
        self.yolo_type = str(yolo_type).strip().lower()

        # 获取包共享目录
        package_share_dir = get_package_share_directory('marnav_vis')
        model_data_dir = os.path.join(package_share_dir, 'detection_yolox', 'model_data')

        # YOLOX模型
        if self.yolo_type in ['yolox']:
            if self.model_path is None:
                self.model_path = os.path.join(model_data_dir, 'YOLOX-final.pth')
            if self.classes_path is None:
                self.classes_path = os.path.join(model_data_dir, 'ship_classes.txt')
            #   获得种类和先验框的数量
            self.class_names, self.num_classes  = get_classes(self.classes_path)
            #   画框设置不同的颜色
            hsv_tuples = [(x / self.num_classes, 1., 1.) for x in range(self.num_classes)]
            self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
            self.colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), self.colors))
            self.generate()
        # YOLOv11模型
        elif self.yolo_type in ['yolo11m', 'yolov11', 'yolo11', 'yolov11m']:
            model_path = os.path.join(model_data_dir, 'Yolo11m.pt')
            self.model = UltralyticsYOLO(model_path)
        else:
            raise ValueError(f"不支持的 YOLO 模型类型: '{self.yolo_type}' (原始值: '{yolo_type}', 类型: {type(yolo_type)}). 支持的类型: 'yolox', 'yolo11m', 'yolov11', 'yolo11', 'yolov11m'")
    #---------------------------------------------------#
    #   生成模型
    #---------------------------------------------------#
    def generate(self):
        self.net    = YoloBody(self.num_classes, self.phi)
        device      = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.net.load_state_dict(torch.load(self.model_path, map_location=device, weights_only=False))
        self.net    = self.net.eval()

        print('{} model, and classes loaded.'.format(self.model_path))

        if self.cuda:
            self.net = nn.DataParallel(self.net)
            self.net = self.net.cuda()

    #---------------------------------------------------#
    #   检测图片
    #---------------------------------------------------#
    def detect_image(self, image):
        if self.yolo_type in ['yolox']:
            image_shape = np.array(np.shape(image)[0:2])
            #---------------------------------------------------------#
            #   在这里将图像转换成RGB图像，防止灰度图在预测时报错。
            #   代码仅仅支持RGB图像的预测，所有其它类型的图像都会转化成RGB
            #---------------------------------------------------------#
            image       = cvtColor(image)
            #---------------------------------------------------------#
            #   给图像增加灰条，实现不失真的resize
            #   也可以直接resize进行识别
            #---------------------------------------------------------#
            image_data  = resize_image(image, (self.input_shape[1],self.input_shape[0]), self.letterbox_image)
            #---------------------------------------------------------#
            #   添加上batch_size维度
            #---------------------------------------------------------#
            image_data  = np.expand_dims(np.transpose(preprocess_input(np.array(image_data, dtype='float32')), (2, 0, 1)), 0)

            with torch.no_grad():
                images = torch.from_numpy(image_data)
                if self.cuda:
                    images = images.cuda()
                #---------------------------------------------------------#
                #   将图像输入网络当中进行预测！
                #---------------------------------------------------------#
                outputs = self.net(images)
                outputs = decode_outputs(outputs, self.input_shape)
                #---------------------------------------------------------#
                #   将预测框进行堆叠，然后进行非极大抑制
                #---------------------------------------------------------#
                results = non_max_suppression(outputs, self.num_classes, self.input_shape, 
                            image_shape, self.letterbox_image, conf_thres = self.confidence, nms_thres = self.nms_iou)
                                                        
                if results[0] is None: 
                    return []

                top_label   = np.array(results[0][:, 6], dtype = 'int32')
                top_conf    = results[0][:, 4] * results[0][:, 5]
                top_boxes   = results[0][:, :4]
            #---------------------------------------------------------#
            #   设置字体与边框厚度
            #---------------------------------------------------------#

            out = []
            for i, c in list(enumerate(top_label)):
                predicted_class = self.class_names[int(c)]
                box             = top_boxes[i]
                score           = top_conf[i]

                top, left, bottom, right = box

                y1     = max(0, np.floor(top).astype('int32'))
                x1    = max(0, np.floor(left).astype('int32'))
                y2  = min(image.size[1], np.floor(bottom).astype('int32'))
                x2   = min(image.size[0], np.floor(right).astype('int32'))
                
                out.append((x1,y1,x2,y2,predicted_class,torch.from_numpy(np.array(score)).cuda()))
    

            return out
        elif self.yolo_type in ['yolo11m', 'yolov11', 'yolo11', 'yolov11m']:
            # ultralytics YOLO 返回 Results 对象,关闭verbose输出
            results = self.model(image, verbose=False)
            out = []
            # 处理第一个结果（通常只有一张图片）
            if len(results) > 0:
                result = results[0]
                if result.boxes is not None and len(result.boxes) > 0:
                    boxes = result.boxes
                    for i in range(len(boxes)):
                        # 获取边界框坐标 (x1, y1, x2, y2)
                        xyxy = boxes.xyxy[i].cpu().numpy()
                        x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                        # 获取类别名称
                        cls_id = int(boxes.cls[i].cpu().numpy())
                        class_name = result.names[cls_id] if hasattr(result, 'names') else 'vessel'
                        # 获取置信度
                        conf_tensor = boxes.conf[i]
                        conf = float(conf_tensor.cpu().numpy())
                        # 转换为与 yolox 相同的格式: (x1, y1, x2, y2, class_name, conf)
                        # 注意：yolox 返回的 conf 是 torch.Tensor，这里也保持一致
                        # 保持原始设备（如果有GPU则使用GPU，否则使用CPU）
                        device = conf_tensor.device if hasattr(conf_tensor, 'device') else torch.device('cpu')
                        out.append((x1, y1, x2, y2, class_name, torch.tensor(conf, device=device)))

            return out
