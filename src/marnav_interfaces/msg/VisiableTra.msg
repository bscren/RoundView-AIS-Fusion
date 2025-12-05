string camera_name # 对应相机的名称
builtin_interfaces/Time timestamp # 时间戳
uint8 ais # 1 为包含视觉信息的AIS信息，0 为只有视觉的检测框信息
uint32 mmsi # 船只的nmsi号
float32 sog # 船只的速度
float32 cog # 船只的航向
float32 lat # 船只的纬度
float32 lon # 船只的经度
float32 box_x1 # 船只检测框的左上角x坐标
float32 box_y1 # 船只检测框的左上角y坐标
float32 box_x2 # 船只检测框的右下角x坐标
float32 box_y2 # 船只检测框的右下角y坐标
