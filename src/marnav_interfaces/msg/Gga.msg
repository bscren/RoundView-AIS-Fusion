# 对应 GGA 句子的 ROS 2 消息接口
builtin_interfaces/Time timestamp  # 接收消息的时间戳（ROS 时间）
builtin_interfaces/Time utc_time   # 字段1：UTC时间（hhmmss.ss）
float64 latitude                   # 字段2+3：纬度（度，含南北半球转换）
float64 longitude                  # 字段4+5：经度（度，含东西半球转换）
uint8 quality_indicator            # 字段6：定位质量（0-8，对应枚举值）
float64 hdop                       # 字段7：水平精度因子（米）
float64 altitude                   # 字段8：海拔高度（米）
float64 geoid_separation           # 字段9：大地水准面分离（米）
