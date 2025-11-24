# 对应 RMC 句子的 ROS 2 消息接口
builtin_interfaces/Time timestamp      # 接收消息的时间戳（ROS 时间）
builtin_interfaces/Time utc_time       # 字段1：UTC时间（hhmmss.ss）
char status                            # 字段2：状态（A=有效，V=无效）
float64 latitude                       # 字段3+4：纬度（度，含南北半球转换）
float64 longitude                      # 字段5+6：经度（度，含东西半球转换）
float64 sog                            # 字段7：对地速度（节）
float64 heading                        # 字段8：航向（真北，度）
float64 magnetic_variation             # 字段9：磁偏角（度）
char magnetic_hemisphere               # 字段10：磁偏角方向（E/W）
