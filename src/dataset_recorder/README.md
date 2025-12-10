多相机-AIS-GNSS 数据集录制工具
================================

面向实船/无人船多传感器同步采集的开源工具，整合 AIS、GNSS 与多路相机视频，提供可靠的持续录制、分片 AIS 组帧、批量落盘与基础健康检查。

主要特性
--------
- **多源采集**：串口/TCP/UDP AIS，UDP GNSS，RTSP 多相机。
- **AIS 分片重组**：支持多片段 `!AIVDM` 缓存、排序、拼接再解码。
- **批量存储**：AIS/GNSS CSV 分秒落盘，相机视频按会话分目录保存。
- **配置驱动**：单一 YAML 覆盖通信、存储、容错、日志等。
- **健壮性**：队列容量控制、缺帧/断流告警、可选自动重连。

环境依赖
--------
- Python ≥ 3.8（含 `opencv-python`, `pyais`, `pyserial` 等，按需要安装）
- Linux / Windows（默认配置为 Linux 串口与路径）
- 相机使用 RTSP 拉流，需要网络可达

快速开始
--------
1. 复制模板并修改配置：
   ```bash
   cd /home/tl/RV/src/dataset_recorder
   cp config_template.yaml config.yaml
   # 按设备实际参数编辑 config.yaml
   ```
2. 运行：
   ```bash
   python3 main.py --config config.yaml --log-level INFO
   ```
3. 退出：`Ctrl+C`，程序会停采并输出最终统计。

配置要点（`config.yaml`）
-----------------------
- `devices.ais`：`comm_type` 选择 `serial`/`tcp`/`udp`，并填端口/波特率或 IP/端口。
- `devices.gnss`：UDP 端口、期望频率、质量阈值。
- `devices.cameras`：每路 `id`、`rtsp_url`、`fps`、相机内参，`enabled` 控制启用。
- `storage`：`root_path` 数据根目录，图像格式/质量，采样率，缓存批量写入参数。
- `fault_tolerance`：重连策略、GNSS/AIS 缺失处理、相机断流处理。
- `logging`：控制台/文件日志级别与轮转。

运行流程
--------
1. 加载 & 校验配置。
2. 验证设备连通性。
3. 初始化并创建会话目录：`session_YYYY_MM_DD_HH_MM_SS/`，包含 `gnss/`、`ais/`、`camera/`。
4. 等待首条 GNSS/AIS 数据用于文件名时间戳。
5. 启动录制，定期打印状态；中断后停止并生成报告。

数据输出
--------
- **AIS CSV**：`ais_<UTC>.csv`，字段包含 `timestamp`、`utc_time`、`mmsi`、`lat`、`lon`、`speed`、`course`、`heading`、`message_type`、`raw_message`。
- **GNSS CSV**：`gnss_<UTC>.csv`，包含 UTC 时间、位置、姿态、速度、质量等（见代码字段）。
- **相机**：按相机 `id` 分目录保存视频或帧（取决于 `camera_receiver` 配置）。
- **元数据**：最终报告包含持续时间与统计信息（保存在会话目录下）。

常见问题
--------
- **队列已满**：`AIS数据队列已满` / `GNSS数据队列已满`，提升 `max_queue_size` 或降低采样/写盘间隔。
- **AIS 子类型不解码**：调试日志提示跳过非 1/2/3 类报文属预期。
- **无首条数据**：若 10 秒未收到 GNSS/AIS，系统使用当前 UTC 生成文件名。

开发与调试
----------
- 主入口：`main.py`
- 模块：`ais_receiver.py`（含多片段组帧）、`gnss_receiver.py`、`camera_receiver.py`、`data_storage.py`
- 提升日志：运行时 `--log-level DEBUG`，或在配置中调低 `logging.console_level`。

License
-------
请根据项目实际 License 填写。*** End Patch」}"]}

