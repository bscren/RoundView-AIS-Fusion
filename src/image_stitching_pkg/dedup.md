### JHjpg 去重与绘制联动方案（最终修订）

#### Summary
在拼接节点内对轨迹框先去重，再生成 `JHjpg.message.trajectories`。  
`draw_box_or_not` 仅控制“是否把去重后框叠加到拼接图像上”，不影响 JSON 输出。

#### Key Changes
- 在 [JH_SeamStitcher.cpp](/home/tl/RV/src/image_stitching_pkg/src/JH_SeamStitcher.cpp) 的 `CalibTrajInPano(...)` 固化顺序：
  1. 收集各相机投影后的候选框  
  2. 去重生成最终框集  
  3. `trajectory_boxes_` 仅保存去重结果  
  4. 若 `draw_box_or_not=true`，只绘制去重后的框；若 `false`，不绘制任何框
- 去重判定规则：
  - 两框 `mmsi>0` 且相同：重复候选
  - 至少一方 `mmsi<=0`：仅当 `IoU>=0.70` 才是重复候选
  - 两框 `mmsi>0` 且不同：不合并
- 保留策略：
  - 默认保留更靠中心框（边缘距离分数更高者）
  - 平分时保留面积更大者，再平分时保留有效 `mmsi` 者
- 参数接入：
  - 在 [JH_ros2_stitch_node.cpp](/home/tl/RV/src/image_stitching_pkg/src/JH_ros2_stitch_node.cpp) 读取并传递去重参数
  - 在两份配置同步新增字段并统一默认值：
    - [JH_stitch_config.yaml](/home/tl/RV/src/image_stitching_pkg/config/JH_stitch_config.yaml)
    - [JH_stitch_rosbag_config.yaml](/home/tl/RV/src/image_stitching_pkg/config/JH_stitch_rosbag_config.yaml)
  - 建议默认：
    - `trajectory_dedup_enable: true`
    - `trajectory_dedup_iou_threshold_no_mmsi: 0.70`
    - `trajectory_dedup_keep_policy: "center"`

#### Public Interfaces / Types
- ROS topic 与消息类型不变。  
- `/image_topic_all` 的 `JHjpg.message.trajectories` 始终输出“去重后结果”。  
- `draw_box_or_not` 只影响图像叠框显示，不改变 `trajectories` 内容。  
- `/fus_trajectory_topic` 不做改动。

#### Test Plan
1. `draw_box_or_not=true`：图像显示去重后框，JSON 也是去重后框。  
2. `draw_box_or_not=false`：图像不显示框，JSON 仍输出去重后框。  
3. 同 `mmsi` 跨相机重复：只保留 1 个。  
4. 至少一方 `mmsi<=0` 且 IoU=0.65：不合并。  
5. 至少一方 `mmsi<=0` 且 IoU=0.75：合并。  
6. 不同 `mmsi` 高重叠：不合并。

#### Assumptions
- 当前范围仅处理 `JHjpg` 去重，不改融合节点发布逻辑。  
- 维持当前 3 相机实现。  
- 旧配置缺少新字段时使用默认值，避免启动失败。
