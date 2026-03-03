ros2 service type /octomap_server/clear_bbox
查看服务类型

ros2 interface show octomap_msgs/srv/BoundingBoxQuery
查看接口定义

```
rm@rm-NUC10i7FNH:~/sim2real/test4_ws$ ros2 param list /octomap_server
  base_frame_id
  color.a
  color.b
  color.g
  color.r
  color_factor
  color_free.a
  color_free.b
  color_free.g
  color_free.r
  colored_map
  compress_map
  filter_ground_plane
  filter_speckles
  frame_id
  ground_filter.angle
  ground_filter.distance
  ground_filter.plane_distance
  incremental_2D_projection
  latch
  max_depth
  min_x_size
  min_y_size
  occupancy_max_z
  occupancy_min_z
  octomap_path
  point_cloud_max_x
  point_cloud_max_y
  point_cloud_max_z
  point_cloud_min_x
  point_cloud_min_y
  point_cloud_min_z
  publish_free_space
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  resolution
  sensor_model.hit
  sensor_model.max
  sensor_model.max_range
  sensor_model.min
  sensor_model.miss
  use_height_map
  use_sim_time
```




通用 / OctomapServer（主节点）

frame_id — "map"
地图的参考框架（world frame）。
base_frame_id — "base_footprint"
机器人 base frame（地面滤波时使用）。
use_height_map — false
是否发布基于高度的颜色映射。
colored_map (或 use_colored_map) — false
是否使用彩色 octomap（需编译支持）。
color_factor — 0.8
point_cloud_min_x — -inf
point_cloud_max_x — +inf
point_cloud_min_y — -inf
point_cloud_max_y — +inf
point_cloud_min_z — -100.0
最小高度（插入点云时过滤）。
point_cloud_max_z — 100.0
最大高度（插入点云时过滤）。
occupancy_min_z — -100.0
投影到 2D 占据图时考虑的最小高度。
occupancy_max_z — 100.0
min_x_size — 0.0
min_y_size — 0.0
filter_speckles — false
是否过滤孤立占据点（speckles）。
filter_ground_plane — false
是否进行地面分割（RANSAC）。
ground_filter.distance — 0.04
RANSAC 判定为地面的距离阈值。
ground_filter.angle — 0.15
平面角度阈值（从水平的偏差）。
ground_filter.plane_distance — 0.07
平面离 z=0 的距离阈值（排除桌面等）。
sensor_model.max_range — -1.0
传感器最大整合范围（-1 表示不限制）。
resolution — 0.05
OctoMap 分辨率（m/voxel）。
sensor_model.hit — 0.7
概率模型：命中概率。
sensor_model.miss — 0.4
概率模型：未命中概率。
sensor_model.min — 0.12
概率下限（clamping）。
sensor_model.max — 0.97
概率上限（clamping）。
compress_map — true
是否压缩 octree（prune）。
incremental_2D_projection — false
是否增量投影 2D 地图。
max_depth — (tree depth)
用于 marker 发布的最大遍历深度（1..16，可动态设置）。
color.r, color.g, color.b, color.a — 0.0/0.0/1.0/1.0（默认蓝色）
占据点 marker 颜色。
color_free.r/g/b/a — 默认设置（free cell 颜色）。
publish_free_space — false
latch — true
发布是否为 latched（决定 QoS：transient_local）。
octomap_path — ""
节点启动时尝试从该路径读取 .bt/.ot（注意：它不会在退出时自动写入该文件，参见之前说明）。
(还有用于 2D 多层及颜色的其它参数，如 project_complete_map_、use_colored_map_ 等)
Tracking 特有（tracking_octomap_server）

topic_changes — "changes"
change-set 发布/订阅的话题名（launch 示例替换为 /octomap_tracking_server/changeset 有的）。
change_id_frame — "talker/changes"
change-set 消息 header.frame_id。
track_changes — false
如果 true，节点在插入后会发布变化点云（作为 server/talker）。
listen_changes — false
如果 true，节点订阅并应用来自其它服务器的变化点云（client/listener）。
min_change_pub — 0
只有当变化数量 > min_change_pub 时才发布 change-set。
Saver / Static（保存与静态服务）

在 octomap_saver：
full — false （是否请求 full map）
octomap_path — ""（保存时写入此路径）
在 OctomapServerStatic：
frame_id — "map"
octomap_path — ""（启动时尝试加载该路径的地图）
