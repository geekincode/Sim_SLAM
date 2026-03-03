include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",       -- 跟踪帧为底盘
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,          -- Cartographer 自行提供 odom
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,               -- 仿真中如果有 /odom 可设为 true
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,                -- 使用 1 路 LaserScan
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,               -- 不直接使用 PointCloud2
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ====== 2D 轨迹构建器参数 ======
-- 针对仿真环境中 Mid-360 特征少、点云频率低的优化
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.15       -- Mid-360 最小量程
TRAJECTORY_BUILDER_2D.max_range = 40.        -- Mid-360 最大量程
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = false   -- 仿真中暂无 IMU，关闭

-- 开启实时在线相关扫描匹配，解决您 problem.md 中位姿漂移的问题
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e1

-- ====== 位姿图优化参数 ======
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.55   -- 适当降低，增强回环检测

return options