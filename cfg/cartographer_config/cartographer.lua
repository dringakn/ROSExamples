include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",  -- The ROS frame ID to use for publishing submaps, the parent frame of poses, usually “map”.
  tracking_frame = "imu", -- The ROS frame ID of the frame that is tracked by the SLAM algorithm. If an IMU is used, it should be at its position, although it might be rotated. A common choice is "imu".
  published_frame = "odom", -- The ROS frame ID to use as the child frame for publishing poses. For example "odom" if an "odom" frame is supplied by a different part of the system, otherwise base_link
  provide_odom_frame = true, -- if true, cartographer will provide an odom frame (local, non-loop-closed, continuous pose)
  odom_frame = "cartographer_odom", -- Only used if provide_odom_frame is true. 
  publish_frame_projected_to_2d = true,
  use_odometry = true, -- true if odom is available, false otherwise
  use_nav_sat = false, -- false if gps is not available, true otherwise
  use_landmarks = false,
  num_laser_scans = 0, -- set the number of 2D laser scans
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1, -- Set the number of point clouds (3D LIDAR)
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-2,
  trajectory_publish_period_sec = 30e-2,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 10 -- try 2
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7
--POSE_GRAPH.optimize_every_n_nodes = 200
--POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
--POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
--POSE_GRAPH.constraint_builder.min_score = 0.62
--POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

return options
