-- Copyright 2023 Amadeusz Szymko
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
        map_builder = MAP_BUILDER,
        trajectory_builder = TRAJECTORY_BUILDER,
        map_frame = "map",
        tracking_frame = "base_link",
        published_frame = "base_link",
        odom_frame = "odom",
        provide_odom_frame = false,
        publish_frame_projected_to_2d = true,
        use_odometry = true,
        use_pose_extrapolator = false,
        use_nav_sat = false,
        use_landmarks = false,
        publish_to_tf = true,
        num_laser_scans = 1,
        num_multi_echo_laser_scans = 0,
        num_subdivisions_per_laser_scan = 1,
        num_point_clouds = 0,
        lookup_transform_timeout_sec = 0.2,  -- ?
        submap_publish_period_sec = 0.3,  -- ?
        pose_publish_period_sec = 5e-3,  -- ?
        trajectory_publish_period_sec = 30e-3,  -- ?
        rangefinder_sampling_ratio = 1.,  -- ?
        odometry_sampling_ratio = 1.,  -- ?
        fixed_frame_pose_sampling_ratio = 1.,  -- ?
        imu_sampling_ratio = 1.,  -- ?
        landmarks_sampling_ratio = 1.,
      }

TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = false
TRAJECTORY_BUILDER.trajectory_builder_2d.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER.trajectory_builder_2d.min_range = 0.2
TRAJECTORY_BUILDER.trajectory_builder_2d.max_range = 25.
TRAJECTORY_BUILDER.trajectory_builder_2d.missing_data_ray_length = 25

TRAJECTORY_BUILDER.trajectory_builder_2d.submaps.num_range_data = 100
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

MAP_BUILDER.use_trajectory_builder_2d = true
-- tuneable:
MAP_BUILDER.pose_graph.optimize_every_n_nodes = 3
-- POSE_GRAPH.global_sampling_ratio =  0.0003 --default=0.003
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.1 --default=0.3

-- -- increase these values when the constraint builder find wrong matches between a new trajectory and an old trajectory
-- POSE_GRAPH.constraint_builder.min_score = 0.3 -- cartographer default 0.55
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.35

return options
