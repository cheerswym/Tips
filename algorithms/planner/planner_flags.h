#ifndef ONBOARD_PLANNER_PLANNER_FLAGS_H_
#define ONBOARD_PLANNER_PLANNER_FLAGS_H_

#include "gflags/gflags.h"

DECLARE_string(planner_name);

DECLARE_int32(planner_thread_pool_size);

DECLARE_double(planner_max_allowed_iteration_time);

DECLARE_double(planner_check_trajectory_engage_condition_duration);

DECLARE_bool(planner_consider_objects);
DECLARE_double(planner_filter_reflected_object_distance);
DECLARE_int32(planner_initializer_debug_level);
DECLARE_bool(planner_initializer_multiple_traj);
DECLARE_bool(planner_initializer_use_refline_search);
DECLARE_int32(planner_lookforward_time_ms);
DECLARE_double(planner_prediction_probability_threshold);
DECLARE_bool(planner_only_use_most_likely_trajectory);
// Offline DataDumping
DECLARE_bool(dumping_initializer_features);
DECLARE_bool(dumping_selector_features);
DECLARE_string(expert_trajectory_file_path);
DECLARE_bool(post_evaluation);

DECLARE_bool(planner_open_door_at_route_end);
DECLARE_double(planner_door_state_override_waiting_time);

// Scene reasoning
DECLARE_bool(planner_enable_occluded_objects_inference);

// Scheduler
DECLARE_int32(planner_est_parallel_branch_num);
DECLARE_bool(planner_est_scheduler_allow_borrow);
DECLARE_bool(planner_save_smooth_result_in_planner_state);
DECLARE_bool(planner_send_lane_graph_to_canvas);

// Decision
DECLARE_bool(planner_consider_all_lanes_virtual);

// Spacetime.
DECLARE_bool(planner_st_traj_mgr_use_all);

// Dopt auto tuning.
DECLARE_bool(auto_tuning_mode);
DECLARE_bool(update_learned_alphas);
DECLARE_bool(update_learned_alphas_except_lane_change);
DECLARE_string(traj_opt_params_file_address);
DECLARE_bool(compare_different_weight);
DECLARE_bool(compare_based_on_original_weight);

// Speed.
DECLARE_double(oncoming_decel);
DECLARE_double(oncoming_human_delay);

// Snapshot. Used for checking prediction state, should be removed later.
DECLARE_string(dump_prediction_output_dir);

DECLARE_int32(planner_drive_passage_debug_level);
DECLARE_bool(est_fallback_upon_traj_validation_failure);
DECLARE_int32(planner_deferred_path_planning_time_ms);
DECLARE_double(planner_path_start_point_time_diff_limit);
DECLARE_bool(enable_path_start_point_look_ahead);

DECLARE_bool(planner_map_patch_enable);

DECLARE_bool(planner_enable_runtime_uturn_task);

// Planner semantic map manager.
DECLARE_double(planner_increase_lane_speed_limit_fraction);

#endif  // ONBOARD_PLANNER_PLANNER_FLAGS_H_
