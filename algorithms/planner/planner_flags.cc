#include "onboard/planner/planner_flags.h"

#include <limits>

DEFINE_string(
    planner_name, "est_planner",
    "The planner to use. It can be selected from dopt_planner, est_planner.");

DEFINE_int32(planner_thread_pool_size, 3, "Planner thread pool size.");

DEFINE_double(planner_max_allowed_iteration_time, 0.35,
              "The maximum allowed planner iteration time in seconds.");

DEFINE_bool(
    planner_consider_objects, true,
    "Planner consider objects in perception and prediction. This is "
    "useful for testing mode that does not have perception or prediction.");

DEFINE_double(
    planner_check_trajectory_engage_condition_duration, 1.0,
    "Validate if we can engage in the trajectory's first this amount of time.");

DEFINE_double(
    planner_filter_reflected_object_distance, -0.001,
    "Filter reflected objects that in AV's current position's proximity and "
    "the distance to SDC is less than this value (in meters). Reflected "
    "objects are fake unknown stationary perception objects "
    "that are near SDC body. When this value is less than zero, we "
    "will not filter reflected object.");

DEFINE_int32(planner_lookforward_time_ms, 200,
             "The planner look forward time when computing trajectory.");

DEFINE_double(planner_prediction_probability_threshold, 0.1,
              "The threshold on probability below which a predicted behavior / "
              "trajectory will be ignored (use at your risk)");

DEFINE_bool(planner_only_use_most_likely_trajectory, false,
            "Only use the most likely trajectory.");

DEFINE_bool(planner_open_door_at_route_end, false,
            "Open door when AV is at end of route and fully stopped.");

DEFINE_double(
    planner_door_state_override_waiting_time, 40.0,
    "Don't change door state for so many seconds after door state override. "
    "This value should be less than the duration fro the AV to travel from one "
    "station to another. This is used to prevent software open door "
    "immediately after driver closed the door.");

DEFINE_bool(planner_enable_occluded_objects_inference, false,
            "Enable reasoning occluded objects according sensor fov.");

// Scheduler.
DEFINE_int32(
    planner_est_parallel_branch_num, 2,
    "The maximal amount of target lane paths to be generated for multi-task "
    "est planner. Note that a borrow-lane branch will be created if this flag "
    "is set to 1, so if only one branch is desired, you should also set the "
    "next flag to false to prohibit lane borrowing.");
DEFINE_bool(planner_est_scheduler_allow_borrow, true,
            "Whether to allow a borrow path boundary if only one target lane "
            "is chosen.");

DEFINE_bool(planner_send_lane_graph_to_canvas, false,
            "Whether to send lane graph to canvas.");

DEFINE_bool(planner_save_smooth_result_in_planner_state, true,
            "Whether calculate smoothed ref line ahead and save the results in "
            "planner state");

DEFINE_int32(
    planner_initializer_debug_level, 0,
    "Initializer debug level: 0:Release, 1:GeometryGraph, 2:MotionSearch");

DEFINE_bool(planner_initializer_use_refline_search, true,
            "Initializer reference line searcher");

DEFINE_bool(
    dumping_initializer_features, false,
    "Whether reading and dumping initializer features' cost of manual driving "
    "trajectory for offline learning");

DEFINE_bool(
    dumping_selector_features, false,
    "Whether reading and dumping selector features' cost of manual driving "
    "trajectory for offline learning");

DEFINE_string(expert_trajectory_file_path, "",
              "Local file storing corresponding manual driving trajectory");

DEFINE_bool(planner_initializer_multiple_traj, true,
            "default single trajectory output");

DEFINE_bool(post_evaluation, true,
            "Whether evaluating topK trajectory whether a new evaluation "
            "function and select the best out of the topK");

DEFINE_bool(planner_consider_all_lanes_virtual, false,
            "Whether to consider all lanes\' type as VIRTUAL, for scenarios "
            "with one single lane for mixed use.");

// Spacetime flags.
DEFINE_bool(planner_st_traj_mgr_use_all, false,
            "Spacetime trajectory manager uses all objects and trajectories.");

// Dopt auto tuning.
DEFINE_bool(auto_tuning_mode, false,
            "When auto tuning mode is on, the optimizer will generate and save "
            "one more output which are the accumulated discounted costs for "
            "different cost type.");
DEFINE_bool(
    update_learned_alphas, false,
    "Whether to use the cost weight alphas learned in auto tuning mode.");
DEFINE_bool(
    update_learned_alphas_except_lane_change, true,
    "Whether to use the cost weight alphas learned in auto tuning mode when "
    "lane change. Only worked when update_learned_alphas is true. So this will "
    "not influence training process but will influence "
    "evaluation/validation/testing.");
DEFINE_string(
    traj_opt_params_file_address,
    "offboard/planner/ml/params_tuning/dopt_auto_tuning/traj_opt_params.pb.txt",
    "The address of the trajectory optimizer params proto file.");
DEFINE_bool(
    compare_different_weight, false,
    "Whether to compare different cost weight learned in auto tuning mode.");
DEFINE_bool(compare_based_on_original_weight, true,
            "Whether to compare the cost weight based on original cost "
            "weight(true) or auto tuned cost weight(false), only works when "
            "compare_different_weight is true.");

// Speed flags.
DEFINE_double(oncoming_decel, -2.5,
              "The assumed deceleration of oncoming object.");
DEFINE_double(oncoming_human_delay, 0.5,
              "The assumed human delay of oncoming driver.");

// Used for checking prediction state, should be removed later.
DEFINE_string(
    dump_prediction_output_dir, "",
    "The dump directory(RW), Only used for debug snapshot in off-board mode.");

DEFINE_int32(planner_drive_passage_debug_level, 0,
             "Debug level for drive passage: 0: no debug info; "
             "1: send to canvas for viz; 2: some other options)");
DEFINE_bool(est_fallback_upon_traj_validation_failure, true,
            "Whether to fall back when the EstPlanner trajectory fails "
            "validation checks.");
DEFINE_int32(planner_deferred_path_planning_time_ms, 0,
             "start point of path planing follows that of speed planning"
             "with a delta_t, in order to reduce the mutual effect"
             "of accelerator/brake controlling and steering wheel controlling");
DEFINE_double(planner_path_start_point_time_diff_limit, 0.5,
              "If relative time of closest point on prev traj from plan "
              "start point larger than this time, set path plan start point to "
              "current close point.");
DEFINE_bool(
    enable_path_start_point_look_ahead, false,
    "Whether to use logic about planner_path_start_point_time_diff_limit.");

// Map patch
DEFINE_bool(planner_map_patch_enable, true, "Enable semantic map patch");

DEFINE_bool(planner_enable_runtime_uturn_task, false,
            "Whether to create uturn task at run time.");

// Planner semantic map manager.
DEFINE_double(planner_increase_lane_speed_limit_fraction, 0.1,
              "Increase lane speed limit by a fraction.");
