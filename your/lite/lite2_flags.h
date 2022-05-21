#ifndef ONBOARD_LITE_LITE2_FLAGS_H_
#define ONBOARD_LITE_LITE2_FLAGS_H_

#include "gflags/gflags.h"

// run module in process and use shm based hub
DECLARE_bool(lite2_multiprocess);

// using autonomy_module to manage other module
DECLARE_bool(lite2_event_based_control);

DECLARE_bool(lite2_multiprocess_output_module_console_log);

DECLARE_string(lite2_module_name);

DECLARE_int64(max_restart_limit);

DECLARE_int64(module_exit_timeout_s);

DECLARE_int64(max_module_startup_exit_priority);

DECLARE_bool(cpu_usage_check);
DECLARE_bool(memory_usage_check);
DECLARE_bool(shared_memory_usage_check);

DECLARE_int64(channel_check_frame_window);

DECLARE_string(run_conf);

DECLARE_string(specified_vehicle_params_path_in_simulation);
DECLARE_string(lite_run_dir);
DECLARE_string(birany_path);

// sensor live
DECLARE_bool(sensor_live_ignore_node_module_memory_check);
DECLARE_string(sensor_live_store_folder);
DECLARE_int32(sensor_live_image_frame_threshold);
DECLARE_int32(sensor_live_liadr_frame_threshold);
DECLARE_int32(sensor_live_radar_frame_threshold);

// rsim
DECLARE_int64(rsim_clock_offset);
DECLARE_int64(rsim_playback_thread_num);
DECLARE_int64(rsim_message_parse_thread_num);
DECLARE_string(rsim_scenario);
DECLARE_string(rsim_testbench_id);
DECLARE_int64(rsim_clock_timepoint);
DECLARE_string(rsim_run_dir);
DECLARE_string(rsim_run_success_flag_store_path);
DECLARE_bool(rsim_vehicle_engine_publish_imu);
DECLARE_bool(rsim_has_only_one_gpu);
DECLARE_string(rsim_state_store_file);
DECLARE_string(rsim_profiling_lite_modules);

// lite pressure test
DECLARE_bool(lite_pressure_test);

DECLARE_string(q_run_context);
DECLARE_string(q_run_conext);  // deprecated

#endif  // ONBOARD_LITE_LITE2_FLAGS_H_
