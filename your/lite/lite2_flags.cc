#include "onboard/lite/lite2_flags.h"

DEFINE_bool(lite2_multiprocess, false, "Use shm based shared_memory_manager");
DEFINE_bool(lite2_multiprocess_output_module_console_log, false,
            "Whether to output console log");
DEFINE_bool(lite2_event_based_control, false,
            "Whether to use event to control module life cycle");

DEFINE_string(lite2_module_name, "AUTONOMY_MODULE", "module name to run");

DEFINE_int64(max_restart_limit, 3, "Maximum number of restarts of the module.");

DEFINE_int64(module_exit_timeout_s, 30,
             "Maximum waiting time for module exit.");
DEFINE_int64(max_module_startup_exit_priority, 10,
             "Maximum number of module startup and shutdown priorities.");

// sensor_live
DEFINE_string(sensor_live_store_folder, "/qcaft/sensor_elements/",
              "The store folder");
DEFINE_int32(sensor_live_image_frame_threshold, 128,
             "The minimum image number for check validity");
DEFINE_int32(sensor_live_liadr_frame_threshold, 128,
             "The minimum lidar number for check validity");
DEFINE_int32(sensor_live_radar_frame_threshold, 128,
             "The minimum radar number for check validity");
DEFINE_bool(sensor_live_ignore_node_module_memory_check, true,
            "Whether ignore node_module memory check.");

// rsim
DEFINE_int64(rsim_clock_offset, 0, "The rsim interval");
DEFINE_int64(rsim_playback_thread_num, 8, "The threads to play");
DEFINE_int64(rsim_message_parse_thread_num, 6, "The threads to parse");
DEFINE_int64(rsim_clock_timepoint, 0, "The same timepoint on node");
DEFINE_string(rsim_scenario, "scenario4", "The scenario to use");
DEFINE_string(rsim_run_dir, "/hosthome/rsim", "The rsim_config_path");
DEFINE_string(rsim_run_success_flag_store_path,
              "/tmp/rsim_run_success_flag_store.txt",
              "The file to store whether run success");
DEFINE_bool(rsim_has_only_one_gpu, false, "Whether has only one gpu.");
DEFINE_string(rsim_state_store_file, "", "The rsim state store file");
DEFINE_string(rsim_testbench_id, "QTB0001", "The testbench index");
DEFINE_string(
    rsim_profiling_lite_modules, "",
    "Comma separated list of module names for CPU profiling on rsim. for "
    "example: PLANNER_MODULE,PERCEPTION_MODULE,POSITIONING_MODULE");

DEFINE_bool(
    rsim_vehicle_engine_publish_imu, true,
    "Publish IMU raw reading synthesized by vehicle engine (with noise). If "
    "false, no IMU raw reading is published and control will not be able to "
    "engage unless GnssImuDriverModule is running or playing back logs.");

DEFINE_bool(cpu_usage_check, true, "Whether to check cpu usage.");
DEFINE_bool(memory_usage_check, true, "Whether to check memory usage.");
DEFINE_bool(shared_memory_usage_check, true,
            "Whether to check shared memory usage.");

DEFINE_int64(channel_check_frame_window, 11,
             "The channel check frequency detection window.");

DEFINE_string(run_conf, "mkz_logless_simulation",
              "Name of the configuration used for this run. Must be set");

DEFINE_string(specified_vehicle_params_path_in_simulation, "",
              "The specified vehicle_params_path in simulation");
DEFINE_string(lite_run_dir, "", "The full path of lite run directory.");

DEFINE_bool(lite_pressure_test, false,
            "Whether to run pressure test on lite system.");

DEFINE_string(birany_path, "/onboard/lite/launch_autonomy_main",
              "the path to executable.");

DEFINE_string(
    q_run_context, "dbqv3",
    "The run conext of binary, must be one of dbqv2/dbqv3/dbqv4/pbqv1, See "
    "QRunConext for correspond type");

DEFINE_string(
    q_run_conext, "",
    "The run conext of binary, must be one of dbqv2/dbqv3/dbqv4/pbqv1, See "
    "QRunConext for correspond type");
