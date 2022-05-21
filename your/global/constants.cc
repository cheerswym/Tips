#include "onboard/global/constants.h"

constexpr char kRunNamePattern[] = "[0-9]{8}_[0-9]{6}_QT?B?[0-9]{4}T?";
constexpr char kRunInfoBinaryFileName[] = "lite_run_info.pb.bin";
constexpr char kImportRunsFileName[] = "import_runs.pb.txt";
constexpr char kLabelFileName[] = "labels.stf";
constexpr char kDefualtLabelStfDir[] = "/media/s3/run_data/stf/";
constexpr char kSecondaryDefualtLabelStfDir[] = "/media/s3/run_data_2/stf/";
constexpr char kSystemInfoProtoLog[] = "system_info_proto.pb.txt";
constexpr char kLiteMsgFileName[] = "lite_msg.stf";
constexpr char kLidarDataFileName[] = "lidar_data.stf";
constexpr char kLidarDataFileNameOld[] = "lidar_spin_hesai_64.stf";
constexpr char kCameraJpgImgFileName[] = "camera_jpg_img.stf";
constexpr char kCameraJpgImgLowResolutionFileName[] =
    "camera_jpg_img_low_resolution.stf";
constexpr char kSystemCheckLogsFileName[] = "system_check.log";
constexpr char kRsyncFinishFlagFileName[] = "zzz_rsync_finish_flag";

constexpr char kOnboardUploadSnippetMetaRelPath[] =
    "onboard_upload_snippet_meta";
constexpr char kLiteMsgSimpleFileName[] = "lite";
constexpr char kLidarDataSimpleFileName[] = "lidar";
constexpr char kCameraJpgImgSimpleFileName[] = "camera";
constexpr char kCameraJpgImgLowResolutionSimpleFileName[] = "camera_low";
constexpr char kRunFilesHintFileName[] = "run_files_hint.pb.txt";
constexpr char kCoreDumpSimpleFileName[] = "core";
constexpr char kMapSimpleFileName[] = "map";

constexpr char kPoseFileName[] = "poses.stf";
constexpr char kLegacyMapFileName[] = "legacy_map_version.txt";
constexpr char kDefaultPoseStfDir[] = "/media/s3/labeling_data/positioning_gt";
constexpr char kDefaultMapName[] = "qcraft-maps";
constexpr char kDefaultChinaMapName[] = "qcraft-maps-china";
constexpr char kDefaultChinaMapFileName[] = "qcraft-maps-china.tar.gz";
constexpr char kRunReleaseChinaMapDirSymlink[] = "/tmp/run-qcraft-maps-china";
constexpr char kCoreDumpFileName[] = "core.tar";
constexpr char kOnboardInfoFileName[] = "onboard_info.tar.gz";
constexpr char kQlogsDumpFileNameSuffix[] = "qlogs.tar.gz";
constexpr char kQlogsDumpSummaryFileName[] = "qlogs_summary.log";

constexpr char kRunPrimaryPath[] = "/media/s3/run_data";
constexpr char kRunSecondaryPath[] = "/media/s3/run_data_2";
constexpr char kRunThirdPath[] = "/media/s3/run_data_3";

constexpr char kFileSourceLocalHost[] = "local_host";
constexpr char kFileSourceLocalNas[] = "nas";
constexpr char kFileSourceLocalS3[] = "cloud";
constexpr char kFileSourceRemoteS3[] = "remote_cloud";

constexpr char kRunDashLabelInNasTillDate[] = "InNasTillDate";

constexpr char kSimulationCarID[] = "SIMULATE";

constexpr char kInternalDiskDir[] = "/";
constexpr char kExternalDiskDir[] = "/media/qcraft/mount_data/";
constexpr char kLoggingRunDirWithouExternalDisk[] = "/hosthome/run_data";
constexpr char kLoggingRunDirWithExternalDisk[] = "/media/qcraft/mount_data";

constexpr char kDeviceNamespeceOmc[] = "omc";
constexpr char kDeviceNamespaceXavier[] = "xavier";
constexpr char kNodeNameOmcPrefix[] = "main_node";
constexpr char kNodeNameXavierPrefix[] = "sensor_node";

constexpr char kUsMapsVersionFileWithSourceCode[] =
    "scripts/us-map-version.txt";
constexpr char kChinaMapsVersionFileWithSourceCode[] =
    "scripts/china-map-version.txt";

constexpr char kRunJobSourceFileSystem[] = "local_fs";
constexpr char kRunJobSourceKafka[] = "kafka";
constexpr char kSimStorageDir[] = "/media/s3/sim_msg";
constexpr char kSimAWSStorageDir[] = "/media/s3/sim_msg_aws";
constexpr char kArkScenarioCache[] = "/tmp/ark_scenario_cache.pb.txt";
constexpr char kSimTaskIDPattern[] = "[0-9]{19}";

constexpr char kCloudProfileAws[] = "default";
constexpr char kCloudProfileAliyun[] = "aliyun-standard";
constexpr char kS3AccessKeyId[] = "AWS_ACCESS_KEY_ID";
constexpr char kS3SecretKey[] = "AWS_SECRET_ACCESS_KEY";
