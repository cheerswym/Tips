#ifndef ONBOARD_GLOBAL_CONSTANTS_H_
#define ONBOARD_GLOBAL_CONSTANTS_H_

// Onboard constants.

// The name of the run info text binary proto file.
extern const char kRunNamePattern[];
extern const char kRunInfoBinaryFileName[];
extern const char kImportRunsFileName[];
extern const char kLabelFileName[];
extern const char kDefualtLabelStfDir[];
extern const char kSecondaryDefualtLabelStfDir[];
extern const char kSystemInfoProtoLog[];
extern const char kLiteMsgFileName[];
extern const char kLidarDataFileName[];
extern const char kLidarDataFileNameOld[];
extern const char kCameraJpgImgFileName[];
extern const char kCameraJpgImgLowResolutionFileName[];
extern const char kSystemCheckLogsFileName[];
extern const char kRsyncFinishFlagFileName[];
extern const char kLiteMsgSimpleFileName[];
extern const char kLidarDataSimpleFileName[];
extern const char kCameraJpgImgSimpleFileName[];
extern const char kCameraJpgImgLowResolutionSimpleFileName[];
extern const char kRunFilesHintFileName[];
extern const char kCoreDumpSimpleFileName[];
extern const char kMapSimpleFileName[];
extern const char kOnboardUploadSnippetMetaRelPath[];

extern const char kPoseFileName[];
extern const char kLegacyMapFileName[];
extern const char kDefaultPoseStfDir[];
extern const char kDefaultMapName[];
extern const char kDefaultChinaMapName[];
extern const char kDefaultChinaMapFileName[];
extern const char kRunReleaseChinaMapDirSymlink[];
extern const char kCoreDumpFileName[];
extern const char kOnboardInfoFileName[];
extern const char kQlogsDumpFileNameSuffix[];
extern const char kQlogsDumpSummaryFileName[];

extern const char kRunPrimaryPath[];
extern const char kRunSecondaryPath[];
extern const char kRunThirdPath[];

extern const char kFileSourceLocalHost[];
extern const char kFileSourceLocalNas[];
extern const char kFileSourceLocalS3[];
extern const char kFileSourceRemoteS3[];

extern const char kRunDashLabelInNasTillDate[];

extern const char kSimulationCarID[];

extern const char kInternalDiskDir[];
extern const char kExternalDiskDir[];
extern const char kLoggingRunDirWithouExternalDisk[];
extern const char kLoggingRunDirWithExternalDisk[];

extern const char kDeviceNamespeceOmc[];
extern const char kDeviceNamespaceXavier[];
extern const char kNodeNameOmcPrefix[];
extern const char kNodeNameXavierPrefix[];

extern const char kUsMapsVersionFileWithSourceCode[];
extern const char kChinaMapsVersionFileWithSourceCode[];

extern const char kRunJobSourceFileSystem[];
extern const char kRunJobSourceKafka[];
extern const char kSimStorageDir[];
extern const char kSimAWSStorageDir[];
extern const char kArkScenarioCache[];
extern const char kSimTaskIDPattern[];

extern const char kCloudProfileAws[];
extern const char kCloudProfileAliyun[];
extern const char kS3AccessKeyId[];
extern const char kS3SecretKey[];

#endif  // ONBOARD_GLOBAL_CONSTANTS_H_
