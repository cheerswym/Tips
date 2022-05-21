#include "onboard/global/port_constants.h"

#include "onboard/proto/port.pb.h"
// Please do not modify this file directly, please refer to
// /onboard/proto/port.proto for the new port definition
// service
constexpr int kQpadServicePort = qcraft::QPAD_SERVICE_PORT;
constexpr int kSimServicePort = qcraft::SIM_SERVISER_PORT;
constexpr int kFleetManagerServicePort = qcraft::FLEET_MANAGER_SERVICE_PORT;
constexpr int kCameraStreamServicePortUs =
    qcraft::CAMERA_STREAM_SERVICE_US_PORT;
constexpr int kWorkflowsServicePort = qcraft::WORKFLOW_SERVICE_PORT;
constexpr int kQDaemonServicePort = qcraft::QDAEMON_SERVICE_PORT;
constexpr int kQviewQShowPublisherPort = qcraft::QVIEW_QSHOW_PUBLISHER_PORT;
constexpr int kAutonomyServicePort = qcraft::AUTONOMY_SERVICE_PORT;
constexpr int kPortalServicePort = qcraft::PORTAL_SERVICE_PORT;
constexpr int kTugboatServicePort = qcraft::TUGBOAT_SERVICE_PORT;
constexpr int kTeleopConnectPort = 6113;
constexpr int kRemoteCarServicePort = qcraft::REMOTECAR_SERVICE_PORT;
constexpr int kPrometheusClientHostPort = qcraft::PROMETHEUS_CLIENT_PORT;
constexpr int kMJPEGServicePort = qcraft::MJPEG_SERVICE_PORT;
constexpr int kSimServiceHttpPort = qcraft::SIM_HTTP_SERVICE_PORT;
constexpr int kInteractivePlaybackServiceHttpPort =
    qcraft::INTERRACTIVE_PLAYBACK_HTTP_SERVICE_PORT;
constexpr int kArkServiceHttpPort = qcraft::ARK_HTTP_SERIVCE_PORT;
constexpr int kVantageServiceHttpPort = qcraft::VANTAGE_HTTP_SERVICE_PORT;
constexpr int kRemoteEchoServicePort = qcraft::REMOTE_ECHO_SERVICE_PORT;
constexpr int kCameraStreamServicePortChina =
    qcraft::CAMERA_STREAM_SERVICE_CHINA_PORT;
constexpr int kQEventServicePort = qcraft::QEVENT_SERVICE_PORT;
constexpr int kImageSyncServicePort = qcraft::IMAGE_SYNC_SERVICE_PORT;
constexpr int kInteractivePlaybackServicePort =
    qcraft::INTERACTIVE_PLAYBACK_SERVICE_PORT;
constexpr int kVantageServicePort = qcraft::VANTAGE_SERVICE_PORT;
constexpr int kArkServicePort = qcraft::ARK_SERVICE_PORT;
constexpr int kRemoteInteractivePlaybackServicePort =
    qcraft::REMOTE_INTERACTIVE_PLAYBACK_SERVICE_PORT;

constexpr int kRearLedPort = qcraft::REAR_LEAD_PORT;

// sensor
constexpr int kOusterLidarCommandClientPort =
    qcraft::OUSTER_LIDAR_COMMAND_CLIENT_PORT;
constexpr int kPandarLidarCommandClientPort =
    qcraft::PANDAR_LIDAR_COMMAND_CLIENT_PORT;
