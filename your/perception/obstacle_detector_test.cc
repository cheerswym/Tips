#include "onboard/perception/obstacle_detector.h"

#include "gtest/gtest.h"
#include "onboard/global/run_context.h"
#include "onboard/lidar/spin_reader_test_util.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/params/param_manager.h"
#include "s2/s2cell.h"
#include "s2/s2latlng.h"

namespace qcraft {

TEST(ObstacleDetectorTest, TestObstacleDetector) {
  FLAGS_enable_context_test = 1;
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);

  ImageryManager imagery_manager;
  ObstacleDetector obstacle_detector(run_params, &imagery_manager,
                                     ThreadPool::DefaultPool());
  // This is a workaround to avoid data race when calling S2CellId::FromFaceIJ
  // concurrently, which fails the TSAN test.
  S2CellId(S2LatLng::FromRadians(0, 0));
  FLAGS_map = "dojo";
  CoordinateConverter coordinate_converter = CoordinateConverter::FromLocale();
  for (const auto& spin : spin_reader::LoadSpinsForTest(LIDAR_PANDAR_64)) {
    const auto global_coord = coordinate_converter.SmoothToGlobal(
        {spin->FirstScan().pose.x, spin->FirstScan().pose.y, 0.0});
    imagery_manager.Load(global_coord.x(), global_coord.y(),
                         /*clear_far_patches=*/false);
    const LidarFrame lidar_frame(
        Spin::CopySpinOnShm(*spin, LIDAR_PANDAR_64, LDR_CENTER));
    labeling::LabelFrameProto labelframe_proto;
    const auto obstacle_manager = obstacle_detector.DetectObstacles(
        {lidar_frame}, lidar_frame.MidPose(), coordinate_converter,
        &labelframe_proto, mapping::SemanticMapManager(),
        run_params.vehicle_params());
    EXPECT_GT(obstacle_manager->obstacle_ptrs().size(), 0);
  }
}

}  // namespace qcraft
