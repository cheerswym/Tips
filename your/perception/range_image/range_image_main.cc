#include "offboard/vfs/vfs.h"
#include "onboard/global/init_qcraft.h"
#include "onboard/lidar/spin_reader.h"
#include "onboard/perception/range_image/range_image.h"
#include "onboard/utils/run_util.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

int main(int argc, char** argv) {
  using namespace qcraft;  // NOLINT
  InitQCraft(&argc, &argv);

  const std::string run_dir = VFS::GetRunPath(FLAGS_run);
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromRunDir(run_dir);
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);

  const LidarId lidar_id = LDR_FRONT_LEFT;
  std::optional<LidarParametersProto> lidar_param;
  for (const auto& lp : run_params.vehicle_params().lidar_params()) {
    if (lp.installation().lidar_id() == lidar_id) {
      lidar_param = lp;
    }
  }
  QCHECK(lidar_param.has_value());

  spin_reader::TraverseLidarFramesFromRunDir(
      run_dir, FLAGS_start, FLAGS_end, lidar_id,
      [&](uint64_t timestamp, LidarFrame lidar_frame) {
        const RangeImage ri(*lidar_param, lidar_frame, {});
        cv::Mat img_to_show;
        cv::vconcat(ri.range_image(), ri.intensity_image(), img_to_show);

        const auto& img_size = img_to_show.size();
        cv::resize(img_to_show, img_to_show, img_size * 2);
        cv::imshow("Range Image", img_to_show);
        cv::waitKey(1);
      });
}
