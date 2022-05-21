#include "onboard/perception/segmentation/proposer_test_util.h"

#include <algorithm>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/vis/common/color.h"

namespace qcraft::segmentation {

namespace {

vis::Color GetClusterColor(const MeasurementType type) {
  switch (type) {
    case MT_VEHICLE:
      return vis::Color::kGreen;
    case MT_PEDESTRIAN:
      return vis::Color::kOrange;
    case MT_CYCLIST:
      return vis::Color::kYellow;
    case MT_MOTORCYCLIST:
      return vis::Color::kMagenta;
    case MT_UNKNOWN:
      return vis::Color::kWhite;
    case MT_VEGETATION:
      return vis::Color::kDarkGreen;
    case MT_BARRIER:
      return vis::Color::kTiffanyBlue;
    case MT_CONE:
      return vis::Color::kLightMagenta;
    case MT_WARNING_TRIANGLE:
      return vis::Color::kLightRed;
    case MT_FLYING_BIRD:
      return vis::Color::kMiddleBlueGreen;
    case MT_STATIC_OBJECT:
    case MT_FOD:
    case MT_ROAD:
    case MT_MIST:
      return vis::Color::kGray;
  }
}

}  // namespace

void DrawProposedClusterToCanvas(const ProposedCluster& cluster,
                                 const double z_offset,
                                 const std::optional<vis::Color>& color) {
  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/proposer_test_cvs");
  const auto contour = cluster_util::ComputeContour(cluster);
  const float ground_z = cluster.ComputeGroundZ();
  const vis::Color cluster_color =
      color ? *color : GetClusterColor(cluster.type());
  canvas.DrawPolygon(contour, ground_z + z_offset, cluster_color);
  if (cluster.bounding_box()) {
    const auto& box = *cluster.bounding_box();
    canvas.DrawBox({box.center(), ground_z + z_offset}, box.heading(),
                   {box.length(), box.width()}, cluster_color);
  }
}

}  // namespace qcraft::segmentation
