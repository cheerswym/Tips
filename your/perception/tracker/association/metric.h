#ifndef ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_METRIC_H_
#define ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_METRIC_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "onboard/lite/check.h"
#include "onboard/perception/multi_camera_fusion/track.h"
#include "onboard/perception/tracker/association/proto/associator_config.pb.h"
#include "onboard/perception/tracker/track.h"
#include "onboard/perception/tracker/tracker_util.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::tracker {
namespace association {

constexpr double kMaxSimilarityValue = 1e3;

template <typename TrackType, typename MeasurementType>
class Metric {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit Metric(const std::vector<double>& thresholds)
      : thresholds_(thresholds) {}

  virtual ~Metric() = default;

  virtual double Similarity(const M& measurement, const T& track) = 0;

  virtual association::MetricsProto::MetricType Type() = 0;

 protected:
  std::vector<double> thresholds_;
};

template <typename TrackType, typename MeasurementType>
class EuclideanMetric final : public Metric<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit EuclideanMetric(const std::vector<double>& thresholds)
      : Metric<T, M>(thresholds) {}

  double Similarity(const M& measurement, const T& track) override;

  association::MetricsProto::MetricType Type() override {
    return association::MetricsProto::EUCLIDEAN_DIST;
  }
};

// If DoF is changing, PValueMetric is a good choice.
template <typename TrackType, typename MeasurementType>
class PValueMetric final : public Metric<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit PValueMetric(const std::vector<double>& thresholds)
      : Metric<T, M>(thresholds) {}

  double Similarity(const M& measurement, const T& track) override;

  association::MetricsProto::MetricType Type() override {
    return association::MetricsProto::P_VALUE;
  }
};

// If DoF is always the same, MahalanobisMetric is a good choice.
template <typename TrackType, typename MeasurementType>
class RadarMahalanobisMetric final : public Metric<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit RadarMahalanobisMetric(const std::vector<double>& thresholds)
      : Metric<T, M>(thresholds) {}

  double Similarity(const M& measurement, const T& track) override;

  association::MetricsProto::MetricType Type() override {
    return association::MetricsProto::RADAR_MAHALANOBIS_DIST;
  }
};

template <typename TrackType, typename MeasurementType>
class VehicleIouMetric final : public Metric<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit VehicleIouMetric(const std::vector<double>& thresholds)
      : Metric<T, M>(thresholds) {}

  double Similarity(const M& measurement, const T& track) override;

  association::MetricsProto::MetricType Type() override {
    return association::MetricsProto::VEHICLE_IOU;
  }
};

template <typename TrackType, typename MeasurementType>
class BevIouMetric final : public Metric<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit BevIouMetric(const std::vector<double>& thresholds)
      : Metric<T, M>(thresholds) {}

  double Similarity(const M& measurement, const T& track) override;

  association::MetricsProto::MetricType Type() override {
    return association::MetricsProto::BEV_IOU;
  }
};

template <typename TrackType, typename MeasurementType>
class ImageIouMetric final : public Metric<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit ImageIouMetric(const std::vector<double>& thresholds)
      : Metric<T, M>(thresholds) {}

  double Similarity(const M& measurement, const T& track) override;

  association::MetricsProto::MetricType Type() override {
    return association::MetricsProto::IMG_IOU;
  }
};

template <typename TrackType, typename MeasurementType>
class ImageAppearanceFeatureMetric final
    : public Metric<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit ImageAppearanceFeatureMetric(const std::vector<double>& thresholds)
      : Metric<T, M>(thresholds) {}

  double Similarity(const M& measurement, const T& track) override;

  association::MetricsProto::MetricType Type() override {
    return association::MetricsProto::IMG_APPEARANCE_FEATURE;
  }
};

template <typename TrackType, typename MeasurementType>
class CameraTrackerIdMetric final : public Metric<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit CameraTrackerIdMetric(const std::vector<double>& thresholds)
      : Metric<T, M>(thresholds) {}

  double Similarity(const M& measurement, const T& track) override;

  association::MetricsProto::MetricType Type() override {
    return association::MetricsProto::CAMERA_TRACKER_ID;
  }
};

template <typename T, typename M>
std::shared_ptr<Metric<T, M>> BuildMetricFromType(
    const association::MetricsProto::MetricType& type,
    const std::vector<double>& thresholds) {
  switch (type) {
    case association::MetricsProto::EUCLIDEAN_DIST:
      return std::make_shared<EuclideanMetric<T, M>>(thresholds);
    case association::MetricsProto::VEHICLE_IOU:
      return std::make_shared<VehicleIouMetric<T, M>>(thresholds);
    case association::MetricsProto::RADAR_MAHALANOBIS_DIST:
      return std::make_shared<RadarMahalanobisMetric<T, M>>(thresholds);
    case association::MetricsProto::IMG_IOU:
      return std::make_shared<ImageIouMetric<T, M>>(thresholds);
    case association::MetricsProto::IMG_APPEARANCE_FEATURE:
      return std::make_shared<ImageAppearanceFeatureMetric<T, M>>(thresholds);
    case association::MetricsProto::CAMERA_TRACKER_ID:
      return std::make_shared<CameraTrackerIdMetric<T, M>>(thresholds);
    case association::MetricsProto::BEV_IOU:
      return std::make_shared<BevIouMetric<T, M>>(thresholds);
    default:
      QLOG(FATAL) << "Unknown supported metric.";
      return nullptr;
  }
}

template <typename T, typename M>
std::shared_ptr<Metric<T, M>> BuildMetricFromType(
    const association::MetricsProto_MetricParamProto& proto) {
  const auto& type = proto.metric_type();
  std::vector<double> thresholds;
  for (int i = 0; i < proto.metric_thresholds_size(); ++i) {
    thresholds.push_back(proto.metric_thresholds(i));
  }
  return BuildMetricFromType<T, M>(type, thresholds);
}

}  // namespace association
}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_METRIC_H_
