#ifndef ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_GATE_H_
#define ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_GATE_H_

#include <memory>
#include <utility>
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

using GateResult = std::pair<bool, double>;

template <typename TrackType, typename MeasurementType>
class Gate {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit Gate(const std::vector<double>& thresholds)
      : thresholds_(thresholds) {}

  virtual ~Gate() = default;

  virtual GateResult Gating(const M& measurement, const T& track) = 0;

  virtual association::GatesProto::GateType Type() = 0;

 protected:
  std::vector<double> thresholds_;
};

template <typename TrackType, typename MeasurementType>
class CircleGate final : public Gate<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit CircleGate(const std::vector<double>& thresholds)
      : Gate<T, M>(thresholds) {}

  GateResult Gating(const M& measurement, const T& track) override;

  association::GatesProto::GateType Type() override {
    return association::GatesProto::CIRCLE;
  }
};

template <typename TrackType, typename MeasurementType>
class TypeCircleGate final : public Gate<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit TypeCircleGate(const std::vector<double>& thresholds)
      : Gate<T, M>(thresholds) {}

  GateResult Gating(const M& measurement, const T& track) override;

  association::GatesProto::GateType Type() override {
    return association::GatesProto::TYPE_CIRCLE;
  }
};

template <typename TrackType, typename MeasurementType>
class ElipseGate final : public Gate<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit ElipseGate(const std::vector<double>& thresholds)
      : Gate<T, M>(thresholds) {}

  GateResult Gating(const M& measurement, const T& track) override;

  association::GatesProto::GateType Type() override {
    return association::GatesProto::ELIPSE;
  }
};

template <typename TrackType, typename MeasurementType>
class SpeedGate final : public Gate<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit SpeedGate(const std::vector<double>& thresholds)
      : Gate<T, M>(thresholds) {}

  GateResult Gating(const M& measurement, const T& track) override;

  association::GatesProto::GateType Type() override {
    return association::GatesProto::SPEED;
  }
};

template <typename TrackType, typename MeasurementType>
class BevIouGate final : public Gate<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit BevIouGate(const std::vector<double>& thresholds)
      : Gate<T, M>(thresholds) {}

  GateResult Gating(const M& measurement, const T& track) override;

  association::GatesProto::GateType Type() override {
    return association::GatesProto::BEV_IOU;
  }
};

template <typename TrackType, typename MeasurementType>
class ImageIouGate final : public Gate<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit ImageIouGate(const std::vector<double>& thresholds)
      : Gate<T, M>(thresholds) {}

  GateResult Gating(const M& measurement, const T& track) override;

  association::GatesProto::GateType Type() override {
    return association::GatesProto::IMG_IOU;
  }
};

template <typename TrackType, typename MeasurementType>
class TypeGate final : public Gate<TrackType, MeasurementType> {
 public:
  using T = TrackType;
  using M = MeasurementType;

  explicit TypeGate(const std::vector<double>& thresholds)
      : Gate<T, M>(thresholds) {}

  GateResult Gating(const M& measurement, const T& track) override;

  association::GatesProto::GateType Type() override {
    return association::GatesProto::TYPE;
  }

  int Dim() const { return dim_; }

 private:
  static constexpr int kTypeMatrix[14 * 14] = {
      /******     0, 1, 2, 3, 4, 5, 6, 7, 8, 9, a, b, c, d*/
      /*[0] UNK*/ 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1,
      /*[1] VEH*/ 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,
      /*[2] MOT*/ 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,
      /*[3] PED*/ 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,
      /*[4] CYC*/ 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,
      /*[5] FOD*/ 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      /*[6] STO*/ 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1,
      /*[7] VEG*/ 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
      /*[8] BAR*/ 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0,
      /*[9] ROA*/ 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      /*[a] CON*/ 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1,
      /*[b] MIS*/ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      /*[c] BIR*/ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
      /*[d] TRI*/ 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1,
  };
  const int dim_ = 14;
};

template <typename T, typename M>
std::shared_ptr<Gate<T, M>> BuildGateFromType(
    const association::GatesProto::GateType& type,
    const std::vector<double>& thresholds) {
  QCHECK(!thresholds.empty());
  switch (type) {
    case association::GatesProto::CIRCLE:
      return std::make_shared<CircleGate<T, M>>(thresholds);
    case association::GatesProto::ELIPSE:
      return std::make_shared<ElipseGate<T, M>>(thresholds);
    case association::GatesProto::BEV_IOU:
      return std::make_shared<BevIouGate<T, M>>(thresholds);
    case association::GatesProto::TYPE:
      return std::make_shared<TypeGate<T, M>>(thresholds);
    case association::GatesProto::SPEED:
      return std::make_shared<SpeedGate<T, M>>(thresholds);
    case association::GatesProto::TYPE_CIRCLE:
      return std::make_shared<TypeCircleGate<T, M>>(thresholds);
    case association::GatesProto::IMG_IOU:
      return std::make_shared<ImageIouGate<T, M>>(thresholds);
    default:
      return nullptr;
  }
}

template <typename T, typename M>
std::shared_ptr<Gate<T, M>> BuildGateFromProto(
    const association::GatesProto_GateParamProto& proto) {
  const auto& type = proto.gate_type();
  std::vector<double> thresholds;
  for (int i = 0; i < proto.gate_thresholds_size(); ++i) {
    thresholds.push_back(proto.gate_thresholds(i));
  }
  return BuildGateFromType<T, M>(type, thresholds);
}

}  // namespace association
}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_GATE_H_
