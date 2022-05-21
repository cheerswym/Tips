#include <random>

#include "glog/logging.h"
#include "onboard/perception/tracker/motion_filter_2/car_model.h"
#include "onboard/perception/tracker/motion_filter_2/extent_filter.h"
#include "onboard/perception/tracker/motion_filter_2/extent_meas_model.h"
#include "onboard/perception/tracker/motion_filter_2/extent_model.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/perception/tracker/tracking_validation/proto/validation.pb.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

DEFINE_bool(save_data, false, "save all the data.");

template <typename MeasurementType>
void AddMeasurementsToValidation(const MeasurementType& bbox,
                                 MeasurementsProto* meassurements_proto) {
  auto& measurement = *meassurements_proto->add_measurements();
  auto& laser_measurement = *measurement.mutable_laser_measurement();
  auto& bb = *laser_measurement.mutable_detection_bounding_box();
  bb.set_x(0.0);
  bb.set_y(0.0);
  bb.set_length(bbox.length());
  bb.set_width(bbox.width());
}

template <typename StateType>
void AddObjectsToValidation(const StateType& bbox,
                            ObjectsProto* objects_proto) {
  auto& object = *objects_proto->add_objects();
  auto& pos = *object.mutable_pos();
  auto& bb = *object.mutable_bounding_box();
  pos.set_x(10.0);
  pos.set_y(10.0);
  bb.set_length(bbox.length());
  bb.set_width(bbox.width());
}

int Main() {
  CarState gt;
  double x_ini{0.0}, y_ini{0.0}, yaw_ini{0.0}, v_ini{1.0}, yawd_ini{0.1},
      a_ini{0.2};
  gt.x() = x_ini;
  gt.y() = y_ini;
  gt.yaw() = yaw_ini;
  gt.vel() = v_ini;
  gt.yawd() = yawd_ini;
  gt.acc() = a_ini;
  BBoxState bbox_gt(5.0, 2.5, 0.0);
  PositionMeasurement pos_m(gt.x() + 5.0, gt.y() + 5.0);

  using MotionModelType = BBoxModel;

  MotionModelType motion_model;

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));

  ExtentFilter<BBoxModel> ef(params);
  // Filter initialization
  ef.Init(bbox_gt);

  const double dt = 0.1;  // seconds

  unsigned seed = std::chrono::steady_clock::now().time_since_epoch().count();
  std::default_random_engine gen(seed);

  std::normal_distribution<double> noise{0, 1};
  const double autual_scale = 1.0;
  const double length_std =
      autual_scale * params.state_length_measurement_noise_std();
  const double width_std =
      autual_scale * params.state_width_measurement_noise_std();

  TrackingValidationProto validation;

  const int count = 600;  // duration = dt * count = 0.1s * 600 = 60s

  for (int i = 0; i < count; ++i) {
    MeasurementsProto measurements_proto;
    ObjectsProto ground_truths_proto;
    ObjectsProto estimators_proto;

    // Generate GT path
    bbox_gt = motion_model.ComputeStateTransition(bbox_gt, dt);

    ef.Predict(dt);

    // Generate gaussian nosie manually
    PositionMeasurement m;
    BBoxMeasurement bbox_m;
    bbox_m.length() = bbox_gt.length() + length_std * noise(gen);
    bbox_m.width() = bbox_gt.width() + width_std * noise(gen);
    auto s = ef.Update(bbox_m);

    AddMeasurementsToValidation(bbox_m, &measurements_proto);
    AddObjectsToValidation(bbox_gt, &ground_truths_proto);
    AddObjectsToValidation(s, &estimators_proto);

    *validation.add_measurements() = measurements_proto;
    *validation.add_ground_truths() = ground_truths_proto;
    *validation.add_estimators() = estimators_proto;
  }

  if (FLAGS_save_data) {
    QCHECK(file_util::ProtoToTextFile(validation, "/tmp/extent_data.pb.txt"));
  }

  return 0;
}

}  // namespace qcraft::tracker

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return qcraft::tracker::Main();
}
