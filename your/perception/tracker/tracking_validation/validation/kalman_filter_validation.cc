#include <random>

#include "glog/logging.h"
#include "onboard/perception/tracker/motion_filter_2/kalman_filter.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/perception/tracker/motion_filter_2/point_model.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/perception/tracker/tracking_validation/proto/validation.pb.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

DEFINE_bool(save_data, false, "save all the data.");

template <typename MeasurementType>
void AddMeasurementsToValidation(const MeasurementType& meas,
                                 MeasurementsProto* meassurements_proto) {
  auto& measurement = *meassurements_proto->add_measurements();
  auto& laser_measurement = *measurement.mutable_laser_measurement();
  auto& bb = *laser_measurement.mutable_detection_bounding_box();
  bb.set_x(meas.x());
  bb.set_y(meas.y());
}

template <typename StateType>
void AddObjectsToValidation(const StateType& state,
                            ObjectsProto* objects_proto) {
  auto& object = *objects_proto->add_objects();
  auto& pos = *object.mutable_pos();
  pos.set_x(state.x());
  pos.set_y(state.y());
}

int Main() {
  PointState gt;
  double x_ini{0.0}, y_ini{0.0}, vx_ini{1.0}, vy_ini{-1.0};
  gt.x() = x_ini;
  gt.y() = y_ini;
  gt.vx() = vx_ini;
  gt.vy() = vy_ini;

  PointModelCV POINT_CV;

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/point_model.pb.txt",
      &params));

  KalmanFilter<PointModelCV> kf(params);
  // Filter initialization
  kf.Init(gt);

  const double dt = 0.1;  // seconds

  unsigned seed = std::chrono::steady_clock::now().time_since_epoch().count();
  std::default_random_engine gen(seed);

  std::normal_distribution<double> noise{0, 1};
  const double autual_scale = 1.0;
  const double x_std =
      autual_scale * sqrt(params.state_x_measurement_noise_std());
  const double y_std =
      autual_scale * sqrt(params.state_y_measurement_noise_std());
  const double vx_std =
      autual_scale * sqrt(params.state_vel_x_measurement_noise_std());
  const double vy_std =
      autual_scale * sqrt(params.state_vel_y_measurement_noise_std());

  TrackingValidationProto validation;

  const int count = 600;  // duration = dt * count = 0.1s * 600 = 60s

  for (int i = 0; i < count; ++i) {
    MeasurementsProto measurements_proto;
    ObjectsProto ground_truths_proto;
    ObjectsProto estimators_proto;

    // Generate GT path
    gt = POINT_CV.ComputeStateTransition(gt, dt);

    kf.Predict(dt);

    // Generate gaussian nosie manually
    PosSpeedMeasurement m;

    m.x() = gt.x() + x_std * noise(gen);
    m.y() = gt.y() + y_std * noise(gen);
    m.vx() = gt.vx() + vx_std * noise(gen);
    m.vy() = gt.vy() + vy_std * noise(gen);
    auto s = kf.Update(m);

    AddMeasurementsToValidation(m, &measurements_proto);
    AddObjectsToValidation(gt, &ground_truths_proto);
    AddObjectsToValidation(s, &estimators_proto);

    *validation.add_measurements() = measurements_proto;
    *validation.add_ground_truths() = ground_truths_proto;
    *validation.add_estimators() = estimators_proto;
  }

  if (FLAGS_save_data) {
    QCHECK(file_util::ProtoToTextFile(validation, "/tmp/data.pb.txt"));
  }

  return 0;
}

}  // namespace qcraft::tracker

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return qcraft::tracker::Main();
}
