#include <random>

#include "glog/logging.h"
#include "onboard/perception/tracker/motion_filter_2/car_model.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/perception/tracker/motion_filter_2/ukf.h"
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
  CarState gt;
  double x_ini{0.0}, y_ini{0.0}, yaw_ini{0.0}, v_ini{1.0}, yawd_ini{0.1},
      a_ini{0.2};
  gt.x() = x_ini;
  gt.y() = y_ini;
  gt.yaw() = yaw_ini;
  gt.vel() = v_ini;
  gt.yawd() = yawd_ini;
  gt.acc() = a_ini;

  using MotionModelType = CarModelCTRV;

  MotionModelType motion_model;

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));

  UKF<MotionModelType> ukf(params);
  // Filter initialization
  ukf.Init(gt);

  const double dt = 0.1;  // seconds

  unsigned seed = std::chrono::steady_clock::now().time_since_epoch().count();
  std::default_random_engine gen(seed);

  std::normal_distribution<double> noise{0, 1};
  const double autual_scale = 1.0;
  const double x_std = autual_scale * params.state_x_measurement_noise_std();
  const double y_std = autual_scale * params.state_x_measurement_noise_std();
  const double yaw_std =
      autual_scale * d2r(params.state_heading_measurement_noise_std());
  const double vel_std =
      autual_scale * params.state_vel_measurement_noise_std();

  TrackingValidationProto validation;

  const int count = 600;  // duration = dt * count = 0.1s * 600 = 60s

  for (int i = 0; i < count; ++i) {
    MeasurementsProto measurements_proto;
    ObjectsProto ground_truths_proto;
    ObjectsProto estimators_proto;

    // Generate GT path
    gt = motion_model.ComputeStateTransition(gt, dt);

    ukf.Predict(dt);

    // Generate gaussian nosie manually
    PositionMeasurement m;
    m.x() = gt.x() + x_std * noise(gen);
    m.y() = gt.y() + y_std * noise(gen);
    auto s = ukf.Update(m);

    HeadingMeasurement heading(gt.yaw() + yaw_std * noise(gen));
    s = ukf.Update(heading);

    VelocityMeasurement vel(gt.vel() + vel_std * noise(gen));
    s = ukf.Update(vel);

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
