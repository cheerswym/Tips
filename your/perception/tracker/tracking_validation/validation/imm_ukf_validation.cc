#include <random>

#include "glog/logging.h"
#include "onboard/perception/tracker/motion_filter_2/estimator.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/perception/tracker/tracking_validation/proto/validation_imm.pb.h"
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
  auto& vel = *object.mutable_vel();
  pos.set_x(state.x());
  pos.set_y(state.y());
  vel.set_x(state.vel() * std::cos(state.yaw()));
  vel.set_y(state.vel() * std::sin(state.yaw()));
}

void AddIMMToValidation(const StateData& state, const Eigen::VectorXd& mu_in,
                        EstimationsProto* ests_proto) {
  int N = mu_in.rows();
  auto& est = *ests_proto->add_estimations();
  est.set_motion_num(N);
  auto& outstate = *est.mutable_est_state();
  auto& mu = *est.mutable_est_mu();
  auto& pos = *outstate.mutable_pos();
  auto& vel = *outstate.mutable_vel();
  pos.set_x(state.x());
  pos.set_y(state.y());
  vel.set_x(state.GetVx());
  vel.set_y(state.GetVy());
  for (int i = 0; i < N; ++i) {
    mu.add_mu(mu_in[i]);
  }
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

  // Motion model for ground truth.
  CarModelCTRV motion_model;

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/"
      "urban_car_model_ctra_car.pb.txt",
      &params));
  auto params_ca = params;
  params_ca.set_type(MotionFilterParamProto::CAR_CA);
  auto params_ctrv = params;
  params_ctrv.set_type(MotionFilterParamProto::CAR_CTRV);

  std::vector<MotionFilterParamProto> v{params_ca, params_ctrv};

  Estimator estimator(ToImmProto(v, {0.5, 0.5}));
  // Filter initialization
  const auto init_state = StateData(gt);
  double timestamp = 0.0;
  estimator.Init(init_state, timestamp);

  const double dt = 0.1;  // seconds

  unsigned seed = std::chrono::steady_clock::now().time_since_epoch().count();
  std::default_random_engine gen(seed);

  std::normal_distribution<double> noise{0, 1};
  const double autual_scale = 0.5;
  std::cout << "params.state_x_measurement_noise_std()     :"
            << params.state_x_measurement_noise_std() << std::endl;
  const double x_std = autual_scale * params.state_x_measurement_noise_std();
  const double y_std = autual_scale * params.state_y_measurement_noise_std();
  const double yaw_std =
      autual_scale * d2r(params.state_heading_measurement_noise_std());
  // const double vel_std =
  //     autual_scale * params.state_vel_measurement_noise_std();

  TrackingValidationProto validation;

  const int count = 40;  // duration = dt * count = 0.1s * 600 = 60s

  for (int i = 0; i < count; ++i) {
    QLOG(INFO) << "***************count: " << i;
    MeasurementsProto measurements_proto;
    ObjectsProto ground_truths_proto;
    EstimationsProto estimators_proto;

    // Generate GT path
    gt = motion_model.ComputeStateTransition(gt, dt);

    timestamp += dt;
    estimator.Predict(timestamp);

    // Generate gaussian nosie manually
    const double m_x = gt.x() + x_std * noise(gen);
    const double m_y = gt.y() + y_std * noise(gen);
    const double m_yaw = gt.yaw() + yaw_std * noise(gen);

    PositionMeasurement m(m_x, m_y);

    const PosHeadingMeasurement pos_yaw(m_x, m_y, m_yaw);
    estimator.Update(pos_yaw);

    QLOG(INFO) << "Update State: \n" << estimator.GetStateData();

    auto mu = estimator.model_probability();

    auto s = estimator.GetStateData();

    AddMeasurementsToValidation(m, &measurements_proto);
    AddObjectsToValidation(gt, &ground_truths_proto);
    AddIMMToValidation(s, mu, &estimators_proto);

    *validation.add_measurements() = measurements_proto;
    *validation.add_ground_truths() = ground_truths_proto;
    *validation.add_estimators() = estimators_proto;
  }

  // Export data to files
  if (FLAGS_save_data) {
    QCHECK(file_util::ProtoToTextFile(validation, "/tmp/data_imm.pb.txt"));
  }

  return 0;
}

}  // namespace qcraft::tracker

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return qcraft::tracker::Main();
}
