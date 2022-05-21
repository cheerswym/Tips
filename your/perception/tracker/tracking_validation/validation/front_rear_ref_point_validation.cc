#include <random>

#include "glog/logging.h"
#include "onboard/perception/tracker/motion_filter_2/car_model.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/perception/tracker/motion_filter_2/ukf.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

std::vector<PosHeadingVelocityMeasurement>
GenerateRearAxisMeasurementsFromUniformCircularMotion(
    const Vec2d center, const double radius, const double yaw_rate,
    const double start_angle, const double end_angle, const double delta_t) {
  const double yaw_diff = yaw_rate * delta_t;
  const int step = static_cast<int>((end_angle - start_angle) / yaw_diff);
  std::vector<PosHeadingVelocityMeasurement> measurements;
  for (int i = 0; i < step; ++i) {
    const double yaw_angle = start_angle + yaw_diff * i;
    PosHeadingVelocityMeasurement measurement;
    measurement.yaw() = yaw_angle + M_PI / 2;
    measurement.x() = center.x() + radius * std::cos(yaw_angle);
    measurement.y() = center.y() + radius * std::sin(yaw_angle);
    measurement.vel() = yaw_rate * radius;
    measurement.NormalizeYaw();
    measurements.push_back(measurement);
  }
  return measurements;
}

std::vector<PosHeadingVelocityMeasurement>
CalculateFrontAxisMeasurementFromRearAxisMeasurements(
    const std::vector<PosHeadingVelocityMeasurement>& rear_measurements,
    const double vehicle_length, const double yaw_rate) {
  std::vector<PosHeadingVelocityMeasurement> front_measurements;
  front_measurements.reserve(rear_measurements.size());
  for (int i = 0; i < rear_measurements.size(); ++i) {
    PosHeadingVelocityMeasurement front_measurement;
    front_measurement.x() =
        rear_measurements[i].x() +
        vehicle_length * std::cos(rear_measurements[i].yaw());
    front_measurement.y() =
        rear_measurements[i].y() +
        vehicle_length * std::sin(rear_measurements[i].yaw());

    const double rear_yaw = rear_measurements[i].yaw();
    const Vec2d rear_vel = rear_measurements[i].vel() *
                           Vec2d(std::cos(rear_yaw), std::sin(rear_yaw));
    const Eigen::Vector3d body_vec =
        vehicle_length *
        Eigen::Vector3d(std::cos(rear_yaw), std::sin(rear_yaw), 0.0);
    const Eigen::Vector3d diff_vel =
        -body_vec.cross(Eigen::Vector3d(0.0, 0.0, yaw_rate));
    const Vec2d front_vel = rear_vel + diff_vel.block<2, 1>(0, 0);

    front_measurement.yaw() = front_vel.FastAngle();
    front_measurement.vel() = front_vel.norm();
    front_measurement.NormalizeYaw();
    front_measurements.push_back(front_measurement);
  }
  return front_measurements;
}

void PrintMeasurements(
    const std::vector<PosHeadingVelocityMeasurement>& measurments) {
  std::cout << "Printing measurements:" << std::endl;
  for (const auto& mea : measurments) {
    std::cout << "measurement: " << mea.x() << " " << mea.y() << " "
              << mea.yaw() << " " << mea.vel() << std::endl;
  }
  std::cout << "==========================" << std::endl;
}

void Main() {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));

  UKF<CarModelCTRV> rear_ukf(params);
  UKF<CarModelCTRV> front_ukf(params);

  // Rear axis.
  const double dt = 0.01;
  const double yaw_rate = 0.2;
  std::vector<PosHeadingVelocityMeasurement> gt_rear_measurements =
      GenerateRearAxisMeasurementsFromUniformCircularMotion(
          /*center*/ {0.0, 0.0}, /*radius*/ 20.0, /*yaw_rate*/ yaw_rate,
          /*start_angle*/ 0.0, /*end_angle*/ M_PI / 2, /*delta_t*/ dt);

  PrintMeasurements(gt_rear_measurements);
  // Front axis.
  std::vector<PosHeadingVelocityMeasurement> gt_front_measurements =
      CalculateFrontAxisMeasurementFromRearAxisMeasurements(
          gt_rear_measurements, 5.0, yaw_rate);
  PrintMeasurements(gt_front_measurements);

  std::vector<PosHeadingMeasurement> rear_measurements;
  for (const auto& meas : gt_rear_measurements) {
    rear_measurements.push_back(
        PosHeadingMeasurement(meas.x(), meas.y(), meas.yaw()));
  }
  std::vector<PosHeadingMeasurement> front_measurements;
  for (int i = 0; i < gt_front_measurements.size(); ++i) {
    front_measurements.push_back(PosHeadingMeasurement(
        gt_front_measurements[i].x(), gt_front_measurements[i].y(),
        gt_rear_measurements[i].yaw()));
  }

  CarState rear_state;
  rear_state.x() = gt_rear_measurements[0].x();
  rear_state.y() = gt_rear_measurements[0].y();
  rear_ukf.Init(rear_state);
  CarState front_state;
  front_state.x() = gt_front_measurements[0].x();
  front_state.y() = gt_front_measurements[0].y();
  front_ukf.Init(front_state);

  for (size_t i = 1; i < rear_measurements.size(); ++i) {
    rear_ukf.Predict(dt * i);
    rear_ukf.Update(rear_measurements[i]);
    QLOG(INFO) << "Rear i=" << i << ", error pos= "
               << (Vec2d(gt_rear_measurements[i].x(),
                         gt_rear_measurements[i].y()) -
                   Vec2d(rear_ukf.state().x(), rear_ukf.state().y()))
                      .transpose();

    front_ukf.Predict(dt * i);
    front_ukf.Update(front_measurements[i]);
    QLOG(INFO) << "Front i=" << i << ", error pos= "
               << (Vec2d(gt_front_measurements[i].x(),
                         gt_front_measurements[i].y()) -
                   Vec2d(front_ukf.state().x(), front_ukf.state().y()))
                      .transpose();
  }
}

}  // namespace qcraft::tracker

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  qcraft::tracker::Main();
  return -1;
}
