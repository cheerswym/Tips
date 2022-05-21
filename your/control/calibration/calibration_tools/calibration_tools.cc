#include "onboard/control/calibration/calibration_tools/calibration_tools.h"

#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "offboard/vis/vantage/charts/chart_util.h"
#include "onboard/control/calibration/calibration_tools/calibration_force.h"
#include "onboard/control/calibration/calibration_tools/calibration_idle.h"
#include "onboard/control/calibration/calibration_tools/calibration_slide.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/utils/file_util.h"
#include "onboard/vis/common/color.h"

namespace qcraft {
namespace control {
namespace calibration {

void ExportProtoToSubchart(
    const std::vector<double> &x, const std::vector<std::vector<double>> &y,
    const std::string &x_name, const std::vector<std::string> &y_names,
    const vis::Color &color,
    const vis::vantage::ChartSeriesDataProto::PenStyle &pen_style,
    vis::vantage::ChartDataProto *chart) {
  QCHECK_NOTNULL(chart);
  CHECK_EQ(y.size(), y_names.size());

  auto *subchart = chart->add_subcharts();
  subchart->set_x_name(x_name);
  for (int i = 0; i < y.size(); ++i) {
    auto *y_elem = subchart->add_y();
    y_elem->set_name(y_names[i]);
    color.ToProto(y_elem->mutable_color());
    y_elem->set_pen_style(pen_style);
    for (int j = 0; j < y[i].size(); ++j) {
      subchart->add_x_values(x[j]);
      y_elem->add_values(y[i][j]);
    }
  }
}

CalibrationTools::CalibrationTools(const std::string &dirname,
                                   const std::string &carid,
                                   const std::string &method,
                                   double throttle_deadzone,
                                   double brake_deadzone,
                                   bool is_save_sampledata) {
  if (0 != access(dirname.c_str(), 0)) {
    if (mkdir(dirname.c_str(), S_IRWXU)) {
      LOG(ERROR) << "[Calibration] Fail to build save dir !";
      return;
    }
  }

  if ((method != "IDLE") && (method != "SLIDE") && (method != "FORCE")) {
    init_success_ = false;
    LOG(ERROR) << "Data Process Init is failed !" << method;
    return;
  }

  const std::string file = absl::StrCat(dirname, "/", carid);
  if (method == "IDLE") {
    method_ = Method::IDLE;
    calibration_method_ = std::make_unique<CalibrationIdle>();
    // Match with calibration_force.cc
    filename_sample_ = absl::StrCat(file, "_idle_sample.pb.txt");
    filename_output_ = absl::StrCat(file, "_idle.pb.txt");
    init_success_ =
        calibration_method_->Init(file, throttle_deadzone, brake_deadzone);
  } else if (method == "SLIDE") {
    method_ = Method::SLIDE;
    calibration_method_ = std::make_unique<CalibrationSlide>();
    // Match with calibration_force.cc
    filename_sample_ = absl::StrCat(file, "_slide_sample.pb.txt");
    filename_output_ = absl::StrCat(file, "_slide.pb.txt");
    init_success_ =
        calibration_method_->Init(file, throttle_deadzone, brake_deadzone);
  } else {
    method_ = Method::FORCE;
    calibration_method_ = std::make_unique<CalibrationForce>();
    filename_sample_ = absl::StrCat(file, "_force_sample.pb.txt");
    filename_output_ = absl::StrCat(file, "_force.pb.txt");
    init_success_ =
        calibration_method_->Init(file, throttle_deadzone, brake_deadzone);
  }

  constexpr int kWindowSize = 10;
  pitch_angle_filter_ =
      std::make_unique<apollo::common::MeanFilter>(kWindowSize);

  is_save_sampledata_ = is_save_sampledata;
}

void CalibrationTools::Reset() { calibration_method_->Reset(); }

bool CalibrationTools::Process(double speed, double acceleration,
                               double pitch_angle, double throttle,
                               double brake, Chassis::GearPosition gear,
                               vis::vantage::ChartDataProto *chart) {
  QCHECK_EQ(init_success_, true);

  // Filter pitch angle.
  const auto pitch_angle_smooth = pitch_angle_filter_->Update(pitch_angle);

  // Pitch angle < 0 is clamp slope.
  constexpr double kGravityAcceleration = 9.80665;
  const auto acceleration_offset =
      -kGravityAcceleration * std::sin(pitch_angle_smooth);
  const auto acceleration_pure = acceleration + acceleration_offset;

  // Speed truncate.
  constexpr double kEpsilonSpeed = 0.01;
  if (gear == Chassis::GEAR_REVERSE) {
    speed = std::min(speed, -kEpsilonSpeed);
  } else {
    speed = std::max(speed, kEpsilonSpeed);
  }

  // Calibration process.
  if (!calibration_method_->Process(speed, acceleration_pure, throttle,
                                    brake)) {
    return false;
  }

  // Get sample and out datas.
  sample_proto_ = calibration_method_->GetSampleProto();
  output_proto_ = calibration_method_->GetOutputProto();

  // Vantage plot datas.
  switch (method_) {
    case Method::IDLE:
      PlotIdleProto(chart);
      break;
    case Method::SLIDE:
      PlotSlideProto(chart);
      break;
    case Method::FORCE:
      PlotThrottleProto(chart);
      PlotBrakeProto(chart);
      break;
  }
  return true;
}

void CalibrationTools::SaveData() {
  if (is_save_sampledata_) {
    if (!file_util::ProtoToTextFile(sample_proto_, filename_sample_)) {
      LOG(ERROR) << "[Calibration]: Save sample data failed !";
    }
  }

  if (method_ == Method::FORCE) {
    if (output_proto_.has_idle_v_a_plf()) {
      const auto idle_proto = AddReverseIdleProto(output_proto_);
      output_proto_.mutable_idle_v_a_plf()->CopyFrom(idle_proto.idle_v_a_plf());
    }

    if (output_proto_.has_a_throttle_plf()) {
      const auto throttle_proto = AddReverseThrottleProto(output_proto_);
      output_proto_.mutable_a_throttle_plf()->CopyFrom(
          throttle_proto.a_throttle_plf());
    }

    if (output_proto_.has_a_brake_plf()) {
      const auto brake_proto = AddReverseBrakeProto(output_proto_);
      output_proto_.mutable_a_brake_plf()->CopyFrom(brake_proto.a_brake_plf());
    }
  }

  if (!file_util::ProtoToTextFile(output_proto_, filename_output_)) {
    LOG(ERROR) << "[Calibration]: Save output data failed !";
  }

  LOG(INFO) << "[Calibration]: Save calibration data success !";
}

CalibrationProto CalibrationTools::AddReverseIdleProto(
    const CalibrationProto &in_proto) {
  CalibrationProto out_proto;
  auto idle_v_a_proto = out_proto.mutable_idle_v_a_plf();
  const auto idle_v_a_plf =
      PiecewiseLinearFunctionFromProto(in_proto.idle_v_a_plf());
  idle_v_a_proto->add_x(-3.0);
  idle_v_a_proto->add_y(-PrecisionTruncation(idle_v_a_plf.Evaluate(3.0), 3));
  idle_v_a_proto->add_x(-1.5);
  idle_v_a_proto->add_y(-PrecisionTruncation(idle_v_a_plf.Evaluate(1.5), 3));
  idle_v_a_proto->add_x(-0.01);
  idle_v_a_proto->add_y(-PrecisionTruncation(idle_v_a_plf.Evaluate(0.01), 3));
  for (int i = 0; i < in_proto.idle_v_a_plf().x_size(); i++) {
    idle_v_a_proto->add_x(in_proto.idle_v_a_plf().x().Get(i));
    idle_v_a_proto->add_y(in_proto.idle_v_a_plf().y().Get(i));
  }
  return out_proto;
}

CalibrationProto CalibrationTools::AddReverseThrottleProto(
    const CalibrationProto &in_proto) {
  CalibrationProto out_proto;
  auto a_throttle_proto = out_proto.mutable_a_throttle_plf();
  const auto a_throttle_plf =
      PiecewiseLinearFunctionFromProto(in_proto.a_throttle_plf());
  for (int i = in_proto.a_throttle_plf().x_size() - 1; i > 0; i--) {
    const auto x = in_proto.a_throttle_plf().x().Get(i);
    if (x != 0.0) {
      a_throttle_proto->add_x(-in_proto.a_throttle_plf().x().Get(i));
      a_throttle_proto->add_y(in_proto.a_throttle_plf().y().Get(i));
    }
  }
  a_throttle_proto->add_x(0.0);
  a_throttle_proto->add_y(0.0);
  for (int i = 0; i < in_proto.a_throttle_plf().x_size(); i++) {
    const auto x = in_proto.a_throttle_plf().x().Get(i);
    if (x != 0.0) {
      a_throttle_proto->add_x(in_proto.a_throttle_plf().x().Get(i));
      a_throttle_proto->add_y(in_proto.a_throttle_plf().y().Get(i));
    }
  }
  return out_proto;
}

CalibrationProto CalibrationTools::AddReverseBrakeProto(
    const CalibrationProto &in_proto) {
  CalibrationProto out_proto;
  auto a_brake_proto = out_proto.mutable_a_brake_plf();
  const auto a_brake_plf =
      PiecewiseLinearFunctionFromProto(in_proto.a_brake_plf());
  for (int i = 0; i < in_proto.a_brake_plf().x_size(); i++) {
    const auto x = in_proto.a_brake_plf().x().Get(i);
    if (x != 0.0) {
      a_brake_proto->add_x(in_proto.a_brake_plf().x().Get(i));
      a_brake_proto->add_y(in_proto.a_brake_plf().y().Get(i));
    }
  }
  a_brake_proto->add_x(0.0);
  a_brake_proto->add_y(0.0);
  for (int i = in_proto.a_brake_plf().x_size() - 1; i > 0; i--) {
    const auto x = in_proto.a_brake_plf().x().Get(i);
    if (x != 0.0) {
      a_brake_proto->add_x(-in_proto.a_brake_plf().x().Get(i));
      a_brake_proto->add_y(in_proto.a_brake_plf().y().Get(i));
    }
  }
  return out_proto;
}

void CalibrationTools::PlotIdleProto(vis::vantage::ChartDataProto *chart) {
  if (!sample_proto_.has_idle_v_a_plf() && !output_proto_.has_idle_v_a_plf()) {
    return;
  }
  chart->set_title("control_calibration");
  const auto plf_v_sample = sample_proto_.idle_v_a_plf().x();
  const auto plf_a_sample = sample_proto_.idle_v_a_plf().y();
  std::vector<double> v_sample(plf_v_sample.begin(), plf_v_sample.end());
  std::vector<std::vector<double>> a_sample = {
      std::vector<double>(plf_a_sample.begin(), plf_a_sample.end())};
  ExportProtoToSubchart(v_sample, a_sample, "speed(m/s)", {"acc_sample(m/s2)"},
                        vis::Color::kOrange,
                        vis::vantage::ChartSeriesDataProto::DOTLINE, chart);

  const auto plf_v_out = output_proto_.idle_v_a_plf().x();
  const auto plf_a_out = output_proto_.idle_v_a_plf().y();
  std::vector<double> v_out(plf_v_out.begin(), plf_v_out.end());
  std::vector<double> a_out(plf_a_out.begin(), plf_a_out.end());
  ExportProtoToSubchart(v_out, {a_out}, "speed(m/s)", {"acc(m/s2)"},
                        vis::Color::kDarkGreen,
                        vis::vantage::ChartSeriesDataProto::SOLIDLINE, chart);
}

void CalibrationTools::PlotSlideProto(vis::vantage::ChartDataProto *chart) {
  if (!sample_proto_.has_idle_v_a_plf() && !output_proto_.has_idle_v_a_plf()) {
    return;
  }
  chart->set_title("control_calibration");
  const auto plf_v_sample = sample_proto_.idle_v_a_plf().x();
  const auto plf_a_sample = sample_proto_.idle_v_a_plf().y();
  std::vector<double> v_sample(plf_v_sample.begin(), plf_v_sample.end());
  std::vector<double> a_sample(plf_a_sample.begin(), plf_a_sample.end());
  ExportProtoToSubchart(v_sample, {a_sample}, "speed(m/s)",
                        {"acc_sample(m/s2)"}, vis::Color::kOrange,
                        vis::vantage::ChartSeriesDataProto::DOTLINE, chart);

  const auto plf_v_out = output_proto_.idle_v_a_plf().x();
  const auto plf_a_out = output_proto_.idle_v_a_plf().y();
  std::vector<double> v_out(plf_v_out.begin(), plf_v_out.end());
  std::vector<double> a_out(plf_a_out.begin(), plf_a_out.end());
  ExportProtoToSubchart(v_out, {a_out}, "speed(m/s)", {"acc(m/s2)"},
                        vis::Color::kDarkGreen,
                        vis::vantage::ChartSeriesDataProto::SOLIDLINE, chart);
}

void CalibrationTools::PlotThrottleProto(vis::vantage::ChartDataProto *chart) {
  if (!sample_proto_.has_a_throttle_plf() &&
      !output_proto_.has_a_throttle_plf()) {
    return;
  }
  chart->set_title("control_calibration");
  const auto plf_a_sample = sample_proto_.a_throttle_plf().x();
  const auto plf_throttle_sample = sample_proto_.a_throttle_plf().y();
  std::vector<double> a_sample(plf_a_sample.begin(), plf_a_sample.end());
  std::vector<double> throttle_sample(plf_throttle_sample.begin(),
                                      plf_throttle_sample.end());
  ExportProtoToSubchart(a_sample, {throttle_sample}, "acc(m/s2)",
                        {"throttle_sample(%)"}, vis::Color::kOrange,
                        vis::vantage::ChartSeriesDataProto::DOTLINE, chart);

  const auto plf_a_out = output_proto_.a_throttle_plf().x();
  const auto plf_throttle_out = output_proto_.a_throttle_plf().y();
  std::vector<double> a_out(plf_a_out.begin(), plf_a_out.end());
  std::vector<double> throttle_out(plf_throttle_out.begin(),
                                   plf_throttle_out.end());
  ExportProtoToSubchart(a_out, {throttle_out}, "acc(m/s2)", {"throttle(%)"},
                        vis::Color::kDarkGreen,
                        vis::vantage::ChartSeriesDataProto::SOLIDLINE, chart);
}

void CalibrationTools::PlotBrakeProto(vis::vantage::ChartDataProto *chart) {
  if (!sample_proto_.has_a_brake_plf() && !output_proto_.has_a_brake_plf()) {
    return;
  }
  chart->set_title("control_calibration");
  const auto plf_a_sample = sample_proto_.a_brake_plf().x();
  const auto plf_brake_sample = sample_proto_.a_brake_plf().y();
  std::vector<double> a_sample(plf_a_sample.begin(), plf_a_sample.end());
  std::vector<double> brake_sample(plf_brake_sample.begin(),
                                   plf_brake_sample.end());
  ExportProtoToSubchart(a_sample, {brake_sample}, "acc(m/s2)",
                        {"brake_sample(%)"}, vis::Color::kBlue,
                        vis::vantage::ChartSeriesDataProto::DOTLINE, chart);

  const auto plf_a_out = output_proto_.a_brake_plf().x();
  const auto plf_brake_out = output_proto_.a_brake_plf().y();
  std::vector<double> a_out(plf_a_out.begin(), plf_a_out.end());
  std::vector<double> brake_out(plf_brake_out.begin(), plf_brake_out.end());
  ExportProtoToSubchart(a_out, {brake_out}, "acc(m/s2)", {"brake(%)"},
                        vis::Color::kCyan,
                        vis::vantage::ChartSeriesDataProto::SOLIDLINE, chart);
}

}  // namespace calibration
}  // namespace control
}  // namespace qcraft
