#include "onboard/planner/planner_params.h"

#include <memory>
#include <vector>

#include "onboard/global/car_common.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/params/param_manager.h"
#include "onboard/utils/proto_util.h"

namespace qcraft {
namespace planner {

namespace {

std::string GetDefaultParamsFile(const RunParamsProtoV2 &run_params) {
  const PlannerParams::EnvironmentClass environment_class =
      PlannerParams::EnvironmentClassFromRunParams(run_params);
  if (environment_class == PlannerParams::EnvironmentClass::kUsSuburban) {
    return "us_params.pb.txt";
  }
  const auto vehicle_params = run_params.vehicle_params();
  // See go/vehicles
  switch (vehicle_params.vehicle_params().model()) {
    case VEHICLE_ZHONGXING:
    case VEHICLE_JINLV_MINIBUS:
    case VEHICLE_HIGER:
    case VEHICLE_ZHONGTONG55:
    case VEHICLE_DONGFENG:
      return "robobus_6m_params.pb.txt";
    case VEHICLE_ZHONGTONG:
    case VEHICLE_ZHONGTONG6:
    case VEHICLE_BYD:
    case VEHICLE_POLERSTAR:
    case VEHICLE_POLERSTAR_2:
      return "robobus_10m_params.pb.txt";
    case VEHICLE_UNKNOWN:
    case VEHICLE_PIXLOOP:
    case VEHICLE_SKYWELL:
    case VEHICLE_TEST_BENCH:
    case VEHICLE_LINCOLN_MKZ:
    case VEHICLE_LINCOLN_MKZ_AS_PACMOD:
    case VEHICLE_AION_LX:
    case VEHICLE_MARVELR:
    case VEHICLE_MARVELX:
      return "planner_default_params.pb.txt";
    // TODO(lidong): We should create a separate config file for
    // VEHICLE_SHUNFENG.
    case VEHICLE_SHUNFENG:
      return "planner_default_params.pb.txt";
  }
}

}  // namespace

bool PlannerParams::IsBus() {
  switch (GetRunParams().vehicle_params().vehicle_params().model()) {
    case VEHICLE_ZHONGXING:
    case VEHICLE_JINLV_MINIBUS:
    case VEHICLE_HIGER:
    case VEHICLE_ZHONGTONG55:
    case VEHICLE_DONGFENG:
    case VEHICLE_ZHONGTONG:
    case VEHICLE_ZHONGTONG6:
    case VEHICLE_BYD:
    case VEHICLE_POLERSTAR:
    case VEHICLE_POLERSTAR_2:
      return true;
    case VEHICLE_UNKNOWN:
    case VEHICLE_PIXLOOP:
    case VEHICLE_SKYWELL:
    case VEHICLE_TEST_BENCH:
    case VEHICLE_LINCOLN_MKZ:
    case VEHICLE_LINCOLN_MKZ_AS_PACMOD:
    case VEHICLE_AION_LX:
    case VEHICLE_MARVELX:
    case VEHICLE_MARVELR:
    case VEHICLE_SHUNFENG:
      return false;
  }
}

PlannerParams::EnvironmentClass PlannerParams::EnvironmentClassFromRunParams(
    const RunParamsProtoV2 &run_params) {
  const Locale locale = run_params.locale_params().locale();
  switch (locale) {
    case LOC_SHENZHEN:
    case LOC_SHANGHAI:
      return EnvironmentClass::kChinaUrban;
    case LOC_HANGZHOU:
    case LOC_SUZHOU:
    case LOC_WUHAN:
    case LOC_JIAXING:
    case LOC_HAIDIAN:
    case LOC_XIXIAN:
    case LOC_LIAOCHENG:
    case LOC_ZHENGZHOU:
    case LOC_CHONGQING:
    case LOC_YUNNAN_DALI:
    case LOC_HUNAN_CHANGSHA:
    case LOC_HEBEI_XIONGAN:
    case LOC_ZHANGZHOU:
      return EnvironmentClass::kChinaSuburban;
    case LOC_DOJO:
      return EnvironmentClass::kChinaUrban;
    case LOC_BAY_AREA:
      return EnvironmentClass::kUsSuburban;
    case LOC_UNKNOWN:
      QLOG(FATAL) << "Unrecognized locale";
      return EnvironmentClass::kChinaUrban;
  }
}

PlannerParams::PlannerParams() {}

void PlannerParams::Init(const RunParamsProtoV2 &run_params) {
  run_params_ = run_params;

  QLOG(INFO) << "Loaded params for: " << run_params_.vehicle_params().car_id();

  // Load and fill in the default where needed.
  PlannerParamsProto run_planner_params;
  file_util::FileToProto(
      "onboard/planner/params/" + GetDefaultParamsFile(run_params_),
      &run_planner_params);
  FillInMissingFieldsWithDefault(run_planner_params, &planner_params_);

  PlannerParamsProto default_planner_params;
  file_util::FileToProto("onboard/planner/params/planner_default_params.pb.txt",
                         &default_planner_params);

  // Fill default speed finder params into default planner params.
  SpeedFinderParamsProto default_speed_finder_params;
  file_util::FileToProto(
      "onboard/planner/params/speed_finder_default_params.pb.txt",
      &default_speed_finder_params);

  // Fill default hybrid_a_star params into default planner params.
  HybridAStarParamsProto default_hybrid_a_star_params;
  file_util::FileToProto(
      "onboard/planner/params/hybrid_a_star_default_params.pb.txt",
      &default_hybrid_a_star_params);

  // Fill default local_smoother params into default planner params.
  FreespaceLocalSmootherParamsProto default_local_smoother_params;
  file_util::FileToProto(
      "onboard/planner/params/local_smoother_default_params.pb.txt",
      &default_local_smoother_params);

  // Fill est planner.
  qcraft::FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_speed_finder_params());

  // Fill freespace planner.
  qcraft::FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_speed_finder_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_hybrid_a_star_params,
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_hybrid_a_star_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_local_smoother_params,
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_local_smoother_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_planner_params.motion_constraint_params(),
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_motion_constraint_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_speed_finder_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_hybrid_a_star_params,
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_hybrid_a_star_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_local_smoother_params,
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_local_smoother_params());
  qcraft::FillInMissingFieldsWithDefault(
      default_planner_params.motion_constraint_params(),
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_motion_constraint_params());

  // Fill fallback planner.
  qcraft::FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_fallback_planner_params()
          ->mutable_speed_finder_params());

  // Fill default planner params.
  FillInMissingFieldsWithDefault(default_planner_params, &planner_params_);

  // Check that all fields are set.
  ValidateParams(planner_params_);
}

// Protests if any field is missing.
void PlannerParams::ValidateParams(const google::protobuf::Message &params) {
  const google::protobuf::Descriptor *descriptor = params.GetDescriptor();
  const google::protobuf::Reflection *reflection = params.GetReflection();
  for (int i = 0; i < descriptor->field_count(); ++i) {
    const google::protobuf::FieldDescriptor *field = descriptor->field(i);
    if (!field->is_optional()) continue;
    QCHECK(reflection->HasField(params, field))
        << "Missing field: " << field->full_name();

    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      ValidateParams(reflection->GetMessage(params, field));
    }
  }
}

}  // namespace planner
}  // namespace qcraft
