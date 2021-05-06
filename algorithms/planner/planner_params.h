#ifndef ONBOARD_PLANNER_PLANNER_PARAMS_H_
#define ONBOARD_PLANNER_PLANNER_PARAMS_H_

#include <string>

#include "onboard/base/macros.h"
#include "onboard/global/singleton.h"
#include "onboard/maps/map_selector.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/proto/planner_params.pb.h"

namespace qcraft {
namespace planner {

// Loads planner params according to the situation (onboard run, sim, locale,
// car ID etc). Also fills in param fields missing from the loaded param file
// with those from the default params (the default params also depend on the
// situation), and validates that all fields have been filled.
// The PlannerParamsProto returned here is guaranteed to have no unset field.
class PlannerParams {
 public:
  void Init(const RunParamsProtoV2 &run_params);
  // TODO(lidong): This setting will cause data diverge in code. We should clean
  // it up.
  // Accessors. Note that planner_params() does not necessarily equal
  // run_params().vehicle_params().planner_params() because the former may have
  // been completed by the default.
  const PlannerParamsProto &planner_params() const { return planner_params_; }
  const RunParamsProtoV2 &run_params() const { return run_params_; }

  static const PlannerParamsProto &Get() {
    return Instance()->planner_params();
  }
  static const RunParamsProtoV2 &GetRunParams() {
    return Instance()->run_params();
  }

  enum class EnvironmentClass {
    kChinaUrban,
    kChinaSuburban,
    kUsSuburban,
  };

  static bool IsBus();
  static EnvironmentClass EnvironmentClassFromRunParams(
      const RunParamsProtoV2 &run_params);

 protected:
  // Protests if any field is missing.
  static void ValidateParams(const google::protobuf::Message &params);

 private:
  PlannerParamsProto planner_params_;
  RunParamsProtoV2 run_params_;

  DECLARE_SINGLETON(PlannerParams);
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLANNER_PARAMS_H_
