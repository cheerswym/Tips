#ifndef ONBOARD_PREDICTION_CONFLICT_RESOLVER_CONFLICT_RESOLVER_PARAMS_H_
#define ONBOARD_PREDICTION_CONFLICT_RESOLVER_CONFLICT_RESOLVER_PARAMS_H_

#include "onboard/lite/logging.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/utils/file_util.h"

namespace qcraft {
namespace prediction {
// Load conflict resolver params when planner init.

class ConflictResolverParams {
  // TODO(changqing): load params when init planner.
 public:
  void LoadParams();
  const ConflictResolutionConfigProto& config_params() const {
    return config_params_;
  }
  const ConflictResolutionConfigProto::ObjectConflictResolverConfig&
  GetConfigByObjectType(const ObjectType& type) const {
    QCHECK(config_params_.has_default_object_config());
    for (const auto& config : config_params_.object_config()) {
      if (config.object_type() == type) {
        return config;
      }
    }
    return config_params_.default_object_config();
  }
  const ConflictResolutionConfigProto::ConflictResolverConfig&
  GetGeneralConfig() const {
    QCHECK(config_params_.has_general_config());
    return config_params_.general_config();
  }

  const ConflictResolutionConfigProto::SpeedFeatureCostConfig& GetCostConfig()
      const {
    QCHECK(config_params_.has_dp_speed_cost_config());
    return config_params_.dp_speed_cost_config();
  }

 private:
  // TODO(changqing): Change name to ConflictResolverConfigProto later.
  ConflictResolutionConfigProto config_params_;  // Contains sampling params and
                                                 // search params (weights).
};
}  // namespace prediction
}  // namespace qcraft
#endif  // ONBOARD_PREDICTION_CONFLICT_RESOLVER_CONFLICT_RESOLVER_PARAMS_H_
