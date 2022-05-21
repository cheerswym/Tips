#include "onboard/prediction/container/model_pool.h"

#include "onboard/prediction/prediction_flags.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr char kProphNetParamKey[] = "prophnet_param";
}  // namespace

ModelPool::ModelPool(const ParamManager& param_manager) {
  if (FLAGS_prediction_enable_ml && FLAGS_prediction_enable_prophnet) {
    RunParamsProtoV2 run_params;
    param_manager.GetRunParams(&run_params);
    NetParam prophnet_param;
    CHECK_OK(param_manager.GetProtoParam(kProphNetParamKey, &prophnet_param));
    prophnet_predictor_ =
        std::make_unique<ProphNetPredictor>(run_params, prophnet_param);
  }
}

}  // namespace prediction
}  // namespace qcraft
