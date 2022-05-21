#ifndef ONBOARD_PREDICTION_CONTAINER_MODEL_POOL_H_
#define ONBOARD_PREDICTION_CONTAINER_MODEL_POOL_H_

#include <memory>

#include "onboard/nets/prophnet_predictor.h"
#include "onboard/params/param_manager.h"

namespace qcraft {
namespace prediction {
class ModelPool {
 public:
  explicit ModelPool(const ParamManager& param_manager);
  const ProphNetPredictor& GetProphNetPredictor() const {
    return *prophnet_predictor_.get();
  }

 private:
  std::unique_ptr<ProphNetPredictor> prophnet_predictor_;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_MODEL_POOL_H_
