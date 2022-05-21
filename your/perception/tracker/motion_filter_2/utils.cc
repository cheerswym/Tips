#include "onboard/perception/tracker/motion_filter_2/utils.h"

namespace qcraft::tracker {

IMMProto ToImmProto(const std::vector<MotionFilterParamProto>& filter_params,
                    const std::vector<double>& weights) {
  if (filter_params.size() != weights.size()) {
    QLOG(FATAL) << "filter_params.size() != weights.size()";
  }

  IMMProto proto;
  for (int i = 0; i < filter_params.size(); ++i) {
    auto* ptr = proto.add_params();
    *ptr->mutable_filter_param() = filter_params[i];
    ptr->set_filter_weight(weights[i]);
  }

  return proto;
}

}  // namespace qcraft::tracker
