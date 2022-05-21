#ifndef ONBOARD_PREDICTION_PREDICTION_FLAGS_H_
#define ONBOARD_PREDICTION_PREDICTION_FLAGS_H_

#include "gflags/gflags.h"

namespace qcraft {
namespace prediction {
// prediction engine
DECLARE_double(prediction_duration);

// ped
DECLARE_bool(prediction_enable_ped_traj_cutoff_at_curb);

// machine learning model
DECLARE_bool(prediction_enable_ml);
DECLARE_bool(prediction_enable_prophnet);
DECLARE_int32(prediction_prophnet_predict_number_threshold);
DECLARE_int32(prediction_tnt_predict_number_threshold);

// data dumping
DECLARE_bool(prediction_dumping_model_feature);

// conflict resolver
DECLARE_bool(prediction_conflict_resolver_on);
DECLARE_bool(prediction_conflict_resolver_visual_on);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTION_FLAGS_H_
