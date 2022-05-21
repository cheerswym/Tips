#include "onboard/prediction/prediction_flags.h"

#include <math.h>

namespace qcraft {
namespace prediction {

DEFINE_double(prediction_duration, 10.0,  // s
              "Prediction seconds for non-stationary trajectory");

DEFINE_bool(
    prediction_enable_ped_traj_cutoff_at_curb, true,
    "switch to enable termination of ped trajectory when crossing the curb.");

// machine learning model
DEFINE_bool(prediction_enable_ml, true, "Enable ml motion predictor.");
DEFINE_bool(prediction_enable_prophnet, true, "Enable prophnet predictor.");
DEFINE_int32(prediction_prophnet_predict_number_threshold, 63,
             "The number of objects that will be predicted by prophnet");
DEFINE_int32(prediction_tnt_predict_number_threshold, 64,
             "The number of objects that will be predicted by tnt");

// data dumping
DEFINE_bool(prediction_dumping_model_feature, false,
            "Whether to dump model input feature to prediction debug");
// conflict resolver
DEFINE_bool(prediction_conflict_resolver_on, true,
            "Conflict resolver module on/off");
DEFINE_bool(prediction_conflict_resolver_visual_on, false,
            "Conflict resolver module debug mode on/off");

}  // namespace prediction
}  // namespace qcraft
