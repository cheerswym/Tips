#ifndef ONBOARD_PERCEPTION_TRACK_CLASSIFIER_TCN_CONSTANT_H_
#define ONBOARD_PERCEPTION_TRACK_CLASSIFIER_TCN_CONSTANT_H_

#include <array>

namespace qcraft::tracker {
namespace tcnnet {
static constexpr int kImageWidth = 112;
static constexpr int kImageHeight = 112;
static constexpr unsigned kRandomSeed = 1024666;
}  // namespace tcnnet
}  // namespace qcraft::tracker
#endif  // ONBOARD_PERCEPTION_TRACK_CLASSIFIER_TCN_CONSTANT_H_
