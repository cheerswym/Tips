#include "onboard/perception/tracker/motion_static_classifier.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/test_util/track_builder.h"

namespace qcraft::tracker {
namespace motion_static_classifier {

TEST(MotionStaticClassifier, TestDebugToString) {
  EXPECT_EQ(MotionStaticClassifier::DebugToString(
                MotionStaticClassifier::DebugState::kIsStaticByType),
            "kIsStaticByType");
}

}  // namespace motion_static_classifier
}  // namespace qcraft::tracker

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
