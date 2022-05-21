#include "onboard/utils/source_location.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

// A dummy function that is used to verify that source location can store the
// actual code location of the caller.
SourceLocation TestFunc(SourceLocation loc = QCRAFT_LOC) { return loc; }

TEST(SourceLocation, Works) {
  // Don't change the order of the following two lines.
  const auto one = TestFunc();
  const auto two = TestFunc();

  EXPECT_LT(one.line(), two.line());
  EXPECT_EQ(one.file_name(), two.file_name());
}

}  // namespace
}  // namespace qcraft
