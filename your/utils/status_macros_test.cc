#include "onboard/utils/status_macros.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

constexpr int kAnswerToUniverse = 42;

absl::Status CheckAnswer(int val) {
  if (val == kAnswerToUniverse) {
    return absl::OkStatus();
  } else {
    return absl::UnknownError("");
  }
}

absl::StatusOr<int> MaybeGetAnswer(int val) {
  if (val == 42) {
    return val;
  } else {
    return absl::InvalidArgumentError("Not 42");
  }
}

absl::Status TestFunc1(int val) {
  RETURN_IF_ERROR(CheckAnswer(val));
  return absl::OkStatus();
}

absl::Status TestFunc2(int* answer, int val) {
  ASSIGN_OR_RETURN(*answer, MaybeGetAnswer(val));
  return absl::OkStatus();
}

int TestFunc3(int* answer, int val) {
  ASSIGN_OR_RETURN(*answer, MaybeGetAnswer(val), kAnswerToUniverse);
  return kAnswerToUniverse;
}

TEST(StatusMacroTest, TestReturnIfError) {
  auto status = TestFunc1(35);
  EXPECT_TRUE(!status.ok());
  status = TestFunc1(kAnswerToUniverse);
  EXPECT_TRUE(status.ok());
}

TEST(StatusMacroTest, TestAssignOrReturn) {
  int answer = 0;
  auto status = TestFunc2(&answer, 15);
  EXPECT_FALSE(status.ok());

  status = TestFunc2(&answer, kAnswerToUniverse);
  EXPECT_TRUE(status.ok());
  EXPECT_EQ(answer, kAnswerToUniverse);

  answer = 0;
  const auto res = TestFunc3(&answer, 21);
  EXPECT_EQ(res, kAnswerToUniverse);
  EXPECT_EQ(answer, 0);
}

TEST(StatusMacroTest, TestAssignOrDie) {
  ASSIGN_OR_DIE(const int x, MaybeGetAnswer(42));
  EXPECT_EQ(x, 42);
  ASSIGN_OR_DIE(const int y, MaybeGetAnswer(42), "We want 42!");
  EXPECT_EQ(y, 42);
  // Using _ to modify the message, will behave like ostream.
  ASSIGN_OR_DIE(const int z, MaybeGetAnswer(42), _ << "We want 42!");
  EXPECT_EQ(z, 42);

  // // The following should fail.
  // int wrong_code = 41;
  // ASSIGN_OR_DIE(const int a, MaybeGetAnswer(wrong_code));
  // EXPECT_EQ(a, 41);
  // ASSIGN_OR_DIE(const int b, MaybeGetAnswer(wrong_code), "We want 42!");
  // EXPECT_EQ(b, 41);
  // ASSIGN_OR_DIE(const int c, MaybeGetAnswer(wrong_code),
  //                 _ << "Oops, " << _ << "we want 42! Got: " << wrong_code);
  // EXPECT_EQ(c, 41);
}

TEST(StatusMacroTest, TestAssignOrContinue) {
  for (int x = 40; x < 45; ++x) {
    ASSIGN_OR_CONTINUE(const int ans, MaybeGetAnswer(x));
    EXPECT_EQ(ans, 42);
  }
}

}  // namespace
}  // namespace qcraft
