#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "gtest/gtest.h"
#include "onboard/lite/check.h"
namespace qcraft {
namespace {

int getvalue() { return 1; }

constexpr char absl_status_error_message[] = "False absl status message.";
constexpr char absl_statusor_error_message[] = "False absl statusor message.";

absl::Status getAbslStatus(bool ok) {
  static int number = 0;
  number++;
  EXPECT_LT(number, 3);
  if (!ok) {
    return absl::InternalError(absl_status_error_message);
  }
  return absl::OkStatus();
}

absl::StatusOr<int> getAbslStatusOr(bool ok) {
  static int number = 0;
  number++;
  EXPECT_LT(number, 3);
  if (!ok) {
    return absl::InternalError(absl_statusor_error_message);
  }
  return number;
}

TEST(QCHECKTest, TestErrorWithOnboard) {
  gflags::SetCommandLineOption("q_run_mode", "onboard");
  qcraft::BufferedLogger::Instance()->SetFatalHandler(
      [](const qcraft::LogItem& log_item) {});

  QCHECK_EQ(getvalue(), 2) << "failure EQ ";

  QCHECK_OK(getAbslStatus(true));
  QCHECK_OK(getAbslStatus(false));

  QCHECK_OK(getAbslStatusOr(true).status());
  QCHECK_OK(getAbslStatusOr(false).status());

  const auto proto = qcraft::BufferedLogger::Instance()->DumpLogProto("empty");
  EXPECT_EQ(3, proto.log_item_size());
  auto item = proto.log_item(0);
  EXPECT_EQ(FATAL, item.severity());
  item = proto.log_item(1);
  EXPECT_NE(-1, item.msg().find(absl_status_error_message));
  EXPECT_EQ("onboard/lite/qcheck_test.cc", item.file_name());
  EXPECT_EQ(41, item.file_line());

  item = proto.log_item(2);
  EXPECT_EQ(FATAL, item.severity());
  EXPECT_NE(-1, item.msg().find(absl_statusor_error_message));
  EXPECT_EQ(44, item.file_line());

  // Unable to compare stack trace or capture time, since it is not immutable.
}
}  // namespace
}  // namespace qcraft
