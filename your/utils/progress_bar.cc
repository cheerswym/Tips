#include "onboard/utils/progress_bar.h"

#include <sys/ioctl.h>

#include <iomanip>
#include <string>

namespace qcraft {
namespace {
constexpr double kTotalPercentage = 100.0;
constexpr int kCharWidthPercentage = 4;
}  // namespace

ProgressBar::ProgressBar(uint64 n, const std::string& description,
                         std::ostream& out)
    : n_(n),
      desc_width_(description.size()),
      frequency_update_(n),
      description_(description) {
  unit_bar_ = "=";
  unit_space_ = " ";
  os_pointer_ = &out;
}

ProgressBar::ProgressBar(const ProgressBar& progress_bar) {
  n_ = progress_bar.n_;
  desc_width_ = progress_bar.desc_width_;
  frequency_update_ = progress_bar.frequency_update_;
  os_pointer_ = &(std::clog);
  description_ = progress_bar.description_;
  unit_bar_ = progress_bar.unit_bar_;
  unit_space_ = progress_bar.unit_space_;
}

void ProgressBar::SetFrequencyUpdate(uint64 frequency_update) {
  frequency_update_ = frequency_update > n_ ? n_ : frequency_update;
}

void ProgressBar::SetUpdateGap(uint64 gap) {
  if (n_ == 0) {
    // dummy value
    SetFrequencyUpdate(1);
    return;
  }
  QCHECK_GE(gap, 1);
  if (gap > n_) {
    gap = n_;
  }
  QCHECK_LE(gap, n_);
  SetFrequencyUpdate(n_ / gap);
}

void ProgressBar::SetStyle(const std::string& unit_bar,
                           const std::string& unit_space) {
  unit_bar_ = unit_bar;
  unit_space_ = unit_space;
}

int ProgressBar::GetConsoleWidth() { return 150; }

int ProgressBar::GetBarLength() {
  return static_cast<int>(
      (GetConsoleWidth() - desc_width_ - kCharWidthPercentage) / 2.);
}

void ProgressBar::Progressed(uint64 idx) {
  QCHECK_LE(idx, n_);
  if (idx != n_ && idx % (n_ / frequency_update_) != 0) return;
  const int bar_size = GetBarLength();
  const double progress_percent = idx * kTotalPercentage / n_;
  const double percent_per_unit_bar = kTotalPercentage / bar_size;

  std::ostringstream oss;
  oss << " " << description_ << " [";
  for (int bar_length = 0; bar_length <= bar_size - 1; ++bar_length) {
    if (bar_length * percent_per_unit_bar < progress_percent) {
      oss << unit_bar_;
    } else {
      oss << unit_space_;
    }
  }
  oss << "]" << std::setw(kCharWidthPercentage + 1) << std::setprecision(1)
      << std::fixed << progress_percent << '%';
  if (idx == n_) oss << '\n';
  oss << '\r' << std::flush;
  std::lock_guard<std::mutex> guard(mutex_os_);
  *os_pointer_ << oss.str();
}

}  // namespace qcraft
