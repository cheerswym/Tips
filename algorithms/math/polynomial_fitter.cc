#include "onboard/math/polynomial_fitter.h"

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/stats.h"

namespace qcraft {

void PolynomialFitter::CheckData(bool check_data_size) const {
  const int nd = data_.size();
  const int nw = weights_.size();
  CHECK(nw == 0 || nw == nd);
  for (const double& w : weights_) {
    CHECK_GE(w, 0.0);
  }

  if (check_data_size) CHECK_GT(data_.size(), degree_);
}

double PolynomialFitter::Evaluate(double x) const {
  double r = 0.0;
  for (int i = degree_; i >= 0; --i) {
    r = r * x + beta_[i];
  }
  return r;
}

void PolynomialFitter::FitData(LS_SOLVER solver, bool compute_mse) {
  const int num_data = data_.size();
  Eigen::MatrixXd X(num_data, degree_ + 1);
  Eigen::MatrixXd Y(num_data, 1);
  for (std::size_t i = 0; i < num_data; ++i) {
    const double weight = weights_.size() == num_data ? sqrt(weights_[i]) : 1.0;

    const double x = data_[i].x();
    const double y = data_[i].y();

    X(i, 0) = weight;
    for (std::size_t j = 1; j < degree_ + 1; ++j) {
      X(i, j) = X(i, j - 1) * x;
    }

    Y(i, 0) = weight * y;
  }

  Eigen::MatrixXd beta;
  if (solver == LS_SOLVER::SVD) {
    beta = X.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
  } else if (solver == LS_SOLVER::QR) {
    beta = X.colPivHouseholderQr().solve(Y);
  } else if (solver == LS_SOLVER::PINV) {
    beta = (X.transpose() * X).ldlt().solve(X.transpose() * Y);
  } else {
    QLOG(FATAL) << absl::StrFormat(
        "  Jesus Christ! You just invented another least-square sovler! Please "
        "add it as an option for me.");
  }

  beta_.clear();
  beta_.reserve(degree_ + 1);
  for (std::size_t i = 0; i < degree_ + 1; ++i) {
    beta_.push_back(beta(i, 0));
  }

  if (compute_mse) {
    mse_ =
        std::accumulate(data_.begin(), data_.end(), 0.0,
                        [this](const double& sum, const Vec2d& sample) {
                          return sum + Sqr(Evaluate(sample.x()) - sample.y());
                        }) /
        data_.size();
  }
}

void PolynomialFitter::PrintDebugInfo() const {
  VLOG(0) << "******** polynomial fitter ********";
  VLOG(0) << " polynomial fitter data: " << VecOfVec2dToString(data_);
  VLOG(0) << " polynomial fitter data weight: "
          << VecOfRealNumbersToString(weights_);
  VLOG(0) << " polynomial fitter coefficients: "
          << VecOfRealNumbersToString(beta_);
  VLOG(0) << " fitting mse: " << mse_;
  VLOG(0) << "******** polynomial fitter result ********";
}

}  // namespace qcraft
