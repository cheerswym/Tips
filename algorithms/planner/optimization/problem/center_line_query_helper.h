#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_CENTER_LINE_QUERY_HELPER_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_CENTER_LINE_QUERY_HELPER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "onboard/math/eigen.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/optimization/problem/cost_helper.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class CenterLineQueryHelper : public CostHelper<PROB> {
 public:
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  CenterLineQueryHelper(const std::vector<Vec2d> &points, std::string name,
                        bool use_qtfm)
      : CostHelper<PROB>(std::move(name)), points_(&points) {
    QCHECK_GT(points.size(), 1);
    if (use_qtfm) {
      path_ = std::make_unique<QtfmEnhancedKdTreeFrenetFrame>(
          BuildQtfmEnhancedKdTreeFrenetFrame(points).value());
    } else {
      path_ = std::make_unique<KdTreeFrenetFrame>(
          BuildKdTreeFrenetFrame(points).value());
    }
  }

  const std::array<FrenetCoordinate, PROB::kHorizon> &s_l_list() const {
    return s_l_list_;
  }
  const std::array<Vec2d, PROB::kHorizon> &normals() const { return normals_; }
  const std::array<std::pair<int, int>, PROB::kHorizon> &index_pairs() const {
    return index_pairs_;
  }
  const std::array<double, PROB::kHorizon> &alphas() const { return alphas_; }
  const std::vector<double> &s_knots() const { return path_->s_knots(); }
  const std::vector<Vec2d> tangents() const { return path_->tangents(); }
  const std::vector<Vec2d> *points() const { return points_; }

  void Update(const StatesType &xs, const ControlsType &us) override {
    for (int k = 0; k < PROB::kHorizon; ++k) {
      const Vec2d pos = PROB::StateGetPos(PROB::GetStateAtStep(xs, k));
      path_->XYToSL(pos, &s_l_list_[k], &normals_[k], &index_pairs_[k],
                    &alphas_[k]);
    }
  }

  const FrenetFrame &path() const {
    QCHECK_NOTNULL(path_);
    return *path_;
  }

 private:
  std::string name_ = "";
  std::unique_ptr<FrenetFrame> path_;
  const std::vector<Vec2d> *points_;

  // Frenet xy to sl query results.
  std::array<FrenetCoordinate, PROB::kHorizon> s_l_list_;
  std::array<Vec2d, PROB::kHorizon> normals_;
  std::array<std::pair<int, int>, PROB::kHorizon> index_pairs_;
  std::array<double, PROB::kHorizon> alphas_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_CENTER_LINE_QUERY_HELPER_H_
