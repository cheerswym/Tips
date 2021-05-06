#ifndef ONBOARD_PLANNER_SELECTOR_COST_FEATURE_BASE_H_
#define ONBOARD_PLANNER_SELECTOR_COST_FEATURE_BASE_H_

#include <string>
#include <utility>
#include <valarray>
#include <vector>

#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/scheduler/scheduler_output.h"

namespace qcraft {
namespace planner {

class CostFeatureBase {
 public:
  using CostVec = std::valarray<double>;

  explicit CostFeatureBase(std::string &&name,
                           std::vector<std::string> &&sub_names, bool is_common)
      : name_(std::move(name)),
        sub_names_(std::move(sub_names)),
        size_(sub_names_.size()),
        is_common_(is_common) {}
  virtual ~CostFeatureBase() {}

  const std::string &name() const { return name_; }
  const std::vector<std::string> &sub_names() const { return sub_names_; }
  int size() const { return size_; }
  bool is_common() const { return is_common_; }
  virtual CostVec ComputeCost(const SchedulerOutput &scheduler_output,
                              const EstPlannerOutput &planner_output,
                              std::vector<std::string> *extra_info) const = 0;

 private:
  std::string name_;
  std::vector<std::string> sub_names_;
  int size_;
  // True if the cost applies for all trajs;
  // false if only applies for comparing trajs from the same start lane.
  bool is_common_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SELECTOR_COST_FEATURE_BASE_H_
