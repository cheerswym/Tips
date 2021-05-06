#include "onboard/planner/test_util/planner_test_setup.h"

#include <string>

namespace qcraft {
namespace planner {

void PlannerTestSetup::set_map(std::string_view map_name) {
  QCHECK_EQ(map_name, "dojo") << "Only dojo map is supported now";
  SetMap(std::string(map_name));
  map_.LoadWholeMap().Build();
}

}  // namespace planner
}  // namespace qcraft
