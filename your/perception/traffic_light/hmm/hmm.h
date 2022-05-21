#ifndef ONBOARD_PERCEPTION_TRAFFIC_LIGHT_HMM_HMM_H
#define ONBOARD_PERCEPTION_TRAFFIC_LIGHT_HMM_HMM_H

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "onboard/math/eigen.h"

namespace qcraft {

class HMM {
 public:
  enum class Observation {
    RED = 0,
    YELLOW,
    GREEN,
    INACTIVE_OR_UNKNOWN,
    COUNT,
  };
  using Observations = std::vector<Observation>;

  enum class State {
    RED = 0,
    YELLOW,
    GREEN,
    RED_FLASHING,
    YELLOW_FLASHING,
    GREEN_FLASHING,
    INACTIVE_OR_UNKNOWN,
    COUNT,
  };
  using States = std::vector<State>;
  using StartProbVector =
      Eigen::Matrix<double, static_cast<int>(State::COUNT), 1>;
  using TransitionProbMatrix =
      Eigen::Matrix<double, static_cast<int>(State::COUNT),
                    static_cast<int>(State::COUNT), Eigen::RowMajor>;
  using EmissionProbMatrix =
      Eigen::Matrix<double, static_cast<int>(State::COUNT),
                    static_cast<int>(Observation::COUNT), Eigen::RowMajor>;

  HMM();
  HMM(const StartProbVector& start_prob_vector,
      const TransitionProbMatrix& transition_prob_matrix,
      const EmissionProbMatrix& emission_prob_matrix);

  std::pair<States, std::vector<double>> Decode(
      const Observations& observations);

 private:
  StartProbVector start_prob_vector_;
  TransitionProbMatrix transition_prob_matrix_;
  EmissionProbMatrix emission_prob_matrix_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_TRAFFIC_LIGHT_HMM_HMM_H
