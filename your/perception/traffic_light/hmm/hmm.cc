#include "onboard/perception/traffic_light/hmm/hmm.h"

#include <iomanip>
#include <limits>

#include "onboard/lite/check.h"

namespace qcraft {

DEFINE_bool(print_hmm_viterbi, false,
            "Whether to print out viterbi and path matrix");

namespace {
HMM::StartProbVector GetStartProbVector() {
  // R, Y, G, R_F, Y_F, G_F, I
  static constexpr double start_probs[] = {
      0.3, 0.3, 0.3, 0.01, 0.01, 0.01, 0.1  // NOLINT
  };
  return HMM::StartProbVector(start_probs);
}

HMM::TransitionProbMatrix GetTransitionProbMatrix() {
  // R, Y, G, R_F, Y_F, G_F, I
  static constexpr double transition_probs[] = {
      0.997, 1e-6,  0.01,  1e-6,  1e-6,  1e-6,   1e-5,   // NOLINT
      0.016, 0.975, 1e-6,  1e-6,  0.008, 1e-6,   1e-5,   // NOLINT
      0.001, 0.01,  0.995, 1e-6,  1e-6,  0.0035, 1e-5,   // NOLINT
      1e-6,  1e-6,  1e-6,  0.997, 1e-6,  0.002,  1e-5,   // NOLINT
      0.008, 0.002, 1e-6,  0.001, 0.988, 1e-6,   1e-5,   // NOLINT
      1e-6,  0.009, 0.003, 1e-6,  1e-6,  0.991,  1e-5,   // NOLINT
      0.046, 0.002, 0.034, 0.003, 0.003, 0.004,  0.811,  // NOLINT
  };
  return HMM::TransitionProbMatrix(transition_probs);
}

HMM::EmissionProbMatrix GetEmissionProbMatrix() {
  // R, Y, G, R_F, Y_F, G_F, I
  // R, Y, G, I
  static constexpr double emission_probs[] = {
      0.961, 0.011, 0.005, 0.025,  // NOLINT
      0.040, 0.816, 0.038, 0.106,  // NOLINT
      0.022, 0.002, 0.965, 0.025,  // NOLINT
      0.704, 0.004, 0.002, 0.290,  // NOLINT
      0.007, 0.544, 0.010, 0.440,  // NOLINT
      0.017, 0.012, 0.635, 0.356,  // NOLINT
      0.043, 0.019, 0.033, 0.905,  // NOLINT
  };
  return HMM::EmissionProbMatrix(emission_probs);
}

void PrintInfos(const HMM::Observations& observations,
                const std::vector<std::vector<double>>& viterbi,
                const std::vector<std::vector<int>>& path,
                const HMM::States& states,
                const std::vector<double>& state_scores) {
  const int num_states = viterbi.size();
  const int num_steps = observations.size();
  std::cout << "----------- observations -----------" << std::endl;
  for (int t = 0; t < num_steps; ++t) {
    std::cout << static_cast<int>(observations[t]) << " ";
  }
  std::cout << std::endl;
  std::cout << "----------- viterbi matrix -----------" << std::endl;
  for (int s = 0; s < num_states; ++s) {
    for (int t = 0; t < num_steps; ++t) {
      std::cout << std::fixed << std::setprecision(5) << viterbi[s][t] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << "----------- path -----------" << std::endl;
  for (int s = 0; s < num_states; ++s) {
    for (int t = 0; t < num_steps; ++t) {
      std::cout << path[s][t] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << "----------- output state -----------" << std::endl;
  for (const auto state : states) {
    std::cout << static_cast<int>(state) << " ";
  }
  std::cout << std::endl;
  std::cout << "----------- output state output -----------" << std::endl;
  for (const auto score : state_scores) {
    std::cout << score << " ";
  }
  std::cout << std::endl;
}
}  // namespace

HMM::HMM()
    : start_prob_vector_(GetStartProbVector()),
      transition_prob_matrix_(GetTransitionProbMatrix()),
      emission_prob_matrix_(GetEmissionProbMatrix()) {}

HMM::HMM(const StartProbVector& start_prob_vector,
         const TransitionProbMatrix& transition_prob_matrix,
         const EmissionProbMatrix& emission_prob_matrix)
    : start_prob_vector_(start_prob_vector),
      transition_prob_matrix_(transition_prob_matrix),
      emission_prob_matrix_(emission_prob_matrix) {}

std::pair<HMM::States, std::vector<double>> HMM::Decode(
    const HMM::Observations& observations) {
  const int num_states = static_cast<int>(State::COUNT);
  const int num_steps = observations.size();
  QCHECK_GT(num_steps, 0);
  std::vector<std::vector<double>> viterbi(num_states,
                                           std::vector<double>(num_steps, 0.));
  std::vector<std::vector<int>> path(num_states,
                                     std::vector<int>(num_steps, -1));
  // Init for the first step.
  for (int s = 0; s < num_states; ++s) {
    viterbi[s][0] = start_prob_vector_(s) *
                    emission_prob_matrix_(s, static_cast<int>(observations[0]));
  }

  // Running through the following steps.
  for (int t = 1; t < num_steps; ++t) {
    for (int s = 0; s < num_states; ++s) {
      double max_prob = -1.;
      int prev_state = -1.;
      for (int s_prev = 0; s_prev < num_states; ++s_prev) {
        double prob =
            viterbi[s_prev][t - 1] * transition_prob_matrix_(s_prev, s) *
            emission_prob_matrix_(s, static_cast<int>(observations[t]));
        if (prob > max_prob) {
          max_prob = prob;
          prev_state = s_prev;
        }
      }
      viterbi[s][t] = max_prob;
      path[s][t] = prev_state;
    }
  }

  // Normalize for each step to retrieve reasonable probability.
  for (int t = 0; t < num_steps; ++t) {
    double accumulated_scores = 0.;
    for (int s = 0; s < num_states; ++s) {
      accumulated_scores += viterbi[s][t];
    }
    if (accumulated_scores == 0.) {
      continue;
    }
    for (int s = 0; s < num_states; ++s) {
      viterbi[s][t] /= accumulated_scores;
    }
  }

  double max_sequence_prob = -1;
  int final_state = -1;
  for (int s = 0; s < num_states; ++s) {
    if (viterbi[s][num_steps - 1] > max_sequence_prob) {
      max_sequence_prob = viterbi[s][num_steps - 1];
      final_state = s;
    }
  }

  // Backtracing to get the reversed selected state indices.
  std::vector<int> state_indices = {final_state};
  for (int t = num_steps - 1; t > 0; --t) {
    state_indices.push_back(path[state_indices.back()][t]);
  }
  QCHECK_EQ(num_steps, state_indices.size());

  // Revert the order and fetch scores.
  States states;
  for (int i = num_steps - 1; i >= 0; --i) {
    states.push_back(static_cast<HMM::State>(state_indices[i]));
  }

  std::vector<double> state_scores;
  for (int i = num_steps - 1; i >= 0; --i) {
    const int t = num_steps - 1 - i;
    state_scores.push_back(viterbi[state_indices[i]][t]);
  }

  if (FLAGS_print_hmm_viterbi) {
    PrintInfos(observations, viterbi, path, states, state_scores);
  }
  return {states, state_scores};
}

}  // namespace qcraft
