#include "onboard/perception/traffic_light/hmm/hmm.h"

#include "gtest/gtest.h"
#include "onboard/global/logging.h"

namespace qcraft {

HMM::StartProbVector GetTestStartProbVector() {
  // R, Y, G, R_F, Y_F, G_F, I
  static constexpr double start_probs[] = {
      0.3, 0.3, 0.3, 0.01, 0.01, 0.01, 0.1  // NOLINT
  };
  return HMM::StartProbVector(start_probs);
}

HMM::TransitionProbMatrix GetTestTransitionProbMatrix() {
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

HMM::EmissionProbMatrix GetTestEmissionProbMatrix() {
  // R, Y, G, R_F, Y_F, G_F, I
  // R, Y, G, I
  static constexpr double emission_probs[] = {
      0.961, 0.011, 0.005, 0.023,  // NOLINT
      0.040, 0.816, 0.038, 0.106,  // NOLINT
      0.022, 0.002, 0.965, 0.011,  // NOLINT
      0.704, 0.004, 0.002, 0.290,  // NOLINT
      0.007, 0.544, 0.010, 0.440,  // NOLINT
      0.017, 0.012, 0.635, 0.356,  // NOLINT
      0.043, 0.019, 0.033, 0.905,  // NOLINT
  };
  return HMM::EmissionProbMatrix(emission_probs);
}

TEST(HmmTest, ExpectCorrectResult1) {
  HMM hmm(GetTestStartProbVector(), GetTestTransitionProbMatrix(),
          GetTestEmissionProbMatrix());
  HMM::Observations observations = {HMM::Observation::GREEN,
                                    HMM::Observation::GREEN,
                                    HMM::Observation::GREEN,
                                    HMM::Observation::GREEN,
                                    HMM::Observation::GREEN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN,
                                    HMM::Observation::GREEN,
                                    HMM::Observation::GREEN,
                                    HMM::Observation::GREEN,
                                    HMM::Observation::GREEN,
                                    HMM::Observation::GREEN};
  HMM::States states;
  std::vector<double> scores;
  std::tie(states, scores) = hmm.Decode(observations);
  EXPECT_EQ(states.back(), HMM::State::GREEN_FLASHING);
}

TEST(HmmTest, ExpectCorrectResult2) {
  HMM hmm(GetTestStartProbVector(), GetTestTransitionProbMatrix(),
          GetTestEmissionProbMatrix());
  HMM::Observations observations = {
      HMM::Observation::GREEN, HMM::Observation::GREEN, HMM::Observation::GREEN,
      HMM::Observation::GREEN, HMM::Observation::GREEN, HMM::Observation::RED,
      HMM::Observation::RED,   HMM::Observation::RED,   HMM::Observation::RED,
      HMM::Observation::RED};
  HMM::States states;
  std::vector<double> scores;
  std::tie(states, scores) = hmm.Decode(observations);
  EXPECT_EQ(states.back(), HMM::State::RED);
}

TEST(HmmTest, ExpectCorrectResult3) {
  HMM hmm(GetTestStartProbVector(), GetTestTransitionProbMatrix(),
          GetTestEmissionProbMatrix());
  HMM::Observations observations = {HMM::Observation::GREEN,
                                    HMM::Observation::GREEN,
                                    HMM::Observation::GREEN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN,
                                    HMM::Observation::INACTIVE_OR_UNKNOWN};
  HMM::States states;
  std::vector<double> scores;
  std::tie(states, scores) = hmm.Decode(observations);
  EXPECT_EQ(states.back(), HMM::State::INACTIVE_OR_UNKNOWN);
}

}  // namespace qcraft
