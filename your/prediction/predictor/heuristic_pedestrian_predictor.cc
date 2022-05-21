
#include "onboard/prediction/predictor/heuristic_pedestrian_predictor.h"

#include <utility>
#include <vector>

#include "onboard/global/trace.h"
#include "onboard/math/line_fitter.h"
#include "onboard/math/stats.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/util/trajectory_developer.h"
namespace qcraft {
namespace prediction {
namespace {
constexpr double kHistoryTime = 1.0;
constexpr double kFittingExpDecayRate = 0.5;
// Get the speed and std along the line path.
constexpr double kMaxPedestrianSpeed = 12.42;     // m/s.  World record held by
                                                  // Usain Bolt in 2009.
constexpr double kPedestrianNominalBrake = -0.5;  // m/s^2.

}  // namespace
PredictedTrajectory MakeHeuristicPedestrianPrediction(
    const ObjectHistory* obj, const SemanticMapManager& semantic_map_mgr,
    ObjectPredictionPriority priority) {
  SCOPED_QTRACE("HeuristicPedestrianPrediction");
  double last_t = obj->timestamp();
  const auto hist_or = obj->GetHistoryFrom(last_t - kHistoryTime);
  QCHECK_OK(hist_or.status());
  const auto& hist = hist_or.value();
  std::vector<PredictedTrajectoryPoint> traj_pts;
  PredictedTrajectory predicted_traj;
  if (hist.size() == 1) {
    traj_pts = DevelopCYCVTrajectory(hist.back().val, kPredictionTimeStep,
                                     kEmergencyGuardHorizon,
                                     /*is_reversed=*/false);
    predicted_traj = PredictedTrajectory(
        /*probability=*/1.0, priority, "heuristic ped prediction: cvch",
        PredictionType::PT_CYCV, 1, std::move(traj_pts),
        /*is_reversed=*/false);
  } else {
    std::vector<Vec2d> vec_pos;
    vec_pos.reserve(hist.size());
    std::vector<double> weights;
    weights.reserve(hist.size());
    for (const auto& ele : hist) {
      const double dt = last_t - ele.time;
      vec_pos.push_back(ele.val.pos());
      weights.push_back(std::pow(kFittingExpDecayRate, dt));
    }
    LineFitter fitter(vec_pos, weights, /*debug=*/false);
    fitter.FitData(FITTER::DEMING, /*normalize=*/true, /*compute_mse=*/true);
    const Vec2d fitter_tangent = fitter.tangent();
    const double line_angle = fitter_tangent.FastAngle();
    const Vec2d vec_vel = hist.back().val.vec_vel();
    double fitted_direction = (fitter_tangent.dot(vec_vel)) >= 0
                                  ? line_angle
                                  : NormalizeAngle(M_PI + line_angle);
    double clamped_speed =
        std::clamp(hist.back().val.v(), 0.0, kMaxPedestrianSpeed);
    // Force speed to zero if object is stationary.
    if (hist.back().val.IsStationary()) {
      clamped_speed = 0.0;
    }
    UniCycleState uni_state{.x = hist.pos().x(),
                            .y = hist.pos().y(),
                            .v = clamped_speed,
                            .heading = fitted_direction,
                            .yaw_rate = 0.0,
                            .acc = 0.0};
    traj_pts =
        DevelopCYCVTrajectory(uni_state, kPredictionTimeStep,
                              kPredictionDuration, /*is_reversed=*/false);
    predicted_traj = PredictedTrajectory(
        /*probability=*/1.0, priority, "heuristic ped prediction",
        PredictionType::PT_PED_KINEMATIC, 1, std::move(traj_pts),
        /*is_reversed=*/false);
  }
  // Check if termination is needed. This assumes that pedestrians are rational
  // and will not behave in aggressive manners such that collision with the AV
  // is possible.
  if (FLAGS_prediction_enable_ped_traj_cutoff_at_curb) {
    const double object_length = hist.bounding_box().length();
    const auto curb_collision_idxes = FindTrajCurbCollisionIndex(
        predicted_traj.points(), semantic_map_mgr, object_length,
        qcraft::ObjectType::OT_PEDESTRIAN);

    CutoffTrajByCurbCollisionIndexes(curb_collision_idxes,
                                     kPedestrianNominalBrake, kSafeHorizon,
                                     kComfortableHorizon, &predicted_traj);
  }

  return predicted_traj;
}
}  // namespace prediction
}  // namespace qcraft
