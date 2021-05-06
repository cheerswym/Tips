#include "onboard/planner/initializer/dp_cost_feature.h"

#include "onboard/planner/planner_defs.h"
namespace qcraft::planner {
namespace {
// Some cost is time consuming to evaluate while does not need high accuracy,
// evalute every kEvalStep point to reduce time consumption.
constexpr int kEvalStep = 2;
constexpr double kBoundarySoftZone = 0.75;
constexpr double kLateralAccelerationHardLimit = 2.5;  // m/s^2
constexpr double kLateralAccelerationSoftLimit = 1.0;  // m/s^2
const double kInvLateralAccelerationSoftLimit =
    1.0 / kLateralAccelerationSoftLimit;  // s^2/m

constexpr double kLateralJerkLimit = 2.0;  // m/s^3
const double kInvLateralJerkLimit = 1 / kLateralJerkLimit;
constexpr double kIgnoreLateralJerkSpeedLimit = 3.0;  // m/s.

constexpr double kNormalizeL = kDefaultHalfLaneWidth;
const double kInvNormalizeL = 1.0 / kNormalizeL;

// If speed lower than kIgnoreLateralAccelerationSpeedLimit, do not add lateral
// acceleration cost.
constexpr double kIgnoreLateralAccelerationSpeedLimit = 2.0;  // m/s
constexpr double kCurvatureRegularizer = 0.1;                 // m^-1
const double kInvCurvatureRegularizer = 1.0 / kCurvatureRegularizer;
constexpr double kRefSpeedRegularizer = 3.0;                        // m/s
const double kInvRefSpeedRegularizer = 1.0 / kRefSpeedRegularizer;  // m/s

constexpr double kMinMotionCostSample =
    5;  // Special handling of very short edge.

// For leading object.
constexpr std::array<double, 11> kInterpT = {0.0, 0.9, 1.9, 2.9, 3.9, 4.9,
                                             5.9, 6.9, 7.9, 8.9, 9.9};
constexpr int kGroupLen = 10;
constexpr double kMaxGroupSize = 10;                        // seconds.
constexpr double kHysteresis = 2.0;                         // m.
constexpr double kEnterLaneLateralDistanceThreshold = 1.0;  // m.
constexpr double kInvEnterLaneLateralDistanceThreshold =
    1 / kEnterLaneLateralDistanceThreshold;
constexpr double kLeadingObjectLateralOffsetWeight = 10.0;

// Curvature cost based on state velocity.
const PiecewiseLinearFunction<double> kCurvatureVelocityPlf(
    {1.0, 2.0, 3.0, 4.0, 5.0}, {0.05, 0.1, 0.2, 0.2, 1.0});

enum TransformType {
  L1_NORM = 0,
  L2_NORM = 1,
};

template <TransformType kType>
double Transform(double val) {
  switch (kType) {
    case L1_NORM:
      return std::fabs(val);
    case L2_NORM:
      return Sqr(val);
  }
}

void SetZeroCost(absl::Span<double> cost) {
  for (auto& val : cost) {
    val = 0.0;
  }
}

[[maybe_unused]] void FixInf(absl::Span<double> ty, bool get_s_max) {
  if (get_s_max) {
    // Fix front object's ty, only fix -inf.
    for (int i = ty.size() - 1; i > 0; --i) {
      if (std::isinf(-ty[i - 1])) {
        ty[i - 1] = ty[i];
      }
    }
    return;
  }
  // Fix rear object's ty, fix inf.
  for (int i = 1; i < ty.size(); ++i) {
    if (std::isinf(ty[i])) {
      ty[i] = ty[i - 1];
    }
  }
}

std::array<double, 4> CalculatePathSlCostRatio(double s, double l,
                                               const PathSlBoundary& path_sl,
                                               double normalize_l,
                                               double inv_normalize_l,
                                               double sdc_half_width) {
  const auto [l_right, l_left] =
      path_sl.QueryBoundaryL(s);  // l_right <= l_left
  const double to_right = l - l_right - sdc_half_width;
  const double to_left = l_left - l - sdc_half_width;
  double l2right_ratio =
      std::clamp(kBoundarySoftZone - to_right, 0.0, normalize_l) *
      inv_normalize_l;
  double l2left_ratio =
      std::clamp(kBoundarySoftZone - to_left, 0.0, normalize_l) *
      inv_normalize_l;

  const auto [l_target_right, l_target_left] = path_sl.QueryTargetBoundaryL(s);
  const double to_target_right = l - l_target_right - sdc_half_width;
  const double to_target_left = l_target_left - l - sdc_half_width;
  const double l2target_right_ratio =
      std::clamp(kBoundarySoftZone - to_target_right, 0.0, normalize_l) *
      inv_normalize_l;
  const double l2target_left_ratio =
      std::clamp(kBoundarySoftZone - to_target_left, 0.0, normalize_l) *
      inv_normalize_l;
  return {
      {l2right_ratio, l2left_ratio, l2target_right_ratio, l2target_left_ratio}};
}

void CalculateLateralAccelerationCost(const std::vector<MotionState>& states,
                                      absl::Span<double> cost) {
  const double duration = states.back().t - states.front().t;
  for (int i = 0, n = states.size(); i < n - 1; ++i) {
    const double lat_acc = states[i].k * states[i].v * states[i].v;
    double abs_lat_acc = std::fabs(lat_acc);
    if (states[i].v < kIgnoreLateralAccelerationSpeedLimit) {
      abs_lat_acc = 0.0;
    }
    if (abs_lat_acc > kLateralAccelerationHardLimit) {
      cost[0] = 1.0 * duration;
    }
    if (abs_lat_acc <= kLateralAccelerationSoftLimit) {
      cost[1] += 0.0;
    } else {
      cost[1] += Transform<TransformType::L2_NORM>(
                     abs_lat_acc * kInvLateralAccelerationSoftLimit - 1.0) *
                 (states[i + 1].t - states[i].t);
    }
  }
}

void GetObjectsFromSearchConfig(const InitializerSearchConfig& search_config,
                                std::vector<std::string>* front,
                                std::vector<std::string>* rear) {
  front->reserve(search_config.leading_object_config().front().size());
  for (const auto& object : search_config.leading_object_config().front()) {
    front->push_back(object);
  }
  // Consider potential rear object while lane changing.
  if (search_config.is_lane_change()) {
    rear->reserve(search_config.leading_object_config().rear().size());
    for (const auto& object : search_config.leading_object_config().rear()) {
      rear->push_back(object);
    }
  }
}

void CalculateLateralJerkCost(const std::vector<MotionState>& states,
                              absl::Span<double> cost) {
  // Calculate jerk in SL coordinate. Current SL reference line is drive passage
  // stations (no smoothing).
  // TODO(changqing): Switch to new reference line when possible.
  std::vector<double> lat_acc_sl(states.size(),
                                 std::numeric_limits<double>::infinity());
  for (int i = 1, n = states.size(); i < n - 1; ++i) {
    const double dldt_1 =
        (states[i].l - states[i - 1].l) / (states[i].t - states[i - 1].t);
    const double dldt_2 =
        (states[i + 1].l - states[i].l) / (states[i + 1].t - states[i].t);
    const double dt = 0.5 * (states[i + 1].t - states[i - 1].t);
    lat_acc_sl[i] = (dldt_2 - dldt_1) / dt;
  }

  for (int i = 1, n = states.size(); i < n - 2; ++i) {
    // Calculate d^3l/dt^3 use lat_acc_sl[i] and lat_acc_sl[i + 1].
    const double dlat_acc = lat_acc_sl[i + 1] - lat_acc_sl[i];
    const double dt = states[i + 1].t - states[i].t;
    double jerk_ratio =
        std::clamp(dlat_acc / dt * kInvLateralJerkLimit, -1.0, 1.0);
    if (states[i].v < kIgnoreLateralJerkSpeedLimit) {
      jerk_ratio = 0.0;
    }
    cost[2] += Transform<TransformType::L2_NORM>(jerk_ratio) * dt;
  }
}

absl::StatusOr<std::pair<double, double>> GetBoxSRange(
    const Box2d& box, const DrivePassage& passage) {
  // Get the s-direction range of the object (box) on drive passage.
  double s_min = std::numeric_limits<double>::infinity();
  double s_max = std::numeric_limits<double>::lowest();
  const auto frenet_box_or = passage.QueryFrenetBoxAt(box);
  if (!frenet_box_or.ok()) {
    return absl::NotFoundError(
        "Cannot project the entire object box onto drive passage.");
  }
  s_min = std::min(s_min, frenet_box_or->s_min);
  s_max = std::max(s_max, frenet_box_or->s_max);
  return std::make_pair(s_min, s_max);
}

PiecewiseLinearFunction<double> GetObjectTimeAccumulatedSPlf(
    const ConstraintManager& c_mgr,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DrivePassage& drive_passage, const std::vector<std::string>& objects,
    bool get_s_max) {
  // Get a time - accumulated_s correspondence of a group of object. If
  // get_s_max is true, the group of objects is the leading object group.
  std::vector<double> ty(kInterpT.size());
  if (get_s_max) {
    std::fill(ty.begin(), ty.end(), std::numeric_limits<double>::infinity());
  } else {
    std::fill(ty.begin(), ty.end(), -std::numeric_limits<double>::infinity());
  }
  for (const auto& object : objects) {
    const auto states =
        QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(object))->states();
    // (s_min, s_max).
    const auto s_range_or = GetBoxSRange(states.front().box, drive_passage);
    if (s_range_or.ok()) {
      ty[0] = get_s_max ? std::min(ty[0], s_range_or->first)
                        : std::max(ty[0], s_range_or->second);
    }

    for (int i = 1; i <= kMaxGroupSize; ++i) {
      if (i * kGroupLen - 1 >= states.size()) {
        break;
      }
      const auto s_range_or =
          GetBoxSRange(states[i * kGroupLen - 1].box, drive_passage);
      if (s_range_or.ok()) {
        ty[i] = get_s_max ? std::min(ty[i], s_range_or->first)
                          : std::max(ty[i], s_range_or->second);
      }
    }
  }
  FixInf(absl::MakeSpan(ty), get_s_max);
  return PiecewiseLinearFunction({kInterpT.begin(), kInterpT.end()},
                                 std::move(ty));
}

}  // namespace

// --------------- Acceleration Cost --------------------
void DpAccelerationFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                            absl::Span<double> cost) const {
  const auto get_normalize_a = [this](double a) {
    return a < 0.0 ? config_->motion_constraint_params().max_deceleration()
                   : config_->motion_constraint_params().max_acceleration();
  };
  const double duration = edge_info.motion_form->duration();
  DCHECK_EQ(cost.size(), 1);
  const auto& states = edge_info.states;
  DCHECK_GE(states.size(), 1);
  // Only one state in sampled states.
  if (states.size() == 1) {
    cost[0] = Transform<TransformType::L2_NORM>(states[0].a /
                                                get_normalize_a(states[0].a)) *
              duration;
    return;
  }
  double accel_cost = 0.0;
  for (int i = 0, n = states.size(); i < n - 1; i += kEvalStep) {
    const double a = states[i].a;
    const int next_idx = std::min<int>(n - 1, i + kEvalStep);
    accel_cost += Transform<TransformType::L2_NORM>(a / get_normalize_a(a)) *
                  (states[next_idx].t - states[i].t);
  }
  cost[0] = accel_cost;
}

//--------------- Lane Boundary Cost -----------------
void DpLaneBoundaryFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                            absl::Span<double> cost) const {
  DCHECK_EQ(cost.size(), 2);
  SetZeroCost(cost);
  const auto& states = edge_info.states;
  DCHECK_GE(states.size(), 1);

  for (int i = 0, n = states.size(); i < std::max(1, n - 1); ++i) {
    const auto l2ratios = CalculatePathSlCostRatio(
        states[i].accumulated_s, states[i].l, *path_sl_, kNormalizeL,
        kInvNormalizeL, sdc_half_width_);

    const double l2right_ratio = l2ratios[0];
    const double l2left_ratio = l2ratios[1];
    const double l2target_right_ratio = l2ratios[2];
    const double l2target_left_ratio = l2ratios[3];
    const double duration = (n == 1) ? edge_info.motion_form->duration()
                                     : (states[i + 1].t - states[i].t);

    cost[0] += (Transform<TransformType::L2_NORM>(l2right_ratio) +
                Transform<TransformType::L2_NORM>(l2left_ratio) +
                Transform<TransformType::L2_NORM>(l2target_right_ratio) +
                Transform<TransformType::L2_NORM>(l2target_left_ratio)) *
               duration;
    const double ref_l =
        path_sl_->QueryReferenceCenterL(states[i].accumulated_s);
    const double l_ratio =
        std::clamp((states[i].l - ref_l) * kInvNormalizeL, -1.0, 1.0);
    cost[1] += Transform<TransformType::L2_NORM>(l_ratio) * duration;
  }
}

//--------------- Curvature Cost -----------------
void DpCurvatureFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                         absl::Span<double> cost) const {
  DCHECK_EQ(cost.size(), 1);
  cost[0] = 0.0;
  const double duration = edge_info.motion_form->duration();
  const auto& states = edge_info.states;
  // If the motion edge's duration is very short, it may cause the sampled
  // states to have only the beginning state and the end state. The curvatures
  // of discretized states cannot actually represent the real form of the
  // trajectory. We thus resample the motion. Only curvature cost needs this
  // kind of special handling.
  if (states.size() < kMinMotionCostSample) {
    const auto resampled_states =
        edge_info.motion_form->Sample(duration * (1.0 / kMinMotionCostSample));
    for (int i = 0, n = resampled_states.size(); i < n - 1; ++i) {
      const double kRelaxationFactor =
          kCurvatureVelocityPlf(resampled_states[i].v);
      cost[0] += Transform<TransformType::L2_NORM>(
                     (resampled_states[i].k - resampled_states[i].ref_k) *
                     kInvCurvatureRegularizer) *
                 (resampled_states[i + 1].t - resampled_states[i].t) *
                 kRelaxationFactor;
    }
    return;
  }

  // Normal handling of curvature.
  for (int i = 0, n = states.size(); i < n - 1; ++i) {
    const double kRelaxationFactor = kCurvatureVelocityPlf(states[i].v);
    cost[0] += Transform<TransformType::L2_NORM>(
                   (states[i].k - states[i].ref_k) * kInvCurvatureRegularizer) *
               (states[i + 1].t - states[i].t) * kRelaxationFactor;
  }
}

//--------------- Lateral acceleration Cost -----------------
void DpLateralAccelerationFeatureCost::ComputeCost(
    const MotionEdgeInfo& edge_info, absl::Span<double> cost) const {
  DCHECK_EQ(cost.size(), 3);
  cost[0] = cost[1] = cost[2] = 0.0;
  const double duration = edge_info.motion_form->duration();

  const auto* states = &edge_info.states;
  DCHECK_GE(states->size(), 1);
  std::vector<MotionState> resampled_states;
  // Only one state in sampled states.
  if (states->size() < kMinMotionCostSample) {
    resampled_states =
        edge_info.motion_form->Sample(duration * (1.0 / kMinMotionCostSample));
    states = &resampled_states;
  }
  CalculateLateralAccelerationCost(*states, cost);
  if (!is_lane_change_) {
    CalculateLateralJerkCost(*states, cost);
  }
}

//--------------- Stop Constraint Cost -----------------
void DpStopConstraintFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                              absl::Span<double> cost) const {
  DCHECK_EQ(cost.size(), 1);
  const auto& states = edge_info.states;
  DCHECK_GE(states.size(), 1);
  const double last_s = edge_info.states.back().accumulated_s;
  cost[0] = std::max(0.0, last_s - nearest_stop_s_);
}

// -------------------- RefSpeedFeatureCost --------------------
void DpRefSpeedFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                        absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 3);
  SetZeroCost(cost);

  const auto& states = edge_info.states;
  DCHECK_GE(states.size(), 1);

  // This cost is time consuming, evalute every kEvalStep point to reduce time
  // consumption.
  for (int i = 0, n = states.size(); i < std::max(1, n - 1); i += kEvalStep) {
    const double t = states[i].t + edge_info.start_t;
    const double accumulated_s = states[i].accumulated_s;
    // TODO(xiangjun): change to batch lookup.
    const auto ref_v = ref_speed_table_->LookUpRefSpeed(t, accumulated_s);
    const double v = states[i].v;
    const int next_idx = std::min<int>(n - 1, i + kEvalStep);
    const double duration = (n == 1) ? edge_info.motion_form->duration()
                                     : (states[next_idx].t - states[i].t);
    // exceeds speed limit
    cost[0] += Transform<TransformType::L2_NORM>(
                   v > ref_v.first ? v - ref_v.first : 0.0) *
               duration;
    // exceeds ref speed
    const double exceed_speed = v > ref_v.second ? v - ref_v.second : 0.0;
    cost[1] += Transform<TransformType::L2_NORM>(exceed_speed *
                                                 kInvRefSpeedRegularizer) *
               duration;
    // below ref speed
    const double below_speed = v < ref_v.second ? ref_v.second - v : 0.0;
    cost[2] += Transform<TransformType::L2_NORM>(below_speed *
                                                 kInvRefSpeedRegularizer) *
               duration;
  }
}

// -------------------- DpDynamicCollisionFeatureCost --------------------
DpDynamicCollisionFeatureCost::DpDynamicCollisionFeatureCost(
    const CollisionChecker* collision_checker)
    : FeatureCost("dp_dynamic_collision"),
      cc_(CHECK_NOTNULL(collision_checker)) {}

void DpDynamicCollisionFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                                absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 1);
  CollisionInfo collision_info;
  const auto start_time = edge_info.start_t;
  cc_->CheckCollisionWithTrajectories(start_time, edge_info.states,
                                      &collision_info);
  if (collision_info.collision_objects.empty()) {
    cost[0] = 0.0;
  } else {
    cost[0] = 1.0;
  }
}

// ----------------------- DpLeadingObjectFeatureCost---------------------
DpLeadingObjectFeatureCost::DpLeadingObjectFeatureCost(
    const ConstraintManager* c_mgr,
    const SpacetimeTrajectoryManager* st_traj_mgr,
    const DrivePassage* drive_passage, double sdc_length,
    const InitializerSearchConfig* search_config)
    : FeatureCost("dp_leading_object"),
      passage_(drive_passage),
      c_mgr_(c_mgr),
      st_traj_mgr_(st_traj_mgr),
      sdc_length_(sdc_length),
      is_lane_change_(search_config->is_lane_change()) {
  std::vector<std::string> front;
  std::vector<std::string> rear;
  GetObjectsFromSearchConfig(*search_config, &front, &rear);
  max_s_t_ = GetObjectTimeAccumulatedSPlf(*c_mgr_, *st_traj_mgr_, *passage_,
                                          front, true);
  if (is_lane_change_) {
    min_s_t_ = GetObjectTimeAccumulatedSPlf(*c_mgr_, *st_traj_mgr_, *passage_,
                                            rear, false);
  }
}

void DpLeadingObjectFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                             absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 1);
  cost[0] = 0.0;
  const auto& states = edge_info.states;
  for (int i = 0, n = states.size(); i < n - 1; i += kEvalStep) {
    const double t = states[i].t + edge_info.start_t;
    const double max_s = max_s_t_(t);
    const double ego_max_s =
        states[i].accumulated_s + sdc_length_ + kInitializerMinFollowDistance;
    double cur_cost = ego_max_s - max_s - kHysteresis;
    cur_cost = cur_cost > 0.0 ? cur_cost : 0.0;
    const double lateral_ratio =
        std::max(kEnterLaneLateralDistanceThreshold - std::fabs(states[i].l),
                 0.0) *
        kInvEnterLaneLateralDistanceThreshold;
    if (states[i].accumulated_s > max_s) {
      // If sdc in front of desired leading object, add a lateral offset cost if
      // it enters the target lane too early.
      cur_cost +=
          kLeadingObjectLateralOffsetWeight * Transform<L2_NORM>(lateral_ratio);
    }

    if (is_lane_change_) {
      const double ego_max_s =
          states[i].accumulated_s - kInitializerMinFollowDistance;
      const double min_s = min_s_t_(t);
      double lead_cost = min_s - ego_max_s - kHysteresis;
      lead_cost = lead_cost > 0.0 ? lead_cost : 0.0;
      cur_cost += lead_cost;
      if (states[i].accumulated_s < min_s) {
        // Add lateral cost if it enters target lane too early (behind desired
        // rear object).
        cur_cost += kLeadingObjectLateralOffsetWeight *
                    Transform<L2_NORM>(lateral_ratio);
      }
    }

    const int next_idx = std::min<int>(n - 1, i + kEvalStep);
    // exceeds speed limit
    cost[0] += (cur_cost * (states[next_idx].t - states[i].t));
  }
}

// ----------------------- DpFinalProgressFeatureCost---------------------
void DpFinalProgressFeatureCost::ComputeCost(const MotionEdgeInfo& edge_info,
                                             absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), 2);
  cost[0] = cost[1] = 0.0;
  const double end_time = edge_info.start_t + edge_info.motion_form->duration();
  if (end_time >= kTrajectoryTimeHorizon) {
    // Only calculate final progress cost when end time greater than 10.0s.
    // Longitudinal progress cost.
    const auto end_state = edge_info.motion_form->State(kTrajectoryTimeHorizon -
                                                        edge_info.start_t);
    cost[0] = std::max(max_accumulated_s_ - end_state.accumulated_s, 0.0);
    // Lateral progress cost.
    const double ref_l =
        path_sl_->QueryReferenceCenterL(end_state.accumulated_s);
    cost[1] = std::fabs(end_state.l - ref_l);
  }
}
}  // namespace qcraft::planner
