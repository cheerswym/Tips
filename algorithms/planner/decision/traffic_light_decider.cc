#include "onboard/planner/decision/traffic_light_decider.h"

#include "onboard/planner/planner_params.h"

namespace qcraft {
namespace planner {

namespace {

constexpr double kBrakeDelayTime = 1.0;                    // s
constexpr double kComfortableDecelerationBeforeTl = -1.6;  // m/s^2
constexpr double kEmergencyDecelerationBeforeTl = -3.5;    // m/s^2
constexpr double kPerceptionDelayCompensation = 0.5;       // s
constexpr double kTrafficLightStandoff = 1.0;              // m
constexpr double kMaxLookAhead = 150.0;                    // m.
constexpr double kMaxPastDist = 10.0;                      // m.

bool CanStopBeforeIntersection(const SpeedProfile &preliminary_speed_profile,
                               double distance_to_intersection) {
  const double ego_speed = preliminary_speed_profile.GetVAtTime(0.0);
  const double stop_in_advance_distance =
      ego_speed * kBrakeDelayTime +
      0.5 * ego_speed * ego_speed /
          std::abs(kComfortableDecelerationBeforeTl);  // s=vt+v^2/(2*a)
  VLOG(2) << "distance to intersecion: " << distance_to_intersection;
  VLOG(2) << "stop in advance distance: " << stop_in_advance_distance;
  return distance_to_intersection >= stop_in_advance_distance;
}

bool CanCrossStopLineBeforeTlTurnsRed(
    const SpeedProfile &preliminary_speed_profile,
    double distance_to_intersection, double turn_red_time_remaining) {
  const double ego_speed = preliminary_speed_profile.GetVAtTime(0.0);
  const double emergency_stop_distance =
      0.5 * ego_speed * ego_speed /
      std::abs(kEmergencyDecelerationBeforeTl);  // s=v^2/(2*a)
  const double time_to_reach_intersection =
      preliminary_speed_profile.GetTimeAtS(distance_to_intersection);

  VLOG(2) << "emergency stop distance:" << emergency_stop_distance;
  VLOG(2) << "time to reach intersection:" << time_to_reach_intersection;
  VLOG(2) << "turn red time remaining:" << turn_red_time_remaining;
  return time_to_reach_intersection <=
             turn_red_time_remaining - kPerceptionDelayCompensation ||
         distance_to_intersection <= emergency_stop_distance;
}

double CalculateStopPoint(double tl_path_s) {
  const double av_rac_to_fb = PlannerParams::GetRunParams()
                                  .vehicle_params()
                                  .vehicle_geometry_params()
                                  .front_edge_to_center();
  return
      /*stop_point=*/(tl_path_s - kTrafficLightStandoff - av_rac_to_fb);
}

absl::StatusOr<ConstraintProto::StopLineProto> GenerateTrafficLightConstraint(
    const DrivePassage &passage, const TlDirectiveGenerator &generator) {
  if (generator.proceed) {
    return absl::PermissionDeniedError(
        "no need to generate stop line constraint for proceed state");
  }
  const auto &curbs = passage.QueryCurbPointAtS(generator.tl_path_s +
                                                passage.lane_path_start_s());
  if (!curbs.ok()) {
    return absl::NotFoundError("no curb found");
  }

  ConstraintProto::StopLineProto stop_line;
  stop_line.set_s(generator.tl_path_s);
  stop_line.set_standoff(kTrafficLightStandoff);
  stop_line.set_time(0.0);
  stop_line.set_id(absl::StrFormat("traffic_light_%d", generator.tl_id));
  stop_line.mutable_source()->mutable_traffic_light()->set_id(generator.tl_id);
  HalfPlane halfplane(curbs->first, curbs->second);
  halfplane.ToProto(stop_line.mutable_half_plane());
  return stop_line;
}

TlDirectiveGenerator MakeSingleTrafficLightStopDecision(
    const TlInfo &tl_info, const SpeedProfile &preliminary_speed_profile,
    bool last_tl_proceed, bool *has_received_known_traffic_light) {
  QCHECK_EQ(tl_info.control_point_relative_s().size(), 1);

  TlDirectiveGenerator generator;

  generator.lane_id = tl_info.lane_id();
  VLOG(3) << "Considering traffic light at lane " << generator.lane_id;
  VLOG(3) << tl_info.DebugString();

  // CVC 21453
  // https://leginfo.legislature.ca.gov/faces/codes_displaySection.xhtml?lawCode=VEH&sectionNum=21453.
  // If we've driven past this TL, but we are confident the TL was red when
  // we entered the intersection, then we can't consider this TL to be in
  // our past: otherwise we'll proceed to run the red light if we failed to
  // stop before entering the intersection for whatever reasons (e.g.
  // insufficient braking due to control or DBW).
  const double tl_path_s =
      tl_info.control_point_relative_s().front();  // only one control point
  const bool tl_is_ahead = tl_path_s >= 0.0 && tl_path_s <= kMaxLookAhead;
  const bool tl_is_past = tl_path_s < 0.0 && tl_path_s > 0.0 - kMaxPastDist;

  if (!tl_is_ahead && !tl_is_past) {
    if (tl_path_s > kMaxLookAhead) {
      generator.reason = "The traffic light is too far ahead.";
    } else if (tl_path_s <= 0.0 - kMaxPastDist) {
      generator.reason = "The traffic light is too far behind us.";
    }
    return generator;
  }

  const auto single_tl_info =
      FindOrNull(tl_info.tls(), TrafficLightDirection::UNMARKED);
  if (single_tl_info == nullptr) {
    generator.reason = "No traffic light info.";
    return generator;
  }
  generator.tl_id = single_tl_info->tl_id;
  const TrafficLightState tl_state = single_tl_info->tl_state;
  const double turn_red_time_remaining =
      single_tl_info->estimated_turn_red_time_left;

  if (tl_state != TrafficLightState::TL_STATE_UNKNOWN) {
    *has_received_known_traffic_light = true;
  }

  // Whether the signal protects us to proceed (either a yellow we can go
  // through or a green).
  bool protected_proceed = false;
  if (tl_state == TrafficLightState::TL_STATE_UNKNOWN) {
    if (*has_received_known_traffic_light) {
      protected_proceed = last_tl_proceed;
      absl::StrAppend(
          &generator.reason,
          "Color is unknow, follow with last traffic light decision;");
    } else {
      absl::StrAppend(&generator.reason,
                      "Color is unknown, no traffic light decision has made, "
                      "forbidden to go.");
    }
  } else if (tl_is_past) {
    if (tl_state == TrafficLightState::TL_STATE_GREEN) {
      protected_proceed = true;
    } else {
      protected_proceed = last_tl_proceed;
    }
  } else {
    if (tl_state == TrafficLightState::TL_STATE_RED) {
      absl::StrAppend(&generator.reason, "color is red; ");
    } else if ((tl_state == TrafficLightState::TL_STATE_YELLOW) ||
               (tl_state == TrafficLightState::TL_STATE_GREEN_FLASHING)) {
      // Yellow or Green flashing. Estimate whether ego can brake with a
      // comfortable deceleration 1.5m/s^2. Make a decision to brake if
      // possible.
      if (tl_state == TrafficLightState::TL_STATE_YELLOW) {
        absl::StrAppend(&generator.reason, "color is yellow; ");
      } else {
        absl::StrAppend(&generator.reason, "color is flashing green; ");
      }
      protected_proceed =
          !CanStopBeforeIntersection(preliminary_speed_profile, tl_path_s) &&
          CanCrossStopLineBeforeTlTurnsRed(preliminary_speed_profile, tl_path_s,
                                           turn_red_time_remaining);

      if (!last_tl_proceed) protected_proceed = false;
    } else if (tl_state == TrafficLightState::TL_STATE_YELLOW_FLASHING) {
      // Flashing yellow. Go.
      absl::StrAppend(&generator.reason, "color is flashing yellow; ");
      protected_proceed = true;
    } else {
      // Green. Go.
      absl::StrAppend(&generator.reason, "color is green; ");
      protected_proceed = true;
    }
  }
  VLOG(3) << "Protected Proceed: " << protected_proceed << ", "
          << generator.reason;

  // If we decided the signal tells us to stop, look at the lane's
  // right-on-red label to decide if we can still proceed. If the signal
  // tells us not to stop, then proceed without checking.
  // Note that when proceeding, we will still consider interaction with
  // other agents, even if we have the green light and they don't. This will
  // be handled by yielding.
  bool proceed = protected_proceed;
  if (!protected_proceed) {
    if (tl_info.can_go_on_red()) {
      absl::StrAppend(&generator.reason, "can go on red; ");
      proceed = true;
      VLOG(3) << "Can go on red.";
    }
  }

  // Add a speed constraint if we can't proceed. Note that if proceed==false
  // there should still be a speed constraint if stop==true, but this will
  // be handled by the stop sign logic which checks
  // traffic_light_stops.
  generator.stop_point = proceed ? std::numeric_limits<double>::infinity()
                                 : CalculateStopPoint(tl_path_s);
  generator.tl_path_s = tl_path_s;
  generator.proceed = proceed;
  generator.protected_proceed = protected_proceed;

  return generator;
}

TlDirectiveGenerator MakeMultiTrafficLightStopDecision(
    const TlInfo &tl_info, const SpeedProfile &preliminary_speed_profile,
    bool last_tl_proceed, bool *entry_with_left_light_not_red) {
  QCHECK_EQ(tl_info.control_point_relative_s().size(), 2);

  TlDirectiveGenerator generator;

  generator.lane_id = tl_info.lane_id();
  VLOG(3) << "Considering traffic light at lane " << generator.lane_id;
  VLOG(3) << tl_info.DebugString();

  const double tl_path_s_first =
      tl_info.control_point_relative_s().front();  // first control point
  const double tl_path_s_second =
      tl_info.control_point_relative_s().back();  // second control point
  const bool tl_is_ahead_first =
      tl_path_s_first >= 0.0 && tl_path_s_first <= kMaxLookAhead;
  const bool tl_is_past_first =
      tl_path_s_first < 0.0 && tl_path_s_second >= 0.0;
  const bool tl_is_past_second =
      tl_path_s_second < 0.0 && tl_path_s_second > 0.0 - kMaxPastDist;

  if (!tl_is_ahead_first && !tl_is_past_first && !tl_is_past_second) {
    if (tl_path_s_first > kMaxLookAhead) {
      generator.reason = "The traffic light is too far ahead.";
    } else if (tl_path_s_second <= 0.0 - kMaxPastDist) {
      generator.reason = "The traffic light is too far behind us.";
      *entry_with_left_light_not_red = false;
    }
    return generator;
  }

  const auto left_tl_info =
      FindOrNull(tl_info.tls(), TrafficLightDirection::LEFT);
  const auto straight_tl_info =
      FindOrNull(tl_info.tls(), TrafficLightDirection::STRAIGHT);
  if (left_tl_info == nullptr || straight_tl_info == nullptr) {
    generator.reason = "No traffic light info.";
    return generator;
  }
  const TrafficLightState left_tl_state = left_tl_info->tl_state;
  const TrafficLightState straight_tl_state = straight_tl_info->tl_state;

  const mapping::ElementId left_tl_id = left_tl_info->tl_id;
  const mapping::ElementId straight_tl_id = straight_tl_info->tl_id;

  const double left_turn_red_time_remaining =
      left_tl_info->estimated_turn_red_time_left;
  // const double straight_turn_red_time_remaining =
  //    straight_tl_info->estimated_turn_red_time_left;

  bool protected_proceed = false;
  if (left_tl_state == TrafficLightState::TL_STATE_GREEN) {
    // Left tl is green, go whenever
    generator.tl_path_s =
        tl_is_ahead_first ? tl_path_s_first : tl_path_s_second;
    generator.tl_id = tl_is_ahead_first ? straight_tl_id : left_tl_id;
    protected_proceed = true;
    if (!tl_is_ahead_first) *entry_with_left_light_not_red = true;

    absl::StrAppend(&generator.reason, "left tl is green; ");
  } else if (left_tl_state == TrafficLightState::TL_STATE_GREEN_FLASHING ||
             left_tl_state == TrafficLightState::TL_STATE_YELLOW) {
    absl::StrAppend(&generator.reason, "left tl is green flashing/yellow; ");
    if (tl_is_ahead_first) {
      generator.tl_path_s = tl_path_s_first;
      generator.tl_id = straight_tl_id;
      protected_proceed = !CanStopBeforeIntersection(preliminary_speed_profile,
                                                     tl_path_s_first) &&
                          CanCrossStopLineBeforeTlTurnsRed(
                              preliminary_speed_profile, tl_path_s_first,
                              left_turn_red_time_remaining);
      if (!last_tl_proceed) protected_proceed = false;
      *entry_with_left_light_not_red = protected_proceed;

      absl::StrAppend(&generator.reason, "not cross first stop line; ");
    } else {
      absl::StrAppend(&generator.reason, "cross first stop line; ");
      generator.tl_id = left_tl_id;
      if (*entry_with_left_light_not_red) {
        generator.tl_path_s = tl_path_s_second;
        protected_proceed = true;

        absl::StrAppend(&generator.reason,
                        "entry with left light not red then can go; ");
      } else {
        generator.tl_path_s = tl_path_s_second;
        protected_proceed = false;

        absl::StrAppend(&generator.reason, "stop in the left waiting area; ");
      }
    }
  } else {
    absl::StrAppend(&generator.reason, "left tl is red/unknown; ");
    if (tl_is_ahead_first) {
      absl::StrAppend(&generator.reason, "not cross first stop line; ");
      if (straight_tl_state == TrafficLightState::TL_STATE_GREEN ||
          straight_tl_state == TrafficLightState::TL_STATE_GREEN_FLASHING ||
          straight_tl_state == TrafficLightState::TL_STATE_YELLOW) {
        generator.tl_path_s = tl_path_s_second;
        generator.tl_id = left_tl_id;
        protected_proceed = false;

        absl::StrAppend(
            &generator.reason,
            "straight tl is green or yellow; stop in the left waiting area; ");
      } else {
        generator.tl_path_s = tl_path_s_first;
        generator.tl_id = straight_tl_id;
        protected_proceed = false;

        absl::StrAppend(
            &generator.reason,
            "straight tl is red or unknow; stop before the first stop line; ");
      }
    } else {
      generator.tl_id = left_tl_id;
      absl::StrAppend(&generator.reason, "cross first stop line; ");
      if (*entry_with_left_light_not_red) {
        generator.tl_path_s = tl_path_s_second;
        protected_proceed = true;

        absl::StrAppend(&generator.reason,
                        "entry with left light not red then can go; ");
      } else {
        generator.tl_path_s = tl_path_s_second;
        protected_proceed = false;

        absl::StrAppend(&generator.reason, "stop in the left waiting area; ");
      }
    }
  }
  VLOG(2) << generator.reason;
  VLOG(2) << "entry_with_left_light_not_red: "
          << *entry_with_left_light_not_red;
  VLOG(2) << "Protected Proceed: " << protected_proceed;

  bool proceed = protected_proceed;
  if (!protected_proceed) {
    if (tl_info.can_go_on_red()) {
      absl::StrAppend(&generator.reason, "can go on red; ");
      proceed = true;
      VLOG(3) << "Can go on red.";
    }
  }

  generator.stop_point = proceed ? std::numeric_limits<double>::infinity()
                                 : CalculateStopPoint(generator.tl_path_s);
  generator.proceed = proceed;
  generator.protected_proceed = protected_proceed;

  return generator;
}

TlDirectiveGenerator MakeTrafficLightStopDecision(
    const TlInfo &tl_info, const SpeedProfile &preliminary_speed_profile,
    bool last_tl_proceed, bool *entry_with_left_light_not_red,
    bool *has_received_known_traffic_light) {
  if (tl_info.tl_control_type() == TrafficLightControlType::SINGLE_DIRECTION) {
    return MakeSingleTrafficLightStopDecision(
        tl_info, preliminary_speed_profile, last_tl_proceed,
        has_received_known_traffic_light);
  } else {
    /* if (tl_info.tl_control_type() ==
     * TrafficLightControlType::LEFT_WAITING_AREA)*/
    return MakeMultiTrafficLightStopDecision(tl_info, preliminary_speed_profile,
                                             last_tl_proceed,
                                             entry_with_left_light_not_red);
  }
}

bool CheckTwoLanesInSameSection(const PlannerSemanticMapManager &psmm,
                                mapping::ElementId id_1,
                                mapping::ElementId id_2) {
  if (id_1 == mapping::kInvalidElementId || id_2 == mapping::kInvalidElementId)
    return false;

  return psmm.FindLaneInfoOrDie(id_1).section_id ==
         psmm.FindLaneInfoOrDie(id_2).section_id;
}

}  // namespace

absl::StatusOr<TrafficLightDeciderOutput> BuildTrafficLightConstraints(
    const PlannerSemanticMapManager &psmm,
    const qcraft::VehicleGeometryParamsProto &vehicle_geometry_params,
    const ApolloTrajectoryPointProto &plan_start_point,
    const mapping::LanePath &lane_path, const DrivePassage &passage,
    const TrafficLightInfoMap &tl_info_map,
    const SpeedProfile &preliminary_speed_profile,
    const TrafficLightDeciderStateProto &tld_state) {
  std::vector<ConstraintProto::StopLineProto> tl_stop_lines;

  bool last_tl_proceed = tld_state.last_tl_proceed();
  bool entry_with_left_light_not_red =
      tld_state.entry_with_left_light_not_red();
  bool has_received_known_traffic_light =
      tld_state.has_received_known_traffic_light();
  mapping::ElementId last_tl_decision_lane_id =
      tld_state.last_tl_decision_lane_id();

  mapping::ElementId current_tl_decision_lane_id = mapping::kInvalidElementId;
  // Note(jiayu): There is some difference about stop line performance, when
  // stop line at the begin or end of the lane. But for now, we check the stop
  // line distance to guarante using correctly.
  for (auto it_lane_path = lane_path.begin(); it_lane_path != lane_path.end();
       ++it_lane_path) {
    const auto lane_id = (*it_lane_path).lane_id;
    const auto &it_tl_info = tl_info_map.find(lane_id);
    if (it_tl_info == tl_info_map.end()) continue;

    const auto &tl_info = it_tl_info->second;
    const auto generator = MakeTrafficLightStopDecision(
        tl_info, preliminary_speed_profile, last_tl_proceed,
        &entry_with_left_light_not_red, &has_received_known_traffic_light);
    const auto stop_line_or =
        GenerateTrafficLightConstraint(passage, generator);
    if (stop_line_or.ok()) {
      tl_stop_lines.emplace_back(*stop_line_or);
    }
    last_tl_proceed = generator.proceed;
    current_tl_decision_lane_id = lane_id;
    // Make traffic light decision for the nearest lane which under traffic
    // light control.
    break;
  }
  // Reset traffic light state.
  if (!CheckTwoLanesInSameSection(psmm, current_tl_decision_lane_id,
                                  last_tl_decision_lane_id)) {
    // Last id :  Invalid   ------->   Current id : Invalid.
    // Last id :  Invalid   ------->   Current id : Element_x.
    // Last id :  Element_x ------->   Current id : Invalid.
    // Last id :  Element_a ------->   Current id : Element_b. Belong to
    // different secions.
    last_tl_proceed = true;
    entry_with_left_light_not_red = false;
    has_received_known_traffic_light = false;
    last_tl_decision_lane_id = current_tl_decision_lane_id;

  } else if (current_tl_decision_lane_id != last_tl_decision_lane_id) {
    // Last id :  Element_c ------->   Current id : Element_d. Belong to same
    // section.
    last_tl_decision_lane_id = current_tl_decision_lane_id;
  }

  TrafficLightDeciderStateProto new_tld_state;
  new_tld_state.set_last_tl_proceed(last_tl_proceed);
  new_tld_state.set_entry_with_left_light_not_red(
      entry_with_left_light_not_red);
  new_tld_state.set_has_received_known_traffic_light(
      has_received_known_traffic_light);
  new_tld_state.set_last_tl_decision_lane_id(last_tl_decision_lane_id);

  return TrafficLightDeciderOutput{
      .stop_lines = std::move(tl_stop_lines),
      .traffic_light_decider_state = std::move(new_tld_state)};
}

}  // namespace planner
}  // namespace qcraft
