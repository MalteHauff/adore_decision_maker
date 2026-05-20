#include "behaviours.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "behaviour_common.hpp"
#include "map_route_utils.hpp"
#include "planning/planning_helpers.hpp"

namespace adore::behaviours
{
namespace
{

using common::fallback_follow_or_standstill;
using common::make_default_participant;
using common::to_map_point;

using map_utils::advance_lane_s_along_travel;
using map_utils::find_center_point_on_lane_at_s;
using map_utils::find_closest_center_point_on_lane;
using map_utils::lane_heading_near_point;
using map_utils::lane_id_at_route_s;
using map_utils::route_heading_at_s;
using map_utils::signed_lateral_offset_from_heading_angle;
using map_utils::wrap_angle;

constexpr int kDesiredLeft  = +1;
constexpr int kDesiredRight = -1;

constexpr double kLaneSearchRadiusMeters    = 4.0;
constexpr double kCurrentLaneMaxCenterDist  = 4.0;

constexpr double kDegToRad               = M_PI / 180.0;
constexpr double kMaxHeadingDiffRad      = 15.0 * kDegToRad;
constexpr double kMaxBranchHeadingDiffRad = 7.0 * kDegToRad;
constexpr double kMinAdjacentLaneDistanceMeters = 2.2;
constexpr double kMaxAdjacentLaneDistanceMeters = 4.8;

constexpr double kTargetLaneReachedDistanceMeters    = 0.2;
constexpr double kSourceLaneLeftDistanceMeters        = 2.3;
constexpr double kTrajectoryFailRescueDistanceMeters  = 1.0;

// Trajectory blending tuning parameters
constexpr double kMinLaneChangeSpeedMs   = 4.0;   // floor speed during lane change
constexpr double kLaneChangeSpeedBiasMs  = 0.5;   // small boost added to current speed
constexpr double kTransitionDistFactor   = 1.3;   // transition_m = factor * target_speed
constexpr double kMinTransitionMeters    = 9.0;
constexpr double kMaxTransitionMeters    = 16.0;
constexpr double kBlendLeadFactor        = 0.10;  // fraction of transition_m used as lead
constexpr double kBlendLeadMaxMeters     = 1.5;

// Topology check window constants
constexpr double kTopoWindowBeforeM  = 8.0;
constexpr double kTopoWindowAfterM   = 20.0;
constexpr double kTopoLookaheadM     = 45.0;
constexpr double kTopoStepM          = 5.0;
constexpr double kTopoMaxRouteHeadingDiffRad = 10.0 * kDegToRad;

struct AdjacentLaneCandidate
{
  size_t lane_id;
  double signed_lat;
  double heading_diff;
  double center_dist;
};

// ── Lane lookup helper ────────────────────────────────────────────────────────
const adore::map::Lane* find_lane(const Domain& domain, size_t lane_id)
{
  if (!domain.map)
    return nullptr;

  auto it = domain.map->lanes.find(lane_id);
  if (it == domain.map->lanes.end() || !it->second)
    return nullptr;

  return it->second.get();
}

std::optional<size_t> get_current_route_lane_id(const Domain& domain)
{
  if (!domain.route.has_value() || !domain.vehicle_state.has_value())
    return std::nullopt;

  if (domain.route->reference_line.empty())
    return std::nullopt;

  const double s_now = domain.route->get_s(*domain.vehicle_state);
  return lane_id_at_route_s(*domain.route, s_now);
}

std::optional<double> distance_to_lane_center(
    const Domain& domain,
    size_t lane_id,
    const adore::map::MapPoint& ego_pt)
{
  const auto* lane = find_lane(domain, lane_id);
  if (!lane)
    return std::nullopt;

  auto match = find_closest_center_point_on_lane(*lane, ego_pt);
  if (!match.has_value())
    return std::nullopt;

  return match->distance;
}

double smooth_step(double t)
{
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

bool route_lane_is_stable_ahead(
    const Domain& domain,
    size_t current_route_lane_id,
    double route_s_now,
    double lookahead_m = kTopoLookaheadM,
    double step_m      = kTopoStepM)
{
  if (!domain.route.has_value())
    return false;

  const auto& route = *domain.route;

  const double route_end_s =
      std::min(route.get_length(), route_s_now + lookahead_m);

  for (double s = route_s_now; s <= route_end_s; s += step_m)
  {
    auto lane_id = lane_id_at_route_s(route, s);

    if (!lane_id.has_value())
      return false;

    if (*lane_id != current_route_lane_id)
      return false;
  }

  return true;
}

bool lane_is_in_current_route_corridor(
    const Domain& domain,
    size_t lane_id,
    size_t route_lane_now)
{
  if (!domain.map)
    return false;

  if (lane_id == route_lane_now)
    return true;

  const auto parallels = domain.map->get_parallel_lanes(route_lane_now);

  return std::find(parallels.begin(), parallels.end(), lane_id) != parallels.end();
}

// ── State helpers (delegate to LaneChangeState methods) ───────────────────────

void reset_lane_change_active_state(PlanningParams& planning_tools)
{
  RCLCPP_INFO(
      rclcpp::get_logger("decision_maker"),
      "RESET active lane-change state: source=%s target=%s switch=%s cached=%s direction=%d",
      planning_tools.lane_change.source_lane_id.has_value() ? "yes" : "no",
      planning_tools.lane_change.target_lane_id.has_value() ? "yes" : "no",
      planning_tools.lane_change.switch_source_s.has_value() ? "yes" : "no",
      planning_tools.lane_change.cached_route.has_value()    ? "yes" : "no",
      planning_tools.lane_change.direction);

  planning_tools.lane_change.reset_active();
}

void clear_lane_change_done_state(PlanningParams& planning_tools)
{
  RCLCPP_INFO(
      rclcpp::get_logger("decision_maker"),
      "CLEAR done lane-change state: done=%d done_direction=%d done_lane=%s",
      planning_tools.lane_change.done,
      planning_tools.lane_change.done_direction,
      planning_tools.lane_change.done_lane_id.has_value() ? "yes" : "no");

  planning_tools.lane_change.reset_done();
}

void mark_lane_change_done(
    PlanningParams& planning_tools,
    int direction,
    size_t reached_lane_id)
{
  RCLCPP_INFO(
      rclcpp::get_logger("decision_maker"),
      "MARK lane-change done: direction=%d reached_lane=%zu",
      direction,
      reached_lane_id);

  planning_tools.lane_change.reset_active();
  planning_tools.lane_change.done           = true;
  planning_tools.lane_change.done_direction = direction;
  planning_tools.lane_change.done_lane_id   = reached_lane_id;
}

// ── Lane detection ────────────────────────────────────────────────────────────

std::optional<size_t> find_current_driving_lane_id(const Domain& domain)
{
  if (!domain.map || !domain.vehicle_state)
    return std::nullopt;

  const auto ego_pt = to_map_point(*domain.vehicle_state);

  std::vector<size_t> candidate_lane_ids;

  if (domain.route.has_value() && !domain.route->reference_line.empty())
  {
    const double route_s = domain.route->get_s(*domain.vehicle_state);
    auto route_lane_id = lane_id_at_route_s(*domain.route, route_s);

    if (route_lane_id.has_value())
    {
      candidate_lane_ids.push_back(*route_lane_id);

      auto parallels = domain.map->get_parallel_lanes(*route_lane_id);
      candidate_lane_ids.insert(
          candidate_lane_ids.end(),
          parallels.begin(),
          parallels.end());
    }
  }

  if (candidate_lane_ids.empty())
  {
    candidate_lane_ids =
        domain.map->get_nearby_lane_ids(ego_pt, kLaneSearchRadiusMeters);
  }

  std::sort(candidate_lane_ids.begin(), candidate_lane_ids.end());
  candidate_lane_ids.erase(
      std::unique(candidate_lane_ids.begin(), candidate_lane_ids.end()),
      candidate_lane_ids.end());

  double best_dist = std::numeric_limits<double>::max();
  std::optional<size_t> best_lane_id;

  for (size_t lane_id : candidate_lane_ids)
  {
    const auto* lane = find_lane(domain, lane_id);
    if (!lane)
      continue;

    if (lane->type != adore::map::LaneType::driving)
      continue;

    auto match = find_closest_center_point_on_lane(*lane, ego_pt);
    if (!match.has_value())
      continue;

    if (match->distance < best_dist)
    {
      best_dist = match->distance;
      best_lane_id = lane_id;
    }
  }

  if (!best_lane_id.has_value())
    return std::nullopt;

  if (best_dist > kCurrentLaneMaxCenterDist)
    return std::nullopt;

  return best_lane_id;
}

// ── Adjacent lane selection ───────────────────────────────────────────────────

std::optional<size_t> choose_adjacent_target_lane_id(
    const Domain& domain,
    size_t current_lane_id,
    int desired_direction)
{
  if (!domain.map || !domain.vehicle_state)
    return std::nullopt;

  const auto ego_pt = to_map_point(*domain.vehicle_state);

  const auto* current_lane = find_lane(domain, current_lane_id);
  if (!current_lane)
    return std::nullopt;

  auto current_heading = lane_heading_near_point(*current_lane, ego_pt);
  if (!current_heading.has_value())
    return std::nullopt;

  const bool current_is_motorway =
      domain.map->is_motorway_lane(current_lane_id);

  const auto candidate_lane_ids =
      domain.map->get_parallel_lanes(current_lane_id);

  std::vector<AdjacentLaneCandidate> valid_candidates;

  for (size_t lane_id : candidate_lane_ids)
  {
    const auto* lane = find_lane(domain, lane_id);
    if (!lane)
      continue;

    if (lane->type != adore::map::LaneType::driving)
      continue;

    const bool candidate_is_motorway =
        domain.map->is_motorway_lane(lane_id);

    const bool candidate_is_branch =
        domain.map->is_branch_lane(lane_id);

    if (current_is_motorway != candidate_is_motorway)
      continue;

    auto match = find_closest_center_point_on_lane(*lane, ego_pt);
    if (!match.has_value())
      continue;

    auto candidate_heading = lane_heading_near_point(*lane, match->point);
    if (!candidate_heading.has_value())
      continue;

    const double heading_diff =
        std::abs(wrap_angle(*candidate_heading - *current_heading));

    if (heading_diff > kMaxHeadingDiffRad)
      continue;

    const double signed_lat =
        signed_lateral_offset_from_heading_angle(
            *current_heading,
            ego_pt,
            match->point);

    const double lat_abs = std::abs(signed_lat);

    if (lat_abs < kMinAdjacentLaneDistanceMeters ||
        lat_abs > kMaxAdjacentLaneDistanceMeters)
    {
      continue;
    }

    const bool current_is_branch =
        domain.map->is_branch_lane(current_lane_id);

    if ((current_is_branch || candidate_is_branch) &&
        heading_diff > kMaxBranchHeadingDiffRad)
    {
      continue;
    }

    const bool direction_matches =
        (desired_direction == kDesiredLeft  && signed_lat > 0.0) ||
        (desired_direction == kDesiredRight && signed_lat < 0.0);

    if (direction_matches)
    {
      valid_candidates.push_back(
          {lane_id, signed_lat, heading_diff, match->distance});
    }
  }

  if (valid_candidates.empty())
    return std::nullopt;

  // If multiple lanes qualify (e.g. on a three-lane road), prefer the one
  // whose heading most closely matches ours to avoid jumping two lanes.
  if (valid_candidates.size() > 1)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("decision_maker"),
        "choose_adjacent_target_lane_id: %zu candidates for direction=%d, "
        "picking best heading match",
        valid_candidates.size(),
        desired_direction);

    std::sort(
        valid_candidates.begin(),
        valid_candidates.end(),
        [](const AdjacentLaneCandidate& a, const AdjacentLaneCandidate& b)
        { return a.heading_diff < b.heading_diff; });
  }

  return valid_candidates.front().lane_id;
}

// ── Topology check ────────────────────────────────────────────────────────────

bool is_simple_lane_change_topology(
    const Domain& domain,
    size_t current_lane_id,
    size_t target_lane_id,
    double route_s_now,
    double window_before = kTopoWindowBeforeM,
    double window_after  = kTopoWindowAfterM)
{
  if (!domain.map || !domain.route || !domain.vehicle_state)
    return false;

  const auto* cur_lane = find_lane(domain, current_lane_id);
  const auto* tgt_lane = find_lane(domain, target_lane_id);

  if (!cur_lane || !tgt_lane)
    return false;

  if (cur_lane->type != adore::map::LaneType::driving ||
      tgt_lane->type != adore::map::LaneType::driving)
    return false;

  const bool current_is_motorway = domain.map->is_motorway_lane(current_lane_id);
  const bool target_is_motorway  = domain.map->is_motorway_lane(target_lane_id);

  if (current_is_motorway != target_is_motorway)
    return false;

  const bool current_is_branch = domain.map->is_branch_lane(current_lane_id);
  const bool target_is_branch  = domain.map->is_branch_lane(target_lane_id);

  const auto ego_pt = to_map_point(*domain.vehicle_state);

  auto cur_match = find_closest_center_point_on_lane(*cur_lane, ego_pt);
  auto tgt_match = find_closest_center_point_on_lane(*tgt_lane, ego_pt);

  if (!cur_match.has_value() || !tgt_match.has_value())
    return false;

  auto cur_heading = lane_heading_near_point(*cur_lane, cur_match->point);
  auto tgt_heading = lane_heading_near_point(*tgt_lane, tgt_match->point);

  if (!cur_heading.has_value() || !tgt_heading.has_value())
    return false;

  const double heading_diff =
      std::abs(wrap_angle(*tgt_heading - *cur_heading));

  if (heading_diff > kMaxHeadingDiffRad)
    return false;

  const double signed_lat =
      signed_lateral_offset_from_heading_angle(
          *cur_heading,
          ego_pt,
          tgt_match->point);

  const double lat_abs = std::abs(signed_lat);

  if (lat_abs < kMinAdjacentLaneDistanceMeters ||
      lat_abs > kMaxAdjacentLaneDistanceMeters)
  {
    return false;
  }

  if ((current_is_branch || target_is_branch) &&
      heading_diff > kMaxBranchHeadingDiffRad)
  {
    return false;
  }

  const auto& route = *domain.route;
  auto route_lane_now = lane_id_at_route_s(route, route_s_now);

  if (!route_lane_now.has_value())
    return false;

  if (!route_lane_is_stable_ahead(
          domain,
          *route_lane_now,
          route_s_now))
  {
    return false;
  }

  const double s0 = std::max(0.0, route_s_now - window_before);
  const double s1 = std::min(route.get_length(), route_s_now + window_after);

  auto h0 = route_heading_at_s(route, s0);
  auto h1 = route_heading_at_s(route, s1);

  if (h0.has_value() && h1.has_value())
  {
    const double route_heading_diff =
        std::abs(wrap_angle(*h1 - *h0));

    if (route_heading_diff > kTopoMaxRouteHeadingDiffRad)
      return false;
  }

  return true;
}

// ── Trajectory builder ────────────────────────────────────────────────────────

std::optional<dynamics::Trajectory> build_lane_change_trajectory(
    const Domain& domain,
    PlanningParams& planning_tools,
    size_t current_lane_id,
    size_t target_lane_id,
    const std::string& label)
{
  if (!domain.vehicle_state || !domain.map || !planning_tools.vehicle_model)
    return std::nullopt;

  const auto* cur_lane = find_lane(domain, current_lane_id);
  const auto* tgt_lane = find_lane(domain, target_lane_id);

  if (!cur_lane || !tgt_lane)
    return std::nullopt;

  const auto ego_pt = to_map_point(*domain.vehicle_state);

  auto cur_match = find_closest_center_point_on_lane(*cur_lane, ego_pt);
  auto tgt_match = find_closest_center_point_on_lane(*tgt_lane, ego_pt);

  if (!cur_match.has_value() || !tgt_match.has_value())
    return std::nullopt;

  const double v_now = std::max(0.0, domain.vehicle_state->vx);

  const double speed_limit =
      domain.map->get_lane_speed_limit(target_lane_id);

  const double min_lane_change_speed =
      std::min(kMinLaneChangeSpeedMs, speed_limit);

  const double target_speed =
      std::clamp(
          v_now + kLaneChangeSpeedBiasMs,
          min_lane_change_speed,
          speed_limit);

  const double transition_m =
      std::clamp(
          kTransitionDistFactor * target_speed,
          kMinTransitionMeters,
          kMaxTransitionMeters);

  constexpr double kStepM = 1.0;
  constexpr double kTargetLaneFollowM = 45.0;

  const double blend_lead_m =
      std::min(kBlendLeadMaxMeters, kBlendLeadFactor * transition_m);

  const size_t expected_waypoints =
      static_cast<size_t>((transition_m + kTargetLaneFollowM) / kStepM) + 2;

  std::vector<adore::map::MapPoint> waypoints;
  waypoints.reserve(expected_waypoints);

  double first_alpha_debug = -1.0;

  // ── Blended transition section ──
  for (double ds = kStepM; ds <= transition_m; ds += kStepM)
  {
    const double t =
        std::clamp(
            (ds + blend_lead_m) / transition_m,
            0.0,
            1.0);

    const double alpha = smooth_step(t);

    if (first_alpha_debug < 0.0)
      first_alpha_debug = alpha;

    const double source_s =
        advance_lane_s_along_travel(
            *cur_lane,
            cur_match->lane_s,
            ds);

    const double target_s =
        advance_lane_s_along_travel(
            *tgt_lane,
            tgt_match->lane_s,
            ds);

    auto source_pt = find_center_point_on_lane_at_s(*cur_lane, source_s);
    auto target_pt = find_center_point_on_lane_at_s(*tgt_lane, target_s);

    if (!source_pt.has_value() || !target_pt.has_value())
      break;

    adore::map::MapPoint wp = *target_pt;
    wp.x       = (1.0 - alpha) * source_pt->x + alpha * target_pt->x;
    wp.y       = (1.0 - alpha) * source_pt->y + alpha * target_pt->y;
    wp.max_speed = target_speed;

    waypoints.push_back(wp);
  }

  // ── Pure target-lane follow section ──
  for (double ds = transition_m + kStepM;
       ds <= transition_m + kTargetLaneFollowM;
       ds += kStepM)
  {
    const double target_s =
        advance_lane_s_along_travel(
            *tgt_lane,
            tgt_match->lane_s,
            ds);

    auto target_pt = find_center_point_on_lane_at_s(*tgt_lane, target_s);

    if (!target_pt.has_value())
      break;

    adore::map::MapPoint wp = *target_pt;
    wp.max_speed = target_speed;

    waypoints.push_back(wp);
  }

  if (waypoints.size() < 10)
    return std::nullopt;

  auto traj =
      planner::waypoints_to_trajectory(
          *domain.vehicle_state,
          waypoints,
          domain.traffic_participants,
          *planning_tools.vehicle_model,
          target_speed);

  traj.adjust_start_time(domain.vehicle_state->time);
  traj.label = label + " (direct blended trajectory)";

  RCLCPP_INFO(
      rclcpp::get_logger("decision_maker"),
      "Built direct lane-change trajectory: "
      "source_lane=%zu target_lane=%zu waypoints=%zu "
      "transition_m=%.1f blend_lead_m=%.1f first_alpha=%.2f "
      "target_speed=%.1f source_dist=%.2f target_dist=%.2f",
      current_lane_id,
      target_lane_id,
      waypoints.size(),
      transition_m,
      blend_lead_m,
      first_alpha_debug,
      target_speed,
      cur_match->distance,
      tgt_match->distance);

  return traj;
}

// ── Main lane-change state machine ────────────────────────────────────────────

Decision change_lane_request(
    const Domain& domain,
    PlanningParams& planning_tools,
    int desired_direction,
    const std::string& label)
{
  // ── Guard: essential data must be present ──
  if (!domain.vehicle_state || !domain.map || !domain.route.has_value())
  {
    return fallback_follow_or_standstill(
        domain,
        planning_tools,
        label + " (fallback: missing state/map/route)");
  }

  auto& lc = planning_tools.lane_change; // convenience alias

  // ── Already done for this direction ──────────────────────────────────────
  if (lc.done && lc.done_direction == desired_direction)
  {
    // If the route planner has not yet caught up to the new lane, hold the
    // target lane ourselves so the vehicle does not drift back.
    if (lc.done_lane_id.has_value())
    {
      const auto current_route_lane = get_current_route_lane_id(domain);
      const bool route_updated =
          current_route_lane.has_value() &&
          *current_route_lane == *lc.done_lane_id;

      if (!route_updated)
      {
        const size_t done_lane = *lc.done_lane_id;
        auto traj = build_lane_change_trajectory(
            domain, planning_tools, done_lane, done_lane, label);

        if (traj.has_value())
        {
          Decision out;
          out.trajectory        = std::move(*traj);
          out.trajectory->label = label + " (holding target lane, awaiting replan)";
          out.traffic_participant = make_default_participant(domain, planning_tools);
          out.assistance_request = false;
          return out;
        }
      }
    }

    auto out = follow_route(domain, planning_tools);
    if (out.trajectory)
      out.trajectory->label = label + " (already completed)";
    out.traffic_participant = make_default_participant(domain, planning_tools);
    return out;
  }

  // ── Stale done-state from a different direction ──
  if (lc.done && lc.done_direction != desired_direction)
    clear_lane_change_done_state(planning_tools);

  // ── Determine which physical lane the vehicle is currently in ──
  auto current_lane_id = find_current_driving_lane_id(domain);

  if (!current_lane_id.has_value())
  {
    // Mid-manoeuvre: we have stored IDs but momentarily lost lane detection.
    if (lc.switch_source_s.has_value() &&
        lc.source_lane_id.has_value() &&
        lc.target_lane_id.has_value())
    {
      RCLCPP_WARN(
          rclcpp::get_logger("decision_maker"),
          "%s: current lane not found mid-manoeuvre — "
          "building trajectory from stored IDs (src=%zu tgt=%zu)",
          label.c_str(),
          *lc.source_lane_id,
          *lc.target_lane_id);

      const size_t target_lane_id = *lc.target_lane_id;
      auto traj = build_lane_change_trajectory(
          domain, planning_tools, target_lane_id, target_lane_id, label);

      if (traj.has_value())
      {
        const auto ego_pt_lc = to_map_point(*domain.vehicle_state);
        auto src_dist_lc = distance_to_lane_center(domain, *lc.source_lane_id, ego_pt_lc);
        auto tgt_dist_lc = distance_to_lane_center(domain, *lc.target_lane_id, ego_pt_lc);

        if (tgt_dist_lc.has_value() &&
            src_dist_lc.has_value() &&
            *tgt_dist_lc < kTargetLaneReachedDistanceMeters &&
            *src_dist_lc > kSourceLaneLeftDistanceMeters)
        {
          mark_lane_change_done(planning_tools, desired_direction, *lc.target_lane_id);

          auto out = follow_route(domain, planning_tools);
          if (out.trajectory)
            out.trajectory->label = label + " (completed: settled in target, no current lane)";
          out.traffic_participant = make_default_participant(domain, planning_tools);
          return out;
        }

        Decision out;
        out.trajectory         = std::move(*traj);
        out.traffic_participant = make_default_participant(domain, planning_tools);
        out.assistance_request = false;
        return out;
      }

      // Trajectory build also failed — yield follow_route for one cycle.
      RCLCPP_WARN(
          rclcpp::get_logger("decision_maker"),
          "%s: trajectory build also failed mid-manoeuvre with no current lane "
          "— yielding follow_route for one cycle",
          label.c_str());

      auto out = follow_route(domain, planning_tools);
      if (out.trajectory)
        out.trajectory->label = label + " (hold: no current lane + trajectory failed)";
      out.traffic_participant = make_default_participant(domain, planning_tools);
      return out;
    }

    // Not mid-manoeuvre: clean up any stale partial state.
    if (lc.is_active())
      reset_lane_change_active_state(planning_tools);

    return fallback_follow_or_standstill(
        domain, planning_tools, label + " (no current lane)");
  }

  // ── Direction mismatch: a different direction was latched ──
  if (lc.target_lane_id.has_value() &&
      lc.direction != 0 &&
      lc.direction != desired_direction)
  {
    reset_lane_change_active_state(planning_tools);
    clear_lane_change_done_state(planning_tools);
  }

  lc.direction = desired_direction;

  auto route_lane_now = get_current_route_lane_id(domain);

  const bool direct_lane_change_active =
      lc.source_lane_id.has_value() &&
      lc.target_lane_id.has_value() &&
      lc.switch_source_s.has_value() &&
      lc.direction == desired_direction;

  // ── Stale source/target: check if route has moved past them ──
  if (!direct_lane_change_active &&
      lc.source_lane_id.has_value() &&
      lc.target_lane_id.has_value() &&
      route_lane_now.has_value())
  {
    const bool source_still_local =
        lane_is_in_current_route_corridor(domain, *lc.source_lane_id, *route_lane_now);

    const bool target_still_local =
        lane_is_in_current_route_corridor(domain, *lc.target_lane_id, *route_lane_now);

    if (!source_still_local && !target_still_local)
    {
      const bool vehicle_is_on_route_lane = (*current_lane_id == *route_lane_now);

      if (vehicle_is_on_route_lane)
      {
        // Back on the route lane — quietly discard stale latch.
        lc.reset_active();
      }
      else
      {
        // Still off-route but both lanes are gone — declare done by corridor handoff.
        auto out = follow_route(domain, planning_tools);
        if (out.trajectory)
          out.trajectory->label = label + " (completed by corridor handoff)";
        out.traffic_participant = make_default_participant(domain, planning_tools);
        mark_lane_change_done(planning_tools, desired_direction, *current_lane_id);
        return out;
      }
    }
  }

  // ── Compute ego distances to source and target lanes ──
  const auto ego_pt = to_map_point(*domain.vehicle_state);

  std::optional<double> source_dist;
  std::optional<double> target_dist;

  if (lc.source_lane_id.has_value())
    source_dist = distance_to_lane_center(domain, *lc.source_lane_id, ego_pt);

  if (lc.target_lane_id.has_value())
    target_dist = distance_to_lane_center(domain, *lc.target_lane_id, ego_pt);

  // ── Completion detection: current physical lane == target lane ──
  if (lc.target_lane_id.has_value() &&
      *current_lane_id == *lc.target_lane_id &&
      source_dist.has_value() &&
      target_dist.has_value() &&
      *source_dist > kSourceLaneLeftDistanceMeters &&
      *target_dist < kTargetLaneReachedDistanceMeters)
  {
    mark_lane_change_done(planning_tools, desired_direction, *lc.target_lane_id);

    auto out = follow_route(domain, planning_tools);
    if (out.trajectory)
      out.trajectory->label = label + " (completed by current lane detection)";
    out.traffic_participant = make_default_participant(domain, planning_tools);
    return out;
  }

  // ── Completion detection: route planner already reports target lane ──
  if (lc.target_lane_id.has_value() &&
      lc.source_lane_id.has_value() &&
      route_lane_now.has_value() &&
      *route_lane_now == *lc.target_lane_id)
  {
    mark_lane_change_done(planning_tools, desired_direction, *route_lane_now);

    auto out = follow_route(domain, planning_tools);
    if (out.trajectory)
      out.trajectory->label = label + " (completed by route handoff)";
    out.traffic_participant = make_default_participant(domain, planning_tools);
    return out;
  }

  // ── Completion detection: ego has settled inside target lane by distance ──
  if (lc.target_lane_id.has_value() &&
      lc.source_lane_id.has_value() &&
      target_dist.has_value() &&
      source_dist.has_value() &&
      *target_dist < kTargetLaneReachedDistanceMeters &&
      *source_dist > kSourceLaneLeftDistanceMeters)
  {
    mark_lane_change_done(planning_tools, desired_direction, *lc.target_lane_id);

    auto out = follow_route(domain, planning_tools);
    if (out.trajectory)
      out.trajectory->label = label + " (completed by target-lane settling)";
    out.traffic_participant = make_default_participant(domain, planning_tools);
    return out;
  }

  // ── Pick target lane (first cycle only) ──
  if (!lc.target_lane_id.has_value())
  {
    auto target_lane_id =
        choose_adjacent_target_lane_id(domain, *current_lane_id, desired_direction);

    if (!target_lane_id.has_value())
    {
      auto out = follow_route(domain, planning_tools);
      if (out.trajectory)
        out.trajectory->label = label + " (No Adjacent Lane Fallback)";
      out.traffic_participant = make_default_participant(domain, planning_tools);
      return out;
    }

    clear_lane_change_done_state(planning_tools);

    lc.target_lane_id = *target_lane_id;
    lc.source_lane_id = *current_lane_id;
    lc.switch_source_s.reset();
    lc.cached_route.reset();
    lc.direction = desired_direction;
  }

  const double route_s_now = domain.route->get_s(*domain.vehicle_state);

  // ── Topology guard: wait for a straight, simple stretch ──
  if (!lc.switch_source_s.has_value())
  {
    const bool topology_is_simple =
        is_simple_lane_change_topology(
            domain,
            *current_lane_id,
            *lc.target_lane_id,
            route_s_now);

    if (!topology_is_simple)
    {
      auto out = follow_route(domain, planning_tools);
      if (out.trajectory)
        out.trajectory->label = label + " (waiting for simpler local topology)";
      out.traffic_participant = make_default_participant(domain, planning_tools);
      return out;
    }
  }

  const size_t source_lane_for_planning =
      lc.source_lane_id.value_or(*current_lane_id);

  const size_t target_lane_for_planning = *lc.target_lane_id;

  // ── Sanity: source == target means we are already done ──
  if (source_lane_for_planning == target_lane_for_planning)
  {
    mark_lane_change_done(planning_tools, desired_direction, target_lane_for_planning);

    auto out = follow_route(domain, planning_tools);
    if (out.trajectory)
      out.trajectory->label = label + " (completed because source == target)";
    out.traffic_participant = make_default_participant(domain, planning_tools);
    return out;
  }

  // ── Latch the start position on the source lane (once per manoeuvre) ──
  if (!lc.switch_source_s.has_value())
  {
    const auto* src_lane_latch = find_lane(domain, source_lane_for_planning);
    if (src_lane_latch)
    {
      const auto ego_pt_latch = to_map_point(*domain.vehicle_state);
      auto cur_match_latch =
          find_closest_center_point_on_lane(*src_lane_latch, ego_pt_latch);
      if (cur_match_latch.has_value())
      {
        lc.switch_source_s = cur_match_latch->lane_s;
        RCLCPP_INFO(
            rclcpp::get_logger("decision_maker"),
            "%s: latched switch_s=%.2f on source_lane=%zu",
            label.c_str(),
            *lc.switch_source_s,
            source_lane_for_planning);
      }
    }
  }

  // ── Build the blended trajectory ──
  auto lane_change_traj =
      build_lane_change_trajectory(
          domain,
          planning_tools,
          source_lane_for_planning,
          target_lane_for_planning,
          label);

  if (!lane_change_traj.has_value())
  {
    // Near target or already past source → declare done despite trajectory failure.
    const bool near_target = target_dist.has_value() &&
        *target_dist < kTrajectoryFailRescueDistanceMeters;

    const bool left_source = source_dist.has_value() &&
        *source_dist > kSourceLaneLeftDistanceMeters;

    if ((near_target || left_source) && lc.target_lane_id.has_value())
    {
      RCLCPP_INFO(
          rclcpp::get_logger("decision_maker"),
          "%s: trajectory build failed but vehicle is near target "
          "(tgt_dist=%.2f src_dist=%.2f) — declaring done",
          label.c_str(),
          target_dist.value_or(-1.0),
          source_dist.value_or(-1.0));

      mark_lane_change_done(planning_tools, desired_direction, *lc.target_lane_id);

      auto out = follow_route(domain, planning_tools);
      if (out.trajectory)
        out.trajectory->label = label + " (completed: trajectory build failed near target)";
      out.traffic_participant = make_default_participant(domain, planning_tools);
      return out;
    }

    // Mid-manoeuvre failure: hold state and retry next cycle.
    if (lc.switch_source_s.has_value())
    {
      RCLCPP_WARN(
          rclcpp::get_logger("decision_maker"),
          "%s: trajectory build failed mid-manoeuvre "
          "(tgt_dist=%.2f src_dist=%.2f) — holding state for retry",
          label.c_str(),
          target_dist.value_or(-1.0),
          source_dist.value_or(-1.0));

      auto out = follow_route(domain, planning_tools);
      if (out.trajectory)
        out.trajectory->label = label + " (holding: trajectory build failed mid-manoeuvre)";
      out.traffic_participant = make_default_participant(domain, planning_tools);
      return out;
    }

    // Pre-manoeuvre failure: reset entirely and fall back.
    reset_lane_change_active_state(planning_tools);
    clear_lane_change_done_state(planning_tools);

    return fallback_follow_or_standstill(
        domain, planning_tools,
        label + " (failed to build direct lane-change trajectory)");
  }

  // ── Periodic debug logging (throttled — no static counter needed) ──
  auto traj = std::move(*lane_change_traj);

  if (!traj.states.empty())
  {
    const auto& first_state = traj.states.front();
    const auto& last_state  = traj.states.back();

    const double traj_start_to_ego =
        std::hypot(domain.vehicle_state->x - first_state.x,
                   domain.vehicle_state->y - first_state.y);

    const double traj_end_to_ego =
        std::hypot(domain.vehicle_state->x - last_state.x,
                   domain.vehicle_state->y - last_state.y);

    double max_traj_step   = 0.0;
    double max_heading_jump = 0.0;
    std::optional<double> prev_heading;

    for (size_t i = 1; i < traj.states.size(); ++i)
    {
      const auto& prev  = traj.states[i - 1];
      const auto& state = traj.states[i];

      const double dx   = state.x - prev.x;
      const double dy   = state.y - prev.y;
      const double step = std::hypot(dx, dy);

      max_traj_step = std::max(max_traj_step, step);

      if (step > 1e-6)
      {
        const double heading = std::atan2(dy, dx);
        if (prev_heading.has_value())
          max_heading_jump = std::max(max_heading_jump,
                                      std::abs(wrap_angle(heading - *prev_heading)));
        prev_heading = heading;
      }
    }

    static auto throttle_clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("decision_maker"),
        *throttle_clock,
        1000, // ms
        "%s direct traj: states=%zu start_to_ego=%.2f end_to_ego=%.2f "
        "max_traj_step=%.2f max_heading_jump_deg=%.1f "
        "src_dist=%.2f tgt_dist=%.2f",
        label.c_str(),
        traj.states.size(),
        traj_start_to_ego,
        traj_end_to_ego,
        max_traj_step,
        max_heading_jump * 180.0 / M_PI,
        source_dist.value_or(-1.0),
        target_dist.value_or(-1.0));
  }

  Decision out;
  out.trajectory         = std::move(traj);
  out.traffic_participant = make_default_participant(domain, planning_tools);
  out.assistance_request = false;
  return out;
}

} // anonymous namespace

// ── Public entry points ───────────────────────────────────────────────────────

Decision change_lane_left_request(
    const Domain& domain,
    PlanningParams& planning_tools)
{
  return change_lane_request(
      domain, planning_tools, kDesiredLeft, "Change Lane Left");
}

Decision change_lane_right_request(
    const Domain& domain,
    PlanningParams& planning_tools)
{
  return change_lane_request(
      domain, planning_tools, kDesiredRight, "Change Lane Right");
}

} // namespace adore::behaviours