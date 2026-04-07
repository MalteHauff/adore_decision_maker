#include "behaviours.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "behaviour_common.hpp"
#include "map_route_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "planning/planning_helpers.hpp"

namespace adore::behaviours
{
namespace
{

using common::fallback_follow_or_standstill;
using common::make_default_participant;
using common::to_map_point;

using map_utils::advance_lane_s_along_travel;
using map_utils::append_lane_interval_to_route;
using map_utils::append_route_tail;
using map_utils::cap_route_speed_until_s;
using map_utils::find_center_point_on_lane_at_s;
using map_utils::find_closest_center_point_on_lane;
using map_utils::lane_heading_near_point;
using map_utils::lane_id_at_route_s;
using map_utils::route_heading_at_s;
using map_utils::signed_lateral_offset_from_heading_angle;
using map_utils::wrap_angle;

constexpr int    kDesiredLeft               = +1;
constexpr int    kDesiredRight              = -1;
constexpr double kLaneSearchRadiusMeters    = 4.0;
constexpr double kLaneChangeLookaheadMeters = 60.0;
constexpr double kCurrentLaneMaxCenterDist  = 5.0;

struct AdjacentLaneCandidate
{
  size_t lane_id;
  double signed_lat;
  double heading_diff;
  double center_dist;
};

std::shared_ptr<adore::map::Map> get_route_map_ptr(const Domain& domain)
{
  if (domain.route.has_value() && domain.route->map)
    return domain.route->map;
  return nullptr;
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

void reset_lane_change_active_state(PlanningParams& planning_tools)
{
  planning_tools.lane_change_target_lane_id.reset();
  planning_tools.lane_change_source_lane_id.reset();
  planning_tools.lane_change_switch_source_s.reset();
  planning_tools.lane_change_direction = 0;
}

void clear_lane_change_done_state(PlanningParams& planning_tools)
{
  planning_tools.lane_change_done = false;
  planning_tools.lane_change_done_direction = 0;
  planning_tools.lane_change_done_lane_id.reset();
}

void mark_lane_change_done(
    PlanningParams& planning_tools,
    int direction,
    size_t reached_lane_id)
{
  reset_lane_change_active_state(planning_tools);
  planning_tools.lane_change_done = true;
  planning_tools.lane_change_done_direction = direction;
  planning_tools.lane_change_done_lane_id = reached_lane_id;
}

std::optional<size_t> find_current_driving_lane_id(const Domain& domain)
{
  if (!domain.map || !domain.vehicle_state) return std::nullopt;

  const auto ego_pt = to_map_point(*domain.vehicle_state);

  // Prefer the lane corridor of the currently published mission route.
  std::vector<size_t> candidate_lane_ids;

  if (domain.route.has_value() && !domain.route->reference_line.empty())
  {
    const double route_s = domain.route->get_s(*domain.vehicle_state);
    auto route_lane_id = lane_id_at_route_s(*domain.route, route_s);

    if (route_lane_id.has_value())
    {
      candidate_lane_ids.push_back(*route_lane_id);

      auto parallels = domain.map->get_parallel_lanes(*route_lane_id);
      candidate_lane_ids.insert(candidate_lane_ids.end(), parallels.begin(), parallels.end());

      RCLCPP_INFO(
          rclcpp::get_logger("decision_maker"),
          "route-aware lane search: route_lane=%zu corridor_candidates=%zu",
          *route_lane_id,
          candidate_lane_ids.size());
    }
  }

  // Fallback only if route corridor is unavailable
  if (candidate_lane_ids.empty())
  {
    candidate_lane_ids = domain.map->get_nearby_lane_ids(ego_pt, kLaneSearchRadiusMeters);
    RCLCPP_INFO(
        rclcpp::get_logger("decision_maker"),
        "fallback nearby_lane_ids count=%zu",
        candidate_lane_ids.size());
  }

  std::sort(candidate_lane_ids.begin(), candidate_lane_ids.end());
  candidate_lane_ids.erase(std::unique(candidate_lane_ids.begin(), candidate_lane_ids.end()), candidate_lane_ids.end());

  double best_dist = std::numeric_limits<double>::max();
  std::optional<size_t> best_lane_id;

  for (size_t lane_id : candidate_lane_ids)
  {
    auto it = domain.map->lanes.find(lane_id);
    if (it == domain.map->lanes.end() || !it->second) continue;

    const auto& lane = *it->second;
    if (lane.type != adore::map::LaneType::driving) continue;

    auto match = find_closest_center_point_on_lane(lane, ego_pt);
    if (!match.has_value()) continue;

    RCLCPP_INFO(
        rclcpp::get_logger("decision_maker"),
        "candidate current lane=%zu type=%d dist=%.2f",
        lane_id,
        static_cast<int>(lane.type),
        match->distance);

    if (match->distance < best_dist)
    {
      best_dist = match->distance;
      best_lane_id = lane_id;
    }
  }

  if (!best_lane_id.has_value() || best_dist > kCurrentLaneMaxCenterDist)
    return std::nullopt;

  RCLCPP_INFO(
      rclcpp::get_logger("decision_maker"),
      "selected current lane=%zu dist=%.2f",
      *best_lane_id,
      best_dist);

  return best_lane_id;
}

std::optional<size_t> choose_adjacent_target_lane_id(
    const Domain& domain,
    size_t current_lane_id,
    int desired_direction)
{
  if (!domain.map || !domain.vehicle_state) return std::nullopt;

  const auto ego_pt = to_map_point(*domain.vehicle_state);
  const auto candidate_lane_ids = domain.map->get_parallel_lanes(current_lane_id);

  auto current_it = domain.map->lanes.find(current_lane_id);
  if (current_it == domain.map->lanes.end() || !current_it->second) return std::nullopt;
  const auto& current_lane = *current_it->second;

  auto current_heading = lane_heading_near_point(current_lane, ego_pt);
  if (!current_heading.has_value()) return std::nullopt;

  RCLCPP_INFO(
      rclcpp::get_logger("decision_maker"),
      "current_lane=%zu parallel_candidates=%zu desired_direction=%d current_heading_deg=%.1f",
      current_lane_id,
      candidate_lane_ids.size(),
      desired_direction,
      *current_heading * 180.0 / M_PI);

  std::vector<AdjacentLaneCandidate> valid_candidates;

  for (size_t lane_id : candidate_lane_ids)
  {
    auto it = domain.map->lanes.find(lane_id);
    if (it == domain.map->lanes.end() || !it->second) continue;

    const auto& lane = *it->second;

    if (lane.type != adore::map::LaneType::driving) continue;
    if (domain.map->is_motorway_lane(lane_id)) continue;
    if (domain.map->is_branch_lane(lane_id)) continue;

    auto match = find_closest_center_point_on_lane(lane, ego_pt);
    if (!match.has_value()) continue;

    auto candidate_heading = lane_heading_near_point(lane, match->point);
    if (!candidate_heading.has_value()) continue;

    const double dh = std::abs(wrap_angle(*candidate_heading - *current_heading));
    const double signed_lat =
        signed_lateral_offset_from_heading_angle(*current_heading, ego_pt, match->point);
    const double lat_abs = std::abs(signed_lat);

    RCLCPP_INFO(
        rclcpp::get_logger("decision_maker"),
        "parallel lane=%zu type=%d motorway=%d branch=%d signed_lat=%.2f heading_diff_deg=%.1f dist=%.2f",
        lane_id,
        static_cast<int>(lane.type),
        domain.map->is_motorway_lane(lane_id),
        domain.map->is_branch_lane(lane_id),
        signed_lat,
        dh * 180.0 / M_PI,
        match->distance);

    // Must be strongly aligned with current lane
    if (dh > 15.0 * M_PI / 180.0)
      continue;

    // Must look like one neighboring lane, not same lane or multiple lanes away
    // Tune these bounds if your lane widths differ.
    if (lat_abs < 2.2 || lat_abs > 4.8)
      continue;

    if (desired_direction == kDesiredLeft)
    {
      if (signed_lat > 0.0)
      {
        valid_candidates.push_back({lane_id, signed_lat, dh, match->distance});
      }
    }
    else if (desired_direction == kDesiredRight)
    {
      if (signed_lat < 0.0)
      {
        valid_candidates.push_back({lane_id, signed_lat, dh, match->distance});
      }
    }
  }

  if (valid_candidates.empty())
  {
    RCLCPP_INFO(
        rclcpp::get_logger("decision_maker"),
        "No valid adjacent lane found in desired direction");
    return std::nullopt;
  }

  // Conservative rule: if there is more than one plausible candidate,
  // reject the lane change instead of guessing.
  if (valid_candidates.size() > 1)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("decision_maker"),
        "Ambiguous adjacent lane selection: %zu candidates -> reject lane change",
        valid_candidates.size());
    return std::nullopt;
  }

  const auto& best = valid_candidates.front();

  RCLCPP_INFO(
      rclcpp::get_logger("decision_maker"),
      "selected adjacent target lane=%zu signed_lat=%.2f",
      best.lane_id,
      best.signed_lat);

  return best.lane_id;
}

static bool is_simple_lane_change_topology(
    const Domain& domain,
    size_t current_lane_id,
    size_t target_lane_id,
    double route_s_now,
    double window_before = 8.0,
    double window_after  = 20.0)
{
  if (!domain.map || !domain.route || !domain.vehicle_state) return false;

  auto cur_it = domain.map->lanes.find(current_lane_id);
  auto tgt_it = domain.map->lanes.find(target_lane_id);
  if (cur_it == domain.map->lanes.end() || !cur_it->second) return false;
  if (tgt_it == domain.map->lanes.end() || !tgt_it->second) return false;

  const auto& cur_lane = *cur_it->second;
  const auto& tgt_lane = *tgt_it->second;

  if (cur_lane.type != adore::map::LaneType::driving) return false;
  if (tgt_lane.type != adore::map::LaneType::driving) return false;

  if (domain.map->is_motorway_lane(current_lane_id)) return false;
  if (domain.map->is_motorway_lane(target_lane_id)) return false;

  const auto ego_pt = to_map_point(*domain.vehicle_state);

  auto cur_match = find_closest_center_point_on_lane(cur_lane, ego_pt);
  auto tgt_match = find_closest_center_point_on_lane(tgt_lane, ego_pt);
  if (!cur_match.has_value() || !tgt_match.has_value()) return false;

  auto cur_heading = lane_heading_near_point(cur_lane, cur_match->point);
  auto tgt_heading = lane_heading_near_point(tgt_lane, tgt_match->point);
  if (!cur_heading.has_value() || !tgt_heading.has_value()) return false;

  const double dh = std::abs(wrap_angle(*tgt_heading - *cur_heading));
  if (dh > 15.0 * M_PI / 180.0) return false;

  const double signed_lat =
      signed_lateral_offset_from_heading_angle(*cur_heading, ego_pt, tgt_match->point);
  const double lat_abs = std::abs(signed_lat);

  if (lat_abs < 2.2 || lat_abs > 4.8) return false;

  const auto& route = *domain.route;
  const double s0 = std::max(0.0, route_s_now - window_before);
  const double s1 = std::min(route.get_length(), route_s_now + window_after);

  auto h0 = route_heading_at_s(route, s0);
  auto h1 = route_heading_at_s(route, s1);
  if (h0.has_value() && h1.has_value())
  {
    const double dh_route = std::abs(wrap_angle(*h1 - *h0));
    if (dh_route > 25.0 * M_PI / 180.0) return false;
  }

  return true;
}


static std::optional<adore::map::Route> build_lane_change_route(
    const Domain& domain,
    PlanningParams& planning_tools,
    size_t current_lane_id,
    size_t target_lane_id)
{
  if (!domain.vehicle_state || !domain.route) return std::nullopt;

  auto route_map = get_route_map_ptr(domain);
  if (!route_map) return std::nullopt;

  auto cur_it = route_map->lanes.find(current_lane_id);
  auto tgt_it = route_map->lanes.find(target_lane_id);
  if (cur_it == route_map->lanes.end() || !cur_it->second) return std::nullopt;
  if (tgt_it == route_map->lanes.end() || !tgt_it->second) return std::nullopt;

  const auto& cur_lane = *cur_it->second;
  const auto& tgt_lane = *tgt_it->second;
  const auto ego_pt = to_map_point(*domain.vehicle_state);

  auto cur_match = find_closest_center_point_on_lane(cur_lane, ego_pt);
  auto tgt_match = find_closest_center_point_on_lane(tgt_lane, ego_pt);
  if (!cur_match.has_value() || !tgt_match.has_value())
    return std::nullopt;

  const double v_now = std::max(0.0, domain.vehicle_state->vx);

  // Latch source lane once
  if (!planning_tools.lane_change_source_lane_id.has_value())
    planning_tools.lane_change_source_lane_id = current_lane_id;

  // Latch a fixed switch point once.
  // Keep it close so the car actually commits, but not exactly at the current pose.
  if (!planning_tools.lane_change_switch_source_s.has_value())
  {
    const double switch_ahead_m = std::clamp(2.0 + 0.3 * v_now, 3.0, 6.0);
    planning_tools.lane_change_switch_source_s =
        advance_lane_s_along_travel(cur_lane, cur_match->lane_s, switch_ahead_m);

    RCLCPP_INFO(
        rclcpp::get_logger("decision_maker"),
        "Latched lane-change switch point on source lane=%zu at s=%.2f",
        current_lane_id,
        *planning_tools.lane_change_switch_source_s);
  }

  const double switch_s = *planning_tools.lane_change_switch_source_s;

  auto cur_exit_pt = find_center_point_on_lane_at_s(cur_lane, switch_s);
  if (!cur_exit_pt.has_value())
    return std::nullopt;

  // Use same longitudinal position on target lane as route.cpp expects for PARALLEL transitions
  auto tgt_entry_pt = find_center_point_on_lane_at_s(tgt_lane, switch_s);
  if (!tgt_entry_pt.has_value())
    return std::nullopt;

  adore::map::Route out;
  out.map = route_map;
  out.start.x = domain.vehicle_state->x;
  out.start.y = domain.vehicle_state->y;
  out.destination = domain.route->destination;

  // 1) current lane only until the latched switch point
  if (!append_lane_interval_to_route(out, cur_it->second, cur_match->lane_s, cur_exit_pt->s))
    return std::nullopt;

  // 2) continue on target lane
  adore::map::Route tail(*tgt_entry_pt, domain.route->destination, route_map);

  if (tail.reference_line.empty())
  {
    // fallback: continue locally on target lane for some distance
    const double tgt_end_s =
        advance_lane_s_along_travel(tgt_lane, tgt_entry_pt->s, std::max(60.0, 6.0 * v_now + 35.0));

    if (!append_lane_interval_to_route(out, tgt_it->second, tgt_entry_pt->s, tgt_end_s))
      return std::nullopt;
  }
  else
  {
    append_route_tail(out, tail);
  }

  out.initialize_reference_line();
  if (out.reference_line.size() < 3)
    return std::nullopt;

  const double lane_change_vmax = std::min(5.0, route_map->get_lane_speed_limit(target_lane_id));
  cap_route_speed_until_s(out, lane_change_vmax, 30.0);

  return out;
}

Decision change_lane_request(
    const Domain& domain,
    PlanningParams& planning_tools,
    int desired_direction,
    const std::string& label)
{
  RCLCPP_INFO(rclcpp::get_logger("decision_maker"), "Entered %s", label.c_str());

  if (!domain.vehicle_state || !domain.map || !domain.route.has_value())
  {
    reset_lane_change_active_state(planning_tools);
    clear_lane_change_done_state(planning_tools);
    return fallback_follow_or_standstill(domain, planning_tools, label + " (fallback)");
  }

  if (!get_route_map_ptr(domain))
  {
    reset_lane_change_active_state(planning_tools);
    clear_lane_change_done_state(planning_tools);
    return fallback_follow_or_standstill(domain, planning_tools, label + " (missing route map)");
  }

  // Same command still active after already finishing once -> do not retrigger
  if (planning_tools.lane_change_done &&
      planning_tools.lane_change_done_direction == desired_direction)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("decision_maker"),
        "%s: command already completed previously -> suppress retrigger",
        label.c_str());

    auto out = follow_route(domain, planning_tools);
    if (out.trajectory) out.trajectory->label = label + " (already completed)";
    out.traffic_participant = make_default_participant(domain, planning_tools);
    return out;
  }

  // Opposite command clears old done-state
  if (planning_tools.lane_change_done &&
      planning_tools.lane_change_done_direction != desired_direction)
  {
    clear_lane_change_done_state(planning_tools);
  }

  auto current_lane_id = find_current_driving_lane_id(domain);
  
  if (!current_lane_id.has_value())
  {
    reset_lane_change_active_state(planning_tools);
    clear_lane_change_done_state(planning_tools);
    return fallback_follow_or_standstill(domain, planning_tools, label + " (no current lane)");
  }

  // New command direction cancels previous active lane change
  if (planning_tools.lane_change_target_lane_id.has_value() &&
      planning_tools.lane_change_direction != 0 &&
      planning_tools.lane_change_direction != desired_direction)
  {
    reset_lane_change_active_state(planning_tools);
    clear_lane_change_done_state(planning_tools);
  }

  planning_tools.lane_change_direction = desired_direction;

  // --- NEW: if mission control has already handed the global route over to a
  // different lane than the original source lane, finish the lane change here.
  auto route_lane_now = get_current_route_lane_id(domain);
  if (planning_tools.lane_change_target_lane_id.has_value() &&
      planning_tools.lane_change_source_lane_id.has_value() &&
      route_lane_now.has_value() &&
      *route_lane_now != *planning_tools.lane_change_source_lane_id)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("decision_maker"),
        "%s: route handoff detected (source_lane=%zu, route_lane=%zu) -> finishing lane change",
        label.c_str(),
        *planning_tools.lane_change_source_lane_id,
        *route_lane_now);

    mark_lane_change_done(planning_tools, desired_direction, *route_lane_now);

    auto out = follow_route(domain, planning_tools);
    if (out.trajectory) out.trajectory->label = label + " (completed by route handoff)";
    out.traffic_participant = make_default_participant(domain, planning_tools);
    return out;
  }

  // Exact target-lane-id completion still stays
  if (planning_tools.lane_change_target_lane_id.has_value() &&
      *current_lane_id == *planning_tools.lane_change_target_lane_id)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("decision_maker"),
        "%s: reached target lane %zu -> lane change complete",
        label.c_str(),
        *current_lane_id);

    mark_lane_change_done(planning_tools, desired_direction, *current_lane_id);

    auto out = follow_route(domain, planning_tools);
    if (out.trajectory) out.trajectory->label = label + " (completed)";
    out.traffic_participant = make_default_participant(domain, planning_tools);
    return out;
  }

  // Latch target lane once
  if (!planning_tools.lane_change_target_lane_id.has_value())
  {
    auto target_lane_id = choose_adjacent_target_lane_id(domain, *current_lane_id, desired_direction);
    if (!target_lane_id.has_value())
    {
      auto out = follow_route(domain, planning_tools);
      if (out.trajectory) out.trajectory->label = label + " (waiting for valid adjacent lane)";
      out.traffic_participant = make_default_participant(domain, planning_tools);
      return out;
    }

    clear_lane_change_done_state(planning_tools);

    planning_tools.lane_change_target_lane_id = *target_lane_id;
    planning_tools.lane_change_source_lane_id = *current_lane_id;
    planning_tools.lane_change_switch_source_s.reset();
    planning_tools.lane_change_direction = desired_direction;

    RCLCPP_INFO(
        rclcpp::get_logger("decision_maker"),
        "%s: current_lane=%zu target_lane=%zu",
        label.c_str(),
        *current_lane_id,
        *planning_tools.lane_change_target_lane_id);
  }
  const double route_s_now = domain.route->get_s(*domain.vehicle_state);

  if (!is_simple_lane_change_topology(
          domain,
          *current_lane_id,
          *planning_tools.lane_change_target_lane_id,
          route_s_now))
  {
    RCLCPP_INFO(
        rclcpp::get_logger("decision_maker"),
        "%s: rejecting lane change because local source-target topology is too complex here",
        label.c_str());

    auto out = follow_route(domain, planning_tools);
    if (out.trajectory) out.trajectory->label = label + " (waiting for simpler local topology)";
    out.traffic_participant = make_default_participant(domain, planning_tools);
    return out;
  }



  auto lane_change_route = build_lane_change_route(
      domain,
      planning_tools,
      *current_lane_id,
      *planning_tools.lane_change_target_lane_id);

  // IMPORTANT: check has_value() BEFORE dereferencing it
  if (!lane_change_route.has_value())
  {
    reset_lane_change_active_state(planning_tools);
    clear_lane_change_done_state(planning_tools);
    return fallback_follow_or_standstill(domain, planning_tools, label + " (failed to build lane-change route)");
  }

  if (lane_change_route->reference_line.size() < 10)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("decision_maker"),
        "%s: waiting because temporary lane-change route is too short (%zu ref points)",
        label.c_str(),
        lane_change_route->reference_line.size());

    auto out = follow_route(domain, planning_tools);
    if (out.trajectory) out.trajectory->label = label + " (waiting for longer clean target corridor)";
    out.traffic_participant = make_default_participant(domain, planning_tools);
    return out;
  }

  RCLCPP_INFO(
      rclcpp::get_logger("decision_maker"),
      "%s: source_lane=%zu current_lane=%zu target_lane=%zu switch_s=%.2f sections=%zu ref_points=%zu",
      label.c_str(),
      planning_tools.lane_change_source_lane_id.value_or(0),
      *current_lane_id,
      *planning_tools.lane_change_target_lane_id,
      planning_tools.lane_change_switch_source_s.value_or(-1.0),
      lane_change_route->sections.size(),
      lane_change_route->reference_line.size());

  RCLCPP_INFO(
      rclcpp::get_logger("decision_maker"),
      "%s: temporary lane-change route sections=%zu ref_points=%zu",
      label.c_str(),
      lane_change_route->sections.size(),
      lane_change_route->reference_line.size());

  auto traj = planning_tools.planner.plan_route_trajectory(
      *lane_change_route,
      *domain.vehicle_state,
      domain.traffic_participants);

  traj.adjust_start_time(domain.vehicle_state->time);
  traj.label = label;

  Decision out;
  out.trajectory = std::move(traj);
  out.traffic_participant = make_default_participant(domain, planning_tools);
  out.assistance_request = false;
  return out;
}
} // anonymous namespace

Decision change_lane_left_request(const Domain& domain, PlanningParams& planning_tools)
{
  return change_lane_request(domain, planning_tools, kDesiredLeft, "Change Lane Left");
}

Decision change_lane_right_request(const Domain& domain, PlanningParams& planning_tools)
{
  return change_lane_request(domain, planning_tools, kDesiredRight, "Change Lane Right");
}

} // namespace adore::behaviours