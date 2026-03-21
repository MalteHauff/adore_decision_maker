/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "behaviours.hpp"
#include "adore_map/map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "planning/planning_helpers.hpp" // your existing helpers
#include <cmath>
#include <unordered_set>
namespace adore::behaviours
{

enum class StopAndParkTargetKind
{
  ParkingOnly,
  ShoulderOnly,
  AnySafeFallback
};

struct StopAndParkTarget
{
  double route_s = 0.0;                       // approximate s along route for the target
  size_t target_lane_id = 0;                  // chosen parking / shoulder lane
  double target_lane_s = 0.0;                 // approximate s on target lane centerline
  adore::map::MapPoint target_point;          // actual point to aim for
  StopAndParkTargetKind kind;                 // parking / shoulder
  int score = std::numeric_limits<int>::min();
};

struct LaneCenterMatch
{
  adore::map::MapPoint point;
  double lane_s = 0.0;
  double distance = std::numeric_limits<double>::max();
};

static std::optional<LaneCenterMatch> find_closest_center_point_on_lane(
    const adore::map::Lane& lane,
    const adore::map::MapPoint& ref_pt)
{
  if (lane.borders.center.interpolated_points.empty())
    return std::nullopt;

  LaneCenterMatch best;
  best.point = lane.borders.center.interpolated_points.front();
  best.lane_s = best.point.s;
  best.distance = adore::math::distance_2d(best.point, ref_pt);

  for (const auto& p : lane.borders.center.interpolated_points)
  {
    const double d = adore::math::distance_2d(p, ref_pt);
    if (d < best.distance)
    {
      best.point = p;
      best.lane_s = p.s;
      best.distance = d;
    }
  }

  return best;
}

Decision
emergency_stop( const Domain& domain, PlanningParams& planning_tools )
{

  Decision             out;
  dynamics::Trajectory emergency_stop_trajectory;
  if( domain.vehicle_state )
    emergency_stop_trajectory.states.push_back( domain.vehicle_state.value() );
  emergency_stop_trajectory.label = "Emergency Stop";
  out.trajectory                  = std::move( emergency_stop_trajectory );
  out.traffic_participant         = make_default_participant( domain, planning_tools );
  RCLCPP_INFO( rclcpp::get_logger("decision_maker"), "Emergency Stop Behaviour Activated" );
  RCLCPP_INFO( rclcpp::get_logger("decision_maker"), "Out: %s", out.trajectory->label.c_str() );
  RCLCPP_INFO( rclcpp::get_logger("decision_maker"), "Out Trajectory: %s", out.trajectory->states.empty() ? "Empty" : "Non-Empty" );
  RCLCPP_INFO( rclcpp::get_logger("decision_maker"), "trajectory label: %s", out.trajectory->label.c_str() );
  if (domain.emergency_stop_request){
    out.emergency_stop_requested = true;
  }
  return out;
}

Decision
standstill( const Domain& domain, PlanningParams& planning_tools )
{

  Decision             out;
  dynamics::Trajectory standstill_trajectory;
  standstill_trajectory.label = "Standstill";
  if( domain.vehicle_state )
    standstill_trajectory.states.push_back( domain.vehicle_state.value() );
  out.trajectory          = std::move( standstill_trajectory );
  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
follow_reference( const Domain& domain, PlanningParams& planning_tools )
{
  Decision out;
  out.trajectory        = *domain.reference_trajectory;
  out.trajectory->label = "Follow Reference";

  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
follow_route( const Domain& domain, PlanningParams& planning_tools )
{
  Decision out;
  auto     route_with_signal = domain.route.value();
  for( auto& p : route_with_signal.reference_line )
  {
    if( std::any_of( domain.traffic_signals.begin(), domain.traffic_signals.end(), [&]( const auto& s ) {
          return adore::math::distance_2d( s.second, p.second ) < 3.0 && s.second.state != adore_ros2_msgs::msg::TrafficSignal::GREEN;
        } ) )
      p.second.max_speed = 0;
  }
  auto traj = planning_tools.planner.plan_route_trajectory( route_with_signal, *domain.vehicle_state, domain.traffic_participants );
  traj.adjust_start_time( domain.vehicle_state->time );
  traj.label              = "Follow Route";
  out.trajectory          = std::move( traj );
  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
waiting_for_assistance( const Domain& domain, PlanningParams& planning_tools )
{

  // if we have no waypoints, do nothing
  Decision out          = minimum_risk( domain, planning_tools );
  out.trajectory->label = "Waiting for Waypoints";

  if( domain.waypoints.has_value() && domain.waypoints->waypoints.size() > 1 )
  {
    // if we have waypoints, create trajectory to send
    dynamics::Trajectory trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, domain.waypoints->waypoints,
                                                                        domain.traffic_participants, *planning_tools.vehicle_model );
    trajectory.label                = "Suggested Trajectory";
    trajectory.adjust_start_time( domain.vehicle_state->time );
    out.trajectory_suggestion = std::move( trajectory );
    out.trajectory->label     = "Waiting for Confirmation";
  }

  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
follow_assistance( const Domain& domain, PlanningParams& planning_tools )
{

  Decision out;

  dynamics::Trajectory trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, domain.waypoints->waypoints,
                                                                      domain.traffic_participants, *planning_tools.vehicle_model );
  trajectory.label                = "Follow Assistance";
  trajectory.adjust_start_time( domain.vehicle_state->time );
  out.trajectory = std::move( trajectory );

  out.traffic_participant = make_default_participant( domain, planning_tools );
  out.assistance_request  = false;
  return out;
}

Decision
safety_corridor( const Domain& domain, PlanningParams& planning_tools )
{

  Decision out; // calculate trajectory getting out of safety corridor
  auto     right_forward_points = planner::filter_points_in_front( domain.safety_corridor->right_border, *domain.vehicle_state );
  auto     safety_waypoints     = planner::shift_points_right( right_forward_points, planning_tools.vehicle_model->params.body_width );
  double   target_speed         = planner::is_point_to_right_of_line( *domain.vehicle_state, right_forward_points ) ? 0 : 2.0;
  auto planned_trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, safety_waypoints, domain.traffic_participants,
                                                              *planning_tools.vehicle_model, target_speed );

  planned_trajectory = planning_tools.planner.optimize_trajectory( *domain.vehicle_state, planned_trajectory );

  planned_trajectory.label = "Safety Corridor";
  out.trajectory           = std::move( planned_trajectory );

  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
request_assistance( const Domain& domain, PlanningParams& planning_tools )
{

  Decision out            = minimum_risk( domain, planning_tools );
  out.assistance_request  = true;
  out.trajectory->label   = "Request Assistance";
  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
minimum_risk( const Domain& domain, PlanningParams& planning_tools )
{

  Decision out;

  double state_s            = domain.route->get_s( *domain.vehicle_state );
  auto   cut_route          = domain.route->get_shortened_route( state_s, 100.0 );
  auto   planned_trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, cut_route, domain.traffic_participants,
                                                                *planning_tools.vehicle_model, 0.0 /* target_speed */ );

  planned_trajectory = planning_tools.planner.optimize_trajectory( *domain.vehicle_state, planned_trajectory );
  if( planned_trajectory.states.size() < 2 )
  {
    out = standstill( domain, planning_tools );
  }
  planned_trajectory.label = "Minimum Risk Maneuver";
  out.trajectory           = std::move( planned_trajectory );

  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}


static std::optional<size_t> lane_id_at_route_s(const adore::map::Route& route, double s)
{
  if(route.reference_line.empty()) return std::nullopt;
  auto it = route.reference_line.lower_bound(s);
  if(it == route.reference_line.end()) it = std::prev(route.reference_line.end());
  return it->second.parent_id;
}

static std::optional<adore::map::MapPoint> route_point_at_s(
    const adore::map::Route& route,
    double s)
{
  if(route.reference_line.empty()) return std::nullopt;

  auto it = route.reference_line.lower_bound(s);
  if(it == route.reference_line.end()) it = std::prev(route.reference_line.end());
  return it->second;
}

static double wrap_angle(double a)
{
  while(a > M_PI)  a -= 2.0 * M_PI;
  while(a < -M_PI) a += 2.0 * M_PI;
  return a;
}
static std::optional<double> route_heading_at_s(
    const adore::map::Route& route,
    double s)
{
  if(route.reference_line.size() < 2) return std::nullopt;

  auto it = route.reference_line.lower_bound(s);
  if(it == route.reference_line.end()) it = std::prev(route.reference_line.end());

  auto it_prev = it;
  auto it_next = it;

  if(it == route.reference_line.begin())
  {
    it_next = std::next(it);
  }
  else if(std::next(it) == route.reference_line.end())
  {
    it_prev = std::prev(it);
  }
  else
  {
    it_prev = std::prev(it);
    it_next = std::next(it);
  }

  const auto& p0 = it_prev->second;
  const auto& p1 = it_next->second;

  const double dx = p1.x - p0.x;
  const double dy = p1.y - p0.y;

  if(std::hypot(dx, dy) < 1e-3) return std::nullopt;
  return std::atan2(dy, dx);
}


static bool is_simple_stop_zone(
    const Domain& domain,
    double target_route_s,
    double window_before = 15.0,
    double window_after = 15.0,
    double nearby_lane_radius = 6.0)
{
  if(!domain.route || !domain.map) return false;
  if(domain.route->reference_line.empty()) return false;

  const auto& route = *domain.route;
  auto logger = rclcpp::get_logger("decision_maker");

  const double s0 = std::max(0.0, target_route_s - window_before);
  const double s1 = std::min(route.get_length(), target_route_s + window_after);

  std::unordered_set<size_t> unique_lane_ids;
  std::unordered_set<size_t> unique_road_ids;

  size_t sampled_points = 0;
  size_t max_nearby_candidates = 0;

  for(const auto& [route_s, mp] : route.reference_line)
  {
    if(route_s < s0) continue;
    if(route_s > s1) break;

    ++sampled_points;

    const size_t lane_id = mp.parent_id;
    unique_lane_ids.insert(lane_id);

    auto lane_it = domain.map->lanes.find(lane_id);
    if(lane_it == domain.map->lanes.end() || !lane_it->second)
    {
      RCLCPP_INFO(logger,
                  "Stop&Park zone reject at s=%.2f: missing lane object for lane=%zu",
                  target_route_s, lane_id);
      return false;
    }

    const auto& lane = *lane_it->second;
    unique_road_ids.insert(lane.road_id);

    // reject obvious complex / invalid stop zones
    if(domain.map->is_branch_lane(lane_id))
    {
      RCLCPP_INFO(logger,
                  "Stop&Park zone reject at s=%.2f: branch lane in zone (lane=%zu)",
                  target_route_s, lane_id);
      return false;
    }

    if(domain.map->is_motorway_lane(lane_id))
    {
      RCLCPP_INFO(logger,
                  "Stop&Park zone reject at s=%.2f: motorway lane in zone (lane=%zu)",
                  target_route_s, lane_id);
      return false;
    }

    if(!domain.map->is_low_speed_lane(lane_id))
    {
      RCLCPP_INFO(logger,
                  "Stop&Park zone reject at s=%.2f: non-low-speed lane in zone (lane=%zu)",
                  target_route_s, lane_id);
      return false;
    }

    auto nearby = domain.map->get_nearby_lane_ids(mp, nearby_lane_radius);
    max_nearby_candidates = std::max(max_nearby_candidates, nearby.size() + 1); // + base lane

    if(nearby.size() + 1 > 4)
    {
      RCLCPP_INFO(logger,
                  "Stop&Park zone reject at s=%.2f: too many nearby lanes (%zu)",
                  target_route_s, nearby.size() + 1);
      return false;
    }
  }

  if(sampled_points < 2)
  {
    RCLCPP_INFO(logger,
                "Stop&Park zone reject at s=%.2f: not enough route samples in zone",
                target_route_s);
    return false;
  }

  // reject topologically unstable zones
  if(unique_road_ids.size() > 2)
  {
    RCLCPP_INFO(logger,
                "Stop&Park zone reject at s=%.2f: too many roads in zone (%zu)",
                target_route_s, unique_road_ids.size());
    return false;
  }

  // reject high-curvature zones such as roundabouts / tight connectors
  auto h0 = route_heading_at_s(route, s0);
  auto h1 = route_heading_at_s(route, s1);
  if(h0.has_value() && h1.has_value())
  {
    const double dh = std::abs(wrap_angle(*h1 - *h0));
    const double max_heading_change = 30.0 * M_PI / 180.0;

    if(dh > max_heading_change)
    {
      RCLCPP_INFO(logger,
                  "Stop&Park zone reject at s=%.2f: heading change too large (%.1f deg)",
                  target_route_s, dh * 180.0 / M_PI);
      return false;
    }
  }

  RCLCPP_INFO(logger,
              "Stop&Park zone accept at s=%.2f: samples=%zu max_nearby=%zu roads=%zu",
              target_route_s, sampled_points, max_nearby_candidates, unique_road_ids.size());

  return true;
}

static int score_stop_and_park_lane(
    const Domain& domain,
    size_t lane_id,
    StopAndParkTargetKind kind)
{
  if(!domain.map) return -1000;

  // V2A / V2B constraints for the final stopping place
  if(!domain.map->is_low_speed_lane(lane_id)) return -1000;
  if(domain.map->is_branch_lane(lane_id))     return -1000;

  switch(kind)
  {
    case StopAndParkTargetKind::ParkingOnly:
      return domain.map->is_parking_lane(lane_id) ? 300 : -1000;

    case StopAndParkTargetKind::ShoulderOnly:
      return domain.map->is_shoulder_lane(lane_id) ? 200 : -1000;

    case StopAndParkTargetKind::AnySafeFallback:
      if(domain.map->is_parking_lane(lane_id))  return 300;
      if(domain.map->is_shoulder_lane(lane_id)) return 200;
      return 100;
  }

  return -1000;
}


static std::optional<StopAndParkTarget> find_stop_and_park_target_via_lane_graph(
    const Domain& domain,
    double s_now,
    double min_search_dist,
    double max_search_dist,
    StopAndParkTargetKind kind)
{
  if (!domain.route || !domain.map) return std::nullopt;
  if (domain.route->reference_line.empty()) return std::nullopt;

  const auto& route = *domain.route;
  auto logger = rclcpp::get_logger("decision_maker");

  std::optional<StopAndParkTarget> best_target;

  // Keep this local search fairly tight to avoid pulling in lots of unrelated roads.
  constexpr double nearby_lane_radius = 6.0;

  for (const auto& [route_s, mp] : route.reference_line)
  {
    if (route_s < s_now + min_search_dist) continue;
    if (route_s > s_now + max_search_dist) break;

    const size_t base_lane_id = mp.parent_id;

    std::vector<size_t> candidates;
    candidates.push_back(base_lane_id);

    auto nearby = domain.map->get_nearby_lane_ids(mp, nearby_lane_radius);
    candidates.insert(candidates.end(), nearby.begin(), nearby.end());

    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());

    RCLCPP_INFO(
        logger,
        "route_s=%.2f base_lane=%zu nearby_candidates=%zu",
        route_s,
        base_lane_id,
        candidates.size());

    for (size_t lane_id : candidates)
    {
      auto lane_it = domain.map->lanes.find(lane_id);
      if (lane_it == domain.map->lanes.end()) continue;

      const auto& lane = *lane_it->second;
      const auto lane_type = lane.type;

      RCLCPP_INFO(
          logger,
          "  raw nearby lane=%zu road=%zu type=%d left_of_reference=%d",
          lane_id,
          lane.road_id,
          static_cast<int>(lane_type),
          static_cast<int>(lane.left_of_reference));

      // Match only the requested kind.
      bool interesting = false;

      if (kind == StopAndParkTargetKind::ParkingOnly)
      {
        interesting = (lane_type == adore::map::LaneType::parking);
      }
      else if (kind == StopAndParkTargetKind::ShoulderOnly)
      {
        interesting = (lane_type == adore::map::LaneType::shoulder);
      }
      else if (kind == StopAndParkTargetKind::AnySafeFallback)
      {
        interesting = true;
      }

      if (!interesting)
        continue;

      auto match_opt = find_closest_center_point_on_lane(lane, mp);
      if (!match_opt.has_value())
        continue;

      const auto& match = *match_opt;

      // Optional sanity limit: reject lanes that are still too far away,
      // even if some center points entered the quadtree radius.
      if (match.distance > nearby_lane_radius + 1.0)
        continue;

      if (!is_simple_stop_zone(domain, route_s))
      {
        RCLCPP_INFO(
            logger,
            "    -> rejected candidate lane=%zu because route stop zone at s=%.2f is not simple",
            lane_id,
            route_s);
        continue;
      }

      RCLCPP_INFO(
          logger,
          "    -> accepted candidate lane=%zu road=%zu type=%d lane_s=%.2f dist=%.2f",
          lane_id,
          lane.road_id,
          static_cast<int>(lane_type),
          match.lane_s,
          match.distance);

      int score = score_stop_and_park_lane(domain, lane_id, kind);
      if (score < 0) continue;

      // Prefer closer opportunities somewhat.
      int distance_penalty = static_cast<int>(route_s - s_now);
      int total_score = score - distance_penalty / 5;

      if (!best_target.has_value() || total_score > best_target->score)
      {
        StopAndParkTarget candidate;
        candidate.route_s = route_s;
        candidate.target_lane_id = lane_id;
        candidate.target_lane_s = match.lane_s;
        candidate.target_point = match.point;
        candidate.kind = kind;
        candidate.score = total_score;

        best_target = candidate;

        RCLCPP_INFO(
            logger,
            "    -> new best target: route_s=%.2f lane=%zu road=%zu type=%d "
            "target=(%.2f, %.2f) lane_s=%.2f total_score=%d",
            candidate.route_s,
            candidate.target_lane_id,
            lane.road_id,
            static_cast<int>(lane_type),
            candidate.target_point.x,
            candidate.target_point.y,
            candidate.target_lane_s,
            candidate.score);
      }
    }
  }

  return best_target;
}

Decision stop_and_park_request(const Domain& domain, PlanningParams& planning_tools)
{
  if (!domain.vehicle_state || !domain.route)
  {
    RCLCPP_WARN(rclcpp::get_logger("decision_maker"),
                "Cannot execute stop_and_park behaviour: missing vehicle state or route");
    auto out = standstill(domain, planning_tools);
    out.trajectory->label = "Stop & Park (Fallback Standstill)";
    return out;
  }
  //RCLCPP_INFO( rclcpp::get_logger("decision_maker"), "start stop and park command");

  const auto& route = *domain.route;
  const double s_now = route.get_s(*domain.vehicle_state);
  const double v_now = domain.vehicle_state->vx;   // <-- fix

  // latch stop target once
  if (!planning_tools.park_target_route_s.has_value())
  {
    const auto& cs = *planning_tools.comfort_settings;
    const double decel = std::max(0.1, -cs.min_acceleration);

    const double d_brake  = (v_now * v_now) / (2.0 * decel);
    const double buffer_m = 5.0 + 0.5 * std::abs(v_now);
    const double min_search_dist = std::max(15.0, d_brake + buffer_m);

    const double parking_search_dist  = std::min(300.0, route.get_length() - s_now);
    const double shoulder_search_dist = std::min(std::max(100.0, min_search_dist + 40.0), route.get_length() - s_now);

    double s_target = std::min(s_now + min_search_dist, route.get_length() - 0.5);

    bool found_special_target = false;

    if(domain.map)
    {
      auto parking_target = find_stop_and_park_target_via_lane_graph(
          domain,
          s_now,
          min_search_dist,
          parking_search_dist,
          StopAndParkTargetKind::ParkingOnly);

      auto shoulder_target = find_stop_and_park_target_via_lane_graph(
          domain,
          s_now,
          min_search_dist,
          shoulder_search_dist,
          StopAndParkTargetKind::ShoulderOnly);

      // auto fallback_target = find_stop_and_park_target_via_lane_graph(
      //     domain,
      //     s_now,
      //     min_search_dist,
      //     shoulder_search_dist,
      //     StopAndParkTargetKind::AnySafeFallback);

      if(parking_target.has_value())
      {
        s_target = std::min(parking_target->route_s, route.get_length() - 0.5);
        found_special_target = true;
        RCLCPP_INFO(rclcpp::get_logger("decision_maker"),
                    "Stop&Park (parking slot): selected parking target at s=%.2f", s_target);
      }
      else if(shoulder_target.has_value())
      {
        s_target = std::min(shoulder_target->route_s, route.get_length() - 0.5);
        found_special_target = true;
        RCLCPP_INFO(rclcpp::get_logger("decision_maker"),
                    "Stop&Park (shoulder): selected shoulder target at s=%.2f", s_target);
      }
      // else if(fallback_target.has_value())
      // {
      //   s_target = std::min(fallback_target->route_s, route.get_length() - 0.5);
      //   found_special_target = true;
      //   RCLCPP_INFO(rclcpp::get_logger("decision_maker"),
      //               "Stop&Park (fallback): selected safe fallback target at s=%.2f", s_target);
      // }
    }

    if(!found_special_target)
    {
      // only allow pure braking-distance fallback if that route zone is simple too
      if(!is_simple_stop_zone(domain, s_target))
      {
        RCLCPP_INFO(rclcpp::get_logger("decision_maker"),
                    "Stop&Park: no valid simple stop zone found yet -> keep following route");
        return waiting_for_safe_parking(domain, planning_tools);
      }

      RCLCPP_INFO(rclcpp::get_logger("decision_maker"),
                  "Stop&Park: using simple-zone braking fallback at s=%.2f", s_target);
    }
    // if(!found_special_target)
    // {
    //   RCLCPP_INFO(rclcpp::get_logger("decision_maker"),
    //               "Stop&Park: no validated parking/shoulder target found yet -> keep following route");
    //   return waiting_for_safe_parking(domain, planning_tools);
    // }

    planning_tools.park_target_route_s = s_target;
  }

  const double s_target = *planning_tools.park_target_route_s;  // <-- fix
  const double remaining = s_target - s_now;

  // parked hold
  if (std::abs(v_now) < 0.2 && std::abs(remaining) < 1.0)
  {
    //RCLCPP_INFO( rclcpp::get_logger("decision_maker"), "continue with standstill");

    auto out2 = standstill(domain, planning_tools);
    out2.trajectory->label = "Parked";
    return out2;
  }

  // force 0 speed after target
  auto route_mod = domain.route.value();
  for (auto& [route_s, mp] : route_mod.reference_line)
  {
    if (route_s >= s_target)
      mp.max_speed = 0.0;
  }

  auto traj = planning_tools.planner.plan_route_trajectory(
    route_mod, *domain.vehicle_state, domain.traffic_participants);

  traj.adjust_start_time(domain.vehicle_state->time);
  traj.label = "Stop & Park (Stopping)";
  //RCLCPP_INFO( rclcpp::get_logger("decision_maker"), "execute stop and park command");
  Decision out;
  out.trajectory = std::move(traj);
  out.traffic_participant = make_default_participant(domain, planning_tools);
  return out;
}


Decision waiting_for_safe_parking(const Domain& domain, PlanningParams& planning_tools)
{
  auto out = follow_route(domain, planning_tools);
  if(out.trajectory) out.trajectory->label = "Stop&Park: waiting for low-speed non-junction lane (V2A/V2B)";
  out.traffic_participant = make_default_participant(domain, planning_tools);
  return out;
}

Decision waiting_for_corridor(const Domain& domain, PlanningParams& planning_tools)
{
  RCLCPP_INFO( rclcpp::get_logger("decision_maker"), "waiting for corridor");
  auto out = follow_route(domain, planning_tools);
  out.assistance_request = false;
  if(out.trajectory) out.trajectory->label = "Stop&Park: waiting for safety corridor";
  out.traffic_participant = make_default_participant(domain, planning_tools);
  return out;
}


dynamics::TrafficParticipant
make_default_participant( const Domain& domain, const PlanningParams& planning_tools )
{
  dynamics::TrafficParticipant participant;
  if( domain.vehicle_state )
    participant.state = domain.vehicle_state.value();
  if( domain.route )
  {
    participant.goal_point = domain.route->destination;
    participant.route      = domain.route.value();
  }
  participant.id                  = planning_tools.v2x_id;
  participant.v2x_id              = planning_tools.v2x_id;
  participant.classification      = dynamics::CAR;
  participant.physical_parameters = planning_tools.vehicle_model->params;
  return participant;
}

Decision resume_ride(const Domain& domain, PlanningParams& planning_tools)
{
  planning_tools.park_target_route_s.reset();

  auto out = follow_route(domain, planning_tools);
  out.assistance_request = false;
  if(out.trajectory) out.trajectory->label = "Resume Ride";
  out.traffic_participant = make_default_participant(domain, planning_tools);
  return out;
}

} // namespace adore::behaviours
