#include "behaviours.hpp"
#include "behaviour_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "planning/planning_helpers.hpp"

#include <algorithm>

namespace adore::behaviours
{
using common::make_default_participant;

Decision emergency_stop(const Domain& domain, PlanningParams& planning_tools)
{
  Decision out;
  dynamics::Trajectory emergency_stop_trajectory;
  if (domain.vehicle_state)
    emergency_stop_trajectory.states.push_back(domain.vehicle_state.value());

  emergency_stop_trajectory.label = "Emergency Stop";
  out.trajectory = std::move(emergency_stop_trajectory);
  out.traffic_participant = make_default_participant(domain, planning_tools);

  if (domain.emergency_stop_request)
    out.emergency_stop_requested = true;

  return out;
}

Decision standstill(const Domain& domain, PlanningParams& planning_tools)
{
  Decision out;
  dynamics::Trajectory standstill_trajectory;
  standstill_trajectory.label = "Standstill";

  if (domain.vehicle_state)
    standstill_trajectory.states.push_back(domain.vehicle_state.value());

  out.trajectory = std::move(standstill_trajectory);
  out.traffic_participant = make_default_participant(domain, planning_tools);
  return out;
}

Decision follow_reference(const Domain& domain, PlanningParams& planning_tools)
{
  Decision out;
  out.trajectory = *domain.reference_trajectory;
  out.trajectory->label = "Follow Reference";
  out.traffic_participant = make_default_participant(domain, planning_tools);
  return out;
}

Decision follow_route(const Domain& domain, PlanningParams& planning_tools)
{
  Decision out;
  auto route_with_signal = domain.route.value();

  for (auto& p : route_with_signal.reference_line)
  {
    if (std::any_of(domain.traffic_signals.begin(), domain.traffic_signals.end(),
                    [&](const auto& s)
                    {
                      return adore::math::distance_2d(s.second, p.second) < 3.0 &&
                             s.second.state != adore_ros2_msgs::msg::TrafficSignal::GREEN;
                    }))
    {
      p.second.max_speed = 0;
    }
  }

  auto traj = planning_tools.planner.plan_route_trajectory(
      route_with_signal,
      *domain.vehicle_state,
      domain.traffic_participants);

  traj.adjust_start_time(domain.vehicle_state->time);
  traj.label = "Follow Route";

  out.trajectory = std::move(traj);
  out.traffic_participant = make_default_participant(domain, planning_tools);
  return out;
}

Decision waiting_for_assistance(const Domain& domain, PlanningParams& planning_tools)
{
  Decision out = minimum_risk(domain, planning_tools);
  out.trajectory->label = "Waiting for Waypoints";

  if (domain.waypoints.has_value() && domain.waypoints->waypoints.size() > 1)
  {
    dynamics::Trajectory trajectory =
        planner::waypoints_to_trajectory(*domain.vehicle_state,
                                         domain.waypoints->waypoints,
                                         domain.traffic_participants,
                                         *planning_tools.vehicle_model);
    trajectory.label = "Suggested Trajectory";
    trajectory.adjust_start_time(domain.vehicle_state->time);

    out.trajectory_suggestion = std::move(trajectory);
    out.trajectory->label = "Waiting for Confirmation";
  }

  out.traffic_participant = make_default_participant(domain, planning_tools);
  return out;
}

Decision follow_assistance(const Domain& domain, PlanningParams& planning_tools)
{
  Decision out;

  dynamics::Trajectory trajectory =
      planner::waypoints_to_trajectory(*domain.vehicle_state,
                                       domain.waypoints->waypoints,
                                       domain.traffic_participants,
                                       *planning_tools.vehicle_model);

  trajectory.label = "Follow Assistance";
  trajectory.adjust_start_time(domain.vehicle_state->time);

  out.trajectory = std::move(trajectory);
  out.traffic_participant = make_default_participant(domain, planning_tools);
  out.assistance_request = false;
  return out;
}

Decision safety_corridor(const Domain& domain, PlanningParams& planning_tools)
{
  Decision out;

  auto right_forward_points =
      planner::filter_points_in_front(domain.safety_corridor->right_border,
                                      *domain.vehicle_state);

  auto safety_waypoints =
      planner::shift_points_right(right_forward_points,
                                  planning_tools.vehicle_model->params.body_width);

  double target_speed =
      planner::is_point_to_right_of_line(*domain.vehicle_state, right_forward_points)
          ? 0.0
          : 2.0;

  auto planned_trajectory =
      planner::waypoints_to_trajectory(*domain.vehicle_state,
                                       safety_waypoints,
                                       domain.traffic_participants,
                                       *planning_tools.vehicle_model,
                                       target_speed);

  planned_trajectory =
      planning_tools.planner.optimize_trajectory(*domain.vehicle_state, planned_trajectory);

  planned_trajectory.label = "Safety Corridor";
  out.trajectory = std::move(planned_trajectory);
  out.traffic_participant = make_default_participant(domain, planning_tools);
  return out;
}

Decision request_assistance(const Domain& domain, PlanningParams& planning_tools)
{
  Decision out = minimum_risk(domain, planning_tools);
  out.assistance_request = true;
  out.trajectory->label = "Request Assistance";
  out.traffic_participant = make_default_participant(domain, planning_tools);
  return out;
}

Decision minimum_risk(const Domain& domain, PlanningParams& planning_tools)
{
  Decision out;

  double state_s = domain.route->get_s(*domain.vehicle_state);
  auto cut_route = domain.route->get_shortened_route(state_s, 100.0);

  auto planned_trajectory =
      planner::waypoints_to_trajectory(*domain.vehicle_state,
                                       cut_route,
                                       domain.traffic_participants,
                                       *planning_tools.vehicle_model,
                                       0.0);

  planned_trajectory =
      planning_tools.planner.optimize_trajectory(*domain.vehicle_state, planned_trajectory);

  // if (planned_trajectory.states.size() < 2)
  // // planned_trajectory = planning_tools.planner.optimize_trajectory( *domain.vehicle_state, planned_trajectory );
  if( planned_trajectory.states.size() < 2 )
  {
    return standstill(domain, planning_tools);
  }
  planned_trajectory.label = "Minimum Risk Maneuver";
  out.trajectory = std::move(planned_trajectory);
  out.traffic_participant = make_default_participant(domain, planning_tools);
  return out;
}

// Decision waiting_for_safe_parking(const Domain& domain, PlanningParams& planning_tools)
// {
//   auto out = follow_route(domain, planning_tools);
//   if (out.trajectory)
//     out.trajectory->label = "Stop&Park: waiting for low-speed non-junction lane (V2A/V2B)";
//   out.traffic_participant = make_default_participant(domain, planning_tools);
//   return out;
// }

// Decision waiting_for_corridor(const Domain& domain, PlanningParams& planning_tools)
// {
//   auto out = follow_route(domain, planning_tools);
//   out.assistance_request = false;
//   if (out.trajectory)
//     out.trajectory->label = "Stop&Park: waiting for safety corridor";
//   out.traffic_participant = make_default_participant(domain, planning_tools);
//   return out;
// }

Decision resume_ride(const Domain& domain, PlanningParams& planning_tools)
{
  planning_tools.park_target_route_s.reset();

  planning_tools.lane_change_target_lane_id.reset();
  planning_tools.lane_change_source_lane_id.reset();
  planning_tools.lane_change_switch_source_s.reset();
  planning_tools.lane_change_direction = 0;
  planning_tools.lane_change_done = false;
  planning_tools.lane_change_done_direction = 0;
  planning_tools.lane_change_done_lane_id.reset();

  auto out = follow_route(domain, planning_tools);
  out.assistance_request = false;
  if (out.trajectory) out.trajectory->label = "Resume Ride";
  out.traffic_participant = make_default_participant(domain, planning_tools);
  return out;
}

} // namespace adore::behaviours