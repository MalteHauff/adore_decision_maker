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

#include "rclcpp/rclcpp.hpp"
#include "planning/planning_helpers.hpp" // your existing helpers

namespace adore::behaviours
{
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

} // namespace adore::behaviours
