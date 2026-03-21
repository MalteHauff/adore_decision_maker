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

#include "domain.hpp"

#include "adore_dynamics_conversions.hpp"
#include "adore_map_conversions.hpp"
#include "adore_math_conversions.hpp"

namespace adore
{

void
Domain::setup( rclcpp::Node& n, const DomainParams& params, const InTopics& topics )
{
  using namespace adore_ros2_msgs;

  for( auto& tp_topic : { topics.sensor_participants, topics.infrastructure_participants } )
  {
    add_subscription<ParticipantSetAdapter>( n, tp_topic, [&]( const dynamics::TrafficParticipantSet& participants ) {
      for( const auto& [id, participant] : participants.participants )
      {
        if( participant.id != params.v2x_id )
          traffic_participants.update_traffic_participants( participant );
        if( participant.trajectory && participant.id == params.v2x_id )
          reference_trajectory = participant.trajectory;
        traffic_participants.remove_old_participants( params.max_participant_age, n.now().seconds() );
      }
    } );
  }

  add_subscription<StateAdapter>( n, topics.state, [&]( const dynamics::VehicleStateDynamic& msg ) { vehicle_state = msg; } );

  add_subscription<RouteAdapter>( n, topics.route, [&]( const map::Route& msg ) { route = msg; } );

  add_subscription<msg::SafetyCorridor>( n, topics.safety_corridor, [&]( const msg::SafetyCorridor& msg ) { safety_corridor = msg; } );

  add_subscription<TrajectoryAdapter>( n, topics.suggested_trajectory,
                                       [&]( const dynamics::Trajectory& msg ) { suggested_trajectory = msg; } );

  add_subscription<TrajectoryAdapter>( n, topics.reference_trajectory,
                                       [&]( const dynamics::Trajectory& msg ) { reference_trajectory = msg; } );

  add_subscription<std_msgs::msg::Bool>( n, topics.suggested_trajectory_acceptance,
                                         [&]( const std_msgs::msg::Bool& msg ) { suggested_trajectory_acceptance = msg.data; } );

  add_subscription<msg::CautionZone>( n, topics.caution_zones, [&]( const msg::CautionZone& msg ) {
    caution_zones[msg.label] = math::conversions::to_cpp_type( msg.polygon );
  } );

  add_subscription<msg::Waypoints>( n, topics.waypoints, [&]( const msg::Waypoints& msg ) { waypoints = msg; } );

  add_subscription<msg::TrafficSignal>( n, topics.traffic_signals,
                                        [&]( const msg::TrafficSignal& msg ) { traffic_signals[msg.signal_group_id] = msg; } );

  // subscribe to assistance request
  add_subscription<msg::AssistanceRequest>( n, topics.assistance_request,
                                            [&]( const msg::AssistanceRequest& msg ) { sent_assistance_request = msg.assistance_needed; } );

  //subriptions for passenger request
  add_subscription<std_msgs::msg::Bool>( n, topics.emergency_stop_request,
                                         [&]( const std_msgs::msg::Bool& msg ) {emergency_stop_request = msg.data;} );

  add_subscription<msg::MissionCommand>(
    n, topics.mission_command,
    [&](const msg::MissionCommand& msg) { mission_command = msg; }
  );
}


} // namespace adore
