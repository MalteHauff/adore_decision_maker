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

#include "conditions.hpp"

namespace adore::conditions
{

bool
state_ok( const Domain& d, const ConditionParams& )
{
  return d.vehicle_state.has_value(); // TODO add covariance of estimate
}

bool
safety_corridor_present( const Domain& d, const ConditionParams& )
{
  return d.safety_corridor.has_value();
}

bool
waypoints_available( const Domain& d, const ConditionParams& )
{
  return d.waypoints.has_value() && d.waypoints->waypoints.size() > 0;
}

bool
reference_traj_valid( const Domain& d, const ConditionParams& p )
{
  if( !d.reference_trajectory )
    return false;

  if( d.reference_trajectory->states.size() < p.min_ref_traj_size )
    return false;

  double age = d.vehicle_state->time - d.reference_trajectory->states.front().time;
  return age <= p.max_ref_traj_age;
}

bool
route_available( const Domain& d, const ConditionParams& p )
{
  if( !d.route || !d.vehicle_state )
    return false;

  double remaining = d.route->get_length() - d.route->get_s( *d.vehicle_state );
  return remaining > p.min_route_length;
}

bool
need_assistance( const Domain& d, const ConditionParams& )
{
  // check if in a caution zone
  return std::any_of( d.caution_zones.begin(), d.caution_zones.end(),
                      [&]( const auto& zone ) { return zone.second.point_inside( *d.vehicle_state ); } );
}

bool
sent_assistance_request( const Domain& d, const ConditionParams& )
{
  return d.sent_assistance_request;
}

bool
suggested_trajectory_accepted( const Domain& d, const ConditionParams& )
{
  return d.suggested_trajectory_acceptance;
}
bool
emergency_stop_requested( const Domain& d, const ConditionParams& )
{
  return d.emergency_stop_request;
}


} // namespace adore::conditions