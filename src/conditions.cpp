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
#include "rclcpp/rclcpp.hpp"

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
    // auto logger = rclcpp::get_logger("conditions");

    // if(!d.safety_corridor.has_value())
    // {
    //   RCLCPP_INFO(logger, "no safety corridor present");
    //   return false;
    // }
    // return true;
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

// passenger requests
bool
emergency_stop_requested( const Domain& d, const ConditionParams& )
{
  return d.emergency_stop_request;
}

bool stop_and_park_active( const Domain& d, const ConditionParams& )
{
  return d.stop_and_park_active;
}

static std::optional<size_t> current_lane_id(const Domain& d)
{
  if(!d.route || !d.vehicle_state) return std::nullopt;
  const auto& route = *d.route;
  double s = route.get_s(*d.vehicle_state);
  auto it = route.reference_line.lower_bound(s);
  if(it == route.reference_line.end()) it = std::prev(route.reference_line.end());
  return it->second.parent_id;
}

static double speed_limit_from_route_fallback(const Domain& d)
{
  const auto& route = *d.route;
  double s = route.get_s(*d.vehicle_state);

  auto it = route.reference_line.lower_bound(s);
  if(it == route.reference_line.end()) it = std::prev(route.reference_line.end());

  if(it->second.max_speed.has_value())
    return *it->second.max_speed;

  constexpr double kFallbackSpeedLimitMs = 13.9; // ≈ 50 km/h
  RCLCPP_WARN_ONCE(
      rclcpp::get_logger("conditions"),
      "No speed limit annotation on route — using fallback %.1f m/s",
      kFallbackSpeedLimitMs);
  return kFallbackSpeedLimitMs;
}

bool park_allowed_here( const Domain& d, const ConditionParams& )
{
  auto logger = rclcpp::get_logger("conditions");
  if (!d.map){
    RCLCPP_INFO(logger, "no valid map state");
  }
  if (!d.route){
    RCLCPP_INFO(logger, "no valid route state");
  }
  if (!d.vehicle_state){
    RCLCPP_INFO(logger, "no valid vehicle state");
  }
  if( !d.map || !d.route || !d.vehicle_state ){
    //RCLCPP_INFO(logger, "no valid vehicle state");
    return false;
  }

  auto lane_id_opt = current_lane_id(d);
  if( !lane_id_opt.has_value() ){
    //RCLCPP_INFO(logger, "no valid vehicle state");
    return false;
  }
  const size_t lane_id = *lane_id_opt;
  const double v_lim = speed_limit_from_route_fallback(d);

  if( v_lim > adore::map::DRIVING_SPEED_LIMIT_TOWN + 1e-3 ){
    //RCLCPP_INFO(logger, "speed limit to high");
    return false;
  }
  
  if( d.map->is_motorway_lane(lane_id) ){
    //RCLCPP_INFO(logger, "motorway");
    return false;
  }
  
  if( d.map->is_branch_lane(lane_id) ){
    //RCLCPP_INFO(logger, "intersection");
    return false;
  }


  return true;
}
bool resume_ride_requested(const Domain& d, const ConditionParams& )
{
  return d.resume_ride_active;
}
// bool safety_corridor_present( const Domain& d, const ConditionParams& )
// {
//   auto logger = rclcpp::get_logger("conditions");

//   if(!d.safety_corridor.has_value())
//   {
//     RCLCPP_INFO(logger, "no safety corridor present");
//     return false;
//   }
//   return true;
// }


bool lane_change_left_requested(
    const Domain& domain,
    const ConditionParams& /*params*/)
{ 
  
  return domain.lane_change_left_active;
}

bool lane_change_right_requested(
    const Domain& domain,
    const ConditionParams& /*params*/)
{
  return domain.lane_change_right_active;
}

} // namespace adore::conditions