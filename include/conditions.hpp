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

#pragma once
#include <array>
#include <cstdint>

#include "decision_types.hpp"
#include "domain.hpp"

namespace adore::conditions
{


using ConditionFnPtr = bool ( * )( const Domain&, const ConditionParams& );
using ConditionMap   = std::unordered_map<std::string, ConditionFnPtr>;
using ConditionState = std::unordered_map<std::string, bool>;

//---------------------------------------------------------------
// Per‑flag check functions
//---------------------------------------------------------------

bool state_ok( const Domain& domain, const ConditionParams& params );
bool safety_corridor_present( const Domain& domain, const ConditionParams& params );
bool waypoints_available( const Domain& domain, const ConditionParams& params );
bool reference_traj_valid( const Domain& domain, const ConditionParams& params );
bool route_available( const Domain& domain, const ConditionParams& params );
bool need_assistance( const Domain& domain, const ConditionParams& params );
bool sent_assistance_request( const Domain& domain, const ConditionParams& params );
bool suggested_trajectory_accepted( const Domain& domain, const ConditionParams& params );
bool emergency_stop_requested( const Domain& domain, const ConditionParams& params );
bool stop_and_park_active( const Domain& domain, const ConditionParams& params );
bool park_allowed_here( const Domain& domain, const ConditionParams& params );
//bool safety_corridor_present( const Domain& domain, const ConditionParams& params );
bool resume_ride_requested(const Domain& domain, const ConditionParams& params);
bool lane_change_left_requested(const Domain& domain, const ConditionParams& params);
bool lane_change_right_requested(const Domain& domain, const ConditionParams& params);

inline ConditionMap
make_condition_map()
{
  return {
    {                      "state_ok",                      state_ok },
    {       "safety_corridor_present",       safety_corridor_present },
    {           "waypoints_available",           waypoints_available },
    {          "reference_traj_valid",          reference_traj_valid },
    {               "route_available",               route_available },
    {               "need_assistance",               need_assistance },
    {       "sent_assistance_request",       sent_assistance_request },
    { "suggested_trajectory_accepted", suggested_trajectory_accepted },
    {      "emergency_stop_requested",        emergency_stop_requested },
    {      "stop_and_park_active",        stop_and_park_active },
    {      "park_allowed_here",        park_allowed_here },
    {      "resume_ride_requested",        resume_ride_requested },
    {      "lane_change_left_requested",        lane_change_left_requested },
    {      "lane_change_right_requested",        lane_change_right_requested },
  };
}

inline ConditionState
evaluate_conditions( const Domain& domain, const ConditionParams& params, const ConditionMap& condition_map )
{
  ConditionState state;
  for( const auto& [name, fn] : condition_map )
    state[name] = fn( domain, params );
  return state;
}


} // namespace adore::conditions
