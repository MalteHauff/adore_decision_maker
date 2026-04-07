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
#include <optional>

#include "adore_dynamics_conversions.hpp"
#include "conditions.hpp"
#include "behaviour_common.hpp"
#include "decision_types.hpp"
#include "domain.hpp"
#include "dynamics/comfort_settings.hpp"
#include "planning/trajectory_planner.hpp"
#include <unordered_map>

namespace adore
{

namespace behaviours
{


[[nodiscard]] Decision emergency_stop( const Domain&, PlanningParams& );
[[nodiscard]] Decision standstill( const Domain&, PlanningParams& );
[[nodiscard]] Decision follow_reference( const Domain&, PlanningParams& );
[[nodiscard]] Decision follow_assistance( const Domain&, PlanningParams& );
[[nodiscard]] Decision waiting_for_assistance( const Domain&, PlanningParams& );
[[nodiscard]] Decision follow_route( const Domain&, PlanningParams& );
[[nodiscard]] Decision safety_corridor( const Domain&, PlanningParams& );
[[nodiscard]] Decision request_assistance( const Domain&, PlanningParams& );
[[nodiscard]] Decision minimum_risk( const Domain&, PlanningParams& );
[[nodiscard]] Decision stop_and_park_request( const Domain&, PlanningParams& );
[[nodiscard]] Decision waiting_for_safe_parking( const Domain&, PlanningParams& );
[[nodiscard]] Decision waiting_for_corridor(const Domain&, PlanningParams&);
[[nodiscard]] Decision resume_ride(const Domain&, PlanningParams&);
[[nodiscard]] Decision change_lane_left_request(const Domain&, PlanningParams&);
[[nodiscard]] Decision change_lane_right_request(const Domain&, PlanningParams&);


// create map lookup
using BehaviourFnPtr = Decision ( * )( const Domain&, PlanningParams& );

using BehaviourMap = std::unordered_map<std::string, BehaviourFnPtr>;

inline BehaviourMap
make_behaviour_map()
{
  using namespace behaviours;
  return {
    {         "emergency_stop",         &emergency_stop },
    {             "standstill",             &standstill },
    {       "follow_reference",       &follow_reference },
    {      "follow_assistance",      &follow_assistance },
    { "waiting_for_assistance", &waiting_for_assistance },
    {           "follow_route",           &follow_route },
    {        "safety_corridor",        &safety_corridor },
    {     "request_assistance",     &request_assistance },
    {           "minimum_risk",           &minimum_risk },
    { "waiting_for_safe_parking", &waiting_for_safe_parking },
    { "emergency_stop_requested", &emergency_stop},
    { "stop_and_park_request", &stop_and_park_request},
    { "waiting_for_corridor", &waiting_for_corridor },
    { "resume_ride", &resume_ride },
    { "change_lane_left_request", &change_lane_left_request },
    { "change_lane_right_request", &change_lane_right_request },
  };
}

// helper for making participant
// dynamics::TrafficParticipant make_default_participant( const Domain& domain, const PlanningParams& planning_tools );

} // namespace behaviours
}// namespace adore
