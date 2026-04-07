#pragma once

#include <string>
#include "domain.hpp"
#include "planning/trajectory_planner.hpp"
#include "adore_map/map.hpp"
#include "dynamics/traffic_participant.hpp"

namespace adore::behaviours::common
{

adore::map::MapPoint to_map_point(const dynamics::VehicleStateDynamic& state);

Decision fallback_follow_or_standstill(
    const Domain& domain,
    PlanningParams& planning_tools,
    const std::string& label);

dynamics::TrafficParticipant make_default_participant(
    const Domain& domain,
    const PlanningParams& planning_tools);

} // namespace adore::behaviours::common