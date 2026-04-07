#include "behaviour_common.hpp"

#include "behaviours.hpp"
#include "planning/planning_helpers.hpp"

namespace adore::behaviours::common
{

adore::map::MapPoint to_map_point(const dynamics::VehicleStateDynamic& state)
{
  adore::map::MapPoint p;
  p.x = state.x;
  p.y = state.y;
  p.s = 0.0;
  p.parent_id = 0;
  return p;
}

Decision fallback_follow_or_standstill(
    const Domain& domain,
    PlanningParams& planning_tools,
    const std::string& label)
{
  Decision out =
      (domain.route.has_value() && domain.vehicle_state.has_value())
          ? behaviours::follow_route(domain, planning_tools)
          : behaviours::standstill(domain, planning_tools);

  if (out.trajectory) out.trajectory->label = label;
  out.traffic_participant = make_default_participant(domain, planning_tools);
  return out;
}

dynamics::TrafficParticipant make_default_participant(
    const Domain& domain,
    const PlanningParams& planning_tools)
{
  dynamics::TrafficParticipant participant;

  if (domain.vehicle_state)
    participant.state = domain.vehicle_state.value();

  if (domain.route)
  {
    participant.goal_point = domain.route->destination;
    participant.route = domain.route.value();
  }

  participant.id = planning_tools.v2x_id;
  participant.v2x_id = planning_tools.v2x_id;
  participant.classification = dynamics::CAR;
  participant.physical_parameters = planning_tools.vehicle_model->params;

  return participant;
}

} // namespace adore::behaviours::common