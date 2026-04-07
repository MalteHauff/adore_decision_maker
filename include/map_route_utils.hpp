#pragma once

#include <memory>
#include <optional>

#include "adore_map/map.hpp"
#include "planning/trajectory_planner.hpp"

namespace adore::behaviours::map_utils
{

struct LaneCenterMatch
{
  adore::map::MapPoint point;
  double lane_s = 0.0;
  double distance = std::numeric_limits<double>::max();
};

double wrap_angle(double a);

double signed_lateral_offset_from_heading_angle(
    double heading,
    const adore::map::MapPoint& origin,
    const adore::map::MapPoint& point);

std::optional<LaneCenterMatch> find_closest_center_point_on_lane(
    const adore::map::Lane& lane,
    const adore::map::MapPoint& ref_pt);

std::optional<double> lane_heading_near_point(
    const adore::map::Lane& lane,
    const adore::map::MapPoint& ref_pt);

std::optional<size_t> lane_id_at_route_s(
    const adore::map::Route& route,
    double s);

double clamp_lane_s_to_bounds(
    const adore::map::Lane& lane,
    double lane_s);

double advance_lane_s_along_travel(
    const adore::map::Lane& lane,
    double start_s,
    double ds);

std::optional<adore::map::MapPoint> find_center_point_on_lane_at_s(
    const adore::map::Lane& lane,
    double lane_s);

std::optional<double> route_heading_at_s(
    const adore::map::Route& route,
    double s);

bool append_lane_interval_to_route(
    adore::map::Route& out,
    const std::shared_ptr<adore::map::Lane>& lane_ptr,
    double start_s,
    double end_s);

void append_route_tail(
    adore::map::Route& out,
    const adore::map::Route& tail);

void cap_route_speed_until_s(
    adore::map::Route& route,
    double vmax,
    double until_route_s);

} // namespace adore::behaviours::map_utils