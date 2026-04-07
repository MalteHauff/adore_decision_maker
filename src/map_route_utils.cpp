#include "map_route_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace adore::behaviours::map_utils
{

double wrap_angle(double a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double signed_lateral_offset_from_heading_angle(
    double heading,
    const adore::map::MapPoint& origin,
    const adore::map::MapPoint& point)
{
  const double dx = point.x - origin.x;
  const double dy = point.y - origin.y;
  return std::cos(heading) * dy - std::sin(heading) * dx;
}

std::optional<LaneCenterMatch> find_closest_center_point_on_lane(
    const adore::map::Lane& lane,
    const adore::map::MapPoint& ref_pt)
{
  if (lane.borders.center.interpolated_points.empty())
    return std::nullopt;

  LaneCenterMatch best;
  best.point = lane.borders.center.interpolated_points.front();
  best.lane_s = best.point.s;
  best.distance = adore::math::distance_2d(best.point, ref_pt);

  for (const auto& p : lane.borders.center.interpolated_points)
  {
    const double d = adore::math::distance_2d(p, ref_pt);
    if (d < best.distance)
    {
      best.point = p;
      best.lane_s = p.s;
      best.distance = d;
    }
  }

  return best;
}

std::optional<double> lane_heading_near_point(
    const adore::map::Lane& lane,
    const adore::map::MapPoint& ref_pt)
{
  if (lane.borders.center.interpolated_points.size() < 2)
    return std::nullopt;

  size_t best_idx = 0;
  double best_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < lane.borders.center.interpolated_points.size(); ++i)
  {
    double d = adore::math::distance_2d(lane.borders.center.interpolated_points[i], ref_pt);
    if (d < best_dist)
    {
      best_dist = d;
      best_idx = i;
    }
  }

  size_t i0 = best_idx;
  size_t i1 = best_idx;

  if (best_idx == 0)
    i1 = 1;
  else if (best_idx + 1 >= lane.borders.center.interpolated_points.size())
    i0 = best_idx - 1;
  else
  {
    i0 = best_idx - 1;
    i1 = best_idx + 1;
  }

  const auto& p0 = lane.borders.center.interpolated_points[i0];
  const auto& p1 = lane.borders.center.interpolated_points[i1];

  double dx = p1.x - p0.x;
  double dy = p1.y - p0.y;
  if (std::hypot(dx, dy) < 1e-3)
    return std::nullopt;

  return std::atan2(dy, dx);
}



std::optional<size_t> lane_id_at_route_s(
    const adore::map::Route& route,
    double s)
{
  if (route.reference_line.empty()) return std::nullopt;

  auto it = route.reference_line.lower_bound(s);
  if (it == route.reference_line.end())
    it = std::prev(route.reference_line.end());

  return it->second.parent_id;
}

double clamp_lane_s_to_bounds(
    const adore::map::Lane& lane,
    double lane_s)
{
  if (lane.borders.center.interpolated_points.empty())
    return lane_s;

  const double s0 = lane.borders.center.interpolated_points.front().s;
  const double s1 = lane.borders.center.interpolated_points.back().s;
  return std::clamp(lane_s, std::min(s0, s1), std::max(s0, s1));
}

double advance_lane_s_along_travel(
    const adore::map::Lane& lane,
    double start_s,
    double ds)
{
  return lane.left_of_reference ? (start_s - ds) : (start_s + ds);
}

std::optional<adore::map::MapPoint> find_center_point_on_lane_at_s(
    const adore::map::Lane& lane,
    double lane_s)
{
  if (lane.borders.center.interpolated_points.empty())
    return std::nullopt;

  const double s_query = clamp_lane_s_to_bounds(lane, lane_s);

  const adore::map::MapPoint* best = &lane.borders.center.interpolated_points.front();
  double best_err = std::abs(best->s - s_query);

  for (const auto& p : lane.borders.center.interpolated_points)
  {
    const double err = std::abs(p.s - s_query);
    if (err < best_err)
    {
      best = &p;
      best_err = err;
    }
  }

  return *best;
}

std::optional<double> route_heading_at_s(
    const adore::map::Route& route,
    double s)
{
  if(route.reference_line.size() < 2) return std::nullopt;

  auto it = route.reference_line.lower_bound(s);
  if(it == route.reference_line.end()) it = std::prev(route.reference_line.end());

  auto it_prev = it;
  auto it_next = it;

  if(it == route.reference_line.begin())
  {
    it_next = std::next(it);
  }
  else if(std::next(it) == route.reference_line.end())
  {
    it_prev = std::prev(it);
  }
  else
  {
    it_prev = std::prev(it);
    it_next = std::next(it);
  }

  const auto& p0 = it_prev->second;
  const auto& p1 = it_next->second;

  const double dx = p1.x - p0.x;
  const double dy = p1.y - p0.y;

  if(std::hypot(dx, dy) < 1e-3) return std::nullopt;
  return std::atan2(dy, dx);
}



bool append_lane_interval_to_route(
    adore::map::Route& out,
    const std::shared_ptr<adore::map::Lane>& lane_ptr,
    double start_s,
    double end_s)
{
  if (!lane_ptr) return false;

  auto start_pt = find_center_point_on_lane_at_s(*lane_ptr, start_s);
  auto end_pt   = find_center_point_on_lane_at_s(*lane_ptr, end_s);
  if (!start_pt.has_value() || !end_pt.has_value())
    return false;

  out.add_route_section(
      lane_ptr->borders.center,
      *start_pt,
      *end_pt,
      lane_ptr->left_of_reference);

  return true;
}



void append_route_tail(
    adore::map::Route& out,
    const adore::map::Route& tail)
{
  if (!tail.map) return;

  for (const auto& sec : tail.sections)
  {
    if (!sec) continue;

    auto lane_it = tail.map->lanes.find(sec->lane_id);
    if (lane_it == tail.map->lanes.end() || !lane_it->second)
      continue;

    append_lane_interval_to_route(out, lane_it->second, sec->start_s, sec->end_s);
  }
}


void cap_route_speed_until_s(
    adore::map::Route& route,
    double vmax,
    double until_route_s)
{
  for (auto& [s, mp] : route.reference_line)
  {
    if (s > until_route_s) break;

    if (mp.max_speed.has_value())
      mp.max_speed = std::min(*mp.max_speed, vmax);
    else
      mp.max_speed = vmax;
  }
}

} // namespace adore::behaviours::map_utils