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

#include "decision_maker.hpp"
#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include "adore_map/map_loader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace adore
{

DecisionMaker::DecisionMaker( const rclcpp::NodeOptions& opts ) :
  rclcpp::Node{ "decision_maker", opts }

{
  setup();
  domain.setup( *this, params.domain_params, params.in_topics );
  publisher.setup( *this, params.out_topics );
}

void
DecisionMaker::run()
{

  std::lock_guard<std::mutex> lk(params_mutex_);
  set_passenger_request_flags();
  auto     condition_state = conditions::evaluate_conditions( domain, params.condition_params, condition_map );
  auto     behaviour       = rules::choose_behaviour( condition_state, rules );
  Decision decision        = behaviour_map[behaviour.value()]( domain, params.planning_params );
  publisher.publish( *this, decision );
  
}

void
DecisionMaker::setup()
{
  params = load_params( *this );

  std::string map_file = declare_parameter( "map_file", std::string("") );
  bool map_allow_lane_changes = declare_parameter( "map_allow_lane_changes", true );
  bool map_ignoring_non_driving = declare_parameter( "map_ignoring_non_driving", false );
  RCLCPP_INFO(get_logger(), "map_file parameter = '%s'", map_file.c_str());
  if (!map_file.empty())
  {
    try{
      auto m = adore::map::MapLoader::load_from_file(
          map_file,
          map_allow_lane_changes,
          map_ignoring_non_driving
      );
      domain.map = std::make_shared<const map::Map>( std::move(m) );
      RCLCPP_INFO(get_logger(), "Map loaded successfully from %s", map_file.c_str());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to load map from %s: %s", map_file.c_str(), e.what());
    }
    
  }
  else
  {
    RCLCPP_WARN(get_logger(), "No map file specified. Domain will operate without a map.");
  }

  for (const auto& [lane_id, lane_ptr] : domain.map->lanes)
  {
    if (!lane_ptr) continue;
    if (lane_ptr->type != adore::map::LaneType::parking) continue;

    RCLCPP_INFO(
        rclcpp::get_logger("decision_maker"),
        "parking lane: id=%zu road=%zu left_of_reference=%d center_pts=%zu",
        lane_id,
        lane_ptr->road_id,
        static_cast<int>(lane_ptr->left_of_reference),
        lane_ptr->borders.center.interpolated_points.size());
  }

  param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&DecisionMaker::on_parameters_set, this, std::placeholders::_1)
    );
  timer = create_wall_timer( std::chrono::milliseconds( static_cast<int>( params.run_delta_time * 1000 ) ),
                             std::bind( &DecisionMaker::run, this ) );

  const std::string           pkg        = ament_index_cpp::get_package_share_directory( "decision_maker" );
  const std::filesystem::path rules_path = std::filesystem::path( pkg ) / "config" / "rules.yaml";

  std::string rules_file = declare_parameter( "rules_file", rules_path.string() );
  rules                  = rules::load_rules_yaml( rules_file );
}

rcl_interfaces::msg::SetParametersResult
DecisionMaker::on_parameters_set(const std::vector<rclcpp::Parameter>& ps)
{
  std::lock_guard<std::mutex> lk(params_mutex_);

  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;

  bool comfort_changed = false;
  bool planner_settings_changed = false;

  std::vector<std::string> keys;
  std::vector<double> values;
  this->get_parameter("planner_settings_keys", keys);
  this->get_parameter("planner_settings_values", values);

  auto& cs = *params.planning_params.comfort_settings;

  for (const auto& p : ps)
  {
    const auto& n = p.get_name();


    if      (n == "comfort.max_speed")                { cs.max_speed = p.as_double(); comfort_changed = true; }
    else if (n == "comfort.max_acceleration")         { cs.max_acceleration = p.as_double(); comfort_changed = true; }
    else if (n == "comfort.min_acceleration")         { cs.min_acceleration = p.as_double(); comfort_changed = true; }
    else if (n == "comfort.max_lateral_acceleration") { cs.max_lateral_acceleration = p.as_double(); comfort_changed = true; }
    else if (n == "comfort.speed_fraction_of_limit")  { cs.speed_fraction_of_limit = p.as_double(); comfort_changed = true; }
    else if (n == "comfort.headway_scale")            { cs.headway_scale = p.as_double(); comfort_changed = true; }
    else if (n == "comfort.time_headway")             { cs.time_headway = p.as_double(); comfort_changed = true; }
    else if (n == "comfort.distance_headway")         { cs.distance_headway = p.as_double(); comfort_changed = true; }
    else if (n == "planner_settings_keys")   { keys = p.as_string_array(); planner_settings_changed = true; }
    else if (n == "planner_settings_values") { values = p.as_double_array(); planner_settings_changed = true; }
    else if (n == "gps_sigma_ok")      { params.condition_params.gps_sigma_ok = p.as_double(); }
    else if (n == "max_ref_traj_age")  { params.condition_params.max_ref_traj_age = p.as_double(); }
    else if (n == "min_ref_traj_size") { params.condition_params.min_ref_traj_size = static_cast<size_t>(p.as_int()); }
    else if (n == "min_route_length")  { params.condition_params.min_route_length = static_cast<size_t>(p.as_int()); }
    else if (n == "debug")             { params.debug = p.as_bool(); }
  }

  if (planner_settings_changed)
  {
    if (keys.size() != values.size())
    {
      res.successful = false;
      res.reason = "planner_settings_keys and planner_settings_values must have same length";
      return res;
    }

    params.planning_params.planner_settings.clear();
    for (size_t i = 0; i < keys.size(); ++i)
      params.planning_params.planner_settings[keys[i]] = values[i];

    params.planning_params.planner.set_parameters(params.planning_params.planner_settings);
  }

  if (comfort_changed)
  {
    cs.clamp(params.planning_params.vehicle_model->params);
    params.planning_params.planner.set_comfort_settings(params.planning_params.comfort_settings);
  }

  return res;
}

void DecisionMaker::set_passenger_request_flags()
{
  //RCLCPP_INFO(get_logger(), "set passenger request flags");
  if (domain.mission_command.has_value())
  {
    const auto& cmd = *domain.mission_command;

    if (cmd.command_id != last_command_id)
    {
      last_command_id = cmd.command_id;
      //RCLCPP_INFO(get_logger(), "proceed command id: %d", cmd.command);
      switch (cmd.command)
      {
        case adore_ros2_msgs::msg::MissionCommand::STOP_AND_PARK:
          park_active = true;
          domain.stop_and_park_active = true;
          domain.resume_ride_active = false;
          park_target_route_s.reset();
          params.planning_params.park_target_route_s.reset();
          break;

        case adore_ros2_msgs::msg::MissionCommand::RESUME_RIDE:
          park_active = false;
          domain.stop_and_park_active = false;
          domain.resume_ride_active = true;
          park_target_route_s.reset();
          params.planning_params.park_target_route_s.reset();
          break;
        case adore_ros2_msgs::msg::MissionCommand::KEEP_MORE_DISTANCE:
          // map to comfort.headway_scale using cmd.enable/cmd.arg_float
          break;

        default:
          break;
      }
    }
  }
}


} // namespace adore

/* Register as component --------------------------------------------- */
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::DecisionMaker )
