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
#include <deque>
#include <map>
#include <vector>
#include <memory>

#include "adore_dynamics_adapters.hpp"
#include "adore_dynamics_conversions.hpp"
#include "adore_map/traffic_light.hpp"
#include "adore_map_adapters.hpp"
#include "adore_ros2_msgs/msg/assistance_request.hpp"
#include "adore_ros2_msgs/msg/caution_zone.hpp"
#include "adore_ros2_msgs/msg/safety_corridor.hpp"
#include "adore_ros2_msgs/msg/state_monitor.hpp"
#include "adore_ros2_msgs/msg/traffic_signals.hpp"
#include "adore_ros2_msgs/msg/waypoints.hpp"
#include "adore_ros2_msgs/msg/mission_command.hpp"
#include "decision_types.hpp"
#include "std_msgs/msg/bool.hpp"

namespace adore
{
  namespace map {class Map;}
struct Domain
{
  void setup( rclcpp::Node& node, const DomainParams& params, const InTopics& topics );

  using CautionZones = std::map<std::string, math::Polygon2d>;

  // world
  std::optional<dynamics::VehicleStateDynamic>          vehicle_state;
  std::optional<map::Route>                             route;
  std::optional<dynamics::Trajectory>                   reference_trajectory;
  dynamics::TrafficParticipantSet                       traffic_participants;
  std::map<size_t, adore_ros2_msgs::msg::TrafficSignal> traffic_signals;
  std::optional<adore_ros2_msgs::msg::SafetyCorridor>   safety_corridor;
  std::optional<dynamics::Trajectory>                   suggested_trajectory;
  CautionZones                                          caution_zones;
  std::optional<adore_ros2_msgs::msg::Waypoints>        waypoints;
  std::optional<adore_ros2_msgs::msg::MissionCommand>   mission_command;
  bool                                                  suggested_trajectory_acceptance = false;

  // memory
  bool sent_assistance_request = false;
  bool emergency_stop_request = false;
  bool stop_and_park_active = false;
  bool resume_ride_active = false;
  bool lane_change_right_active = false;
  bool lane_change_left_active = false;

  std::shared_ptr<const map::Map> map;


  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers;

  template<typename MsgT, typename CallbackT>
  void
  add_subscription( rclcpp::Node& node, const std::string& topic_name, CallbackT&& callback )
  {
    auto sub = node.create_subscription<MsgT>( topic_name, 1, std::forward<CallbackT>( callback ) );
    subscribers.emplace_back( sub );
  }
};
} // namespace adore
