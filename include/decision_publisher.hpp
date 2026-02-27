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
#include "adore_dynamics_adapters.hpp"

#include "behaviours.hpp"
#include "conditions.hpp"
#include "domain.hpp"
#include "rules.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace adore
{

struct DecisionPublisher
{
  rclcpp::Publisher<TrajectoryAdapter>::SharedPtr                       trajectory_publisher;
  rclcpp::Publisher<TrajectoryAdapter>::SharedPtr                       trajectory_suggestion_publisher;
  rclcpp::Publisher<adore_ros2_msgs::msg::AssistanceRequest>::SharedPtr assistance_publisher;
  rclcpp::Publisher<ParticipantAdapter>::SharedPtr                      traffic_participant_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                   emergency_stop_started_publisher;
  void                                                                  setup( rclcpp::Node& node, const OutTopics& topics );
  void                                                                  publish( const rclcpp::Node& node, const Decision& decision );
};
} // namespace adore