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
#include "decision_publisher.hpp"
#include "decision_types.hpp"
#include "domain.hpp"
#include "rules.hpp"
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace adore
{

class DecisionMaker : public rclcpp::Node
{
public:

  explicit DecisionMaker( const rclcpp::NodeOptions& opts );

private:

  conditions::ConditionMap condition_map = conditions::make_condition_map();
  behaviours::BehaviourMap behaviour_map = behaviours::make_behaviour_map();
  rules::Rules             rules;

  // subscribers and latest sensor snapshots are stored in domain
  Domain domain;
  // publishers
  DecisionPublisher publisher;


  DecisionParams               params;
  rclcpp::TimerBase::SharedPtr timer;
  std::mutex params_mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  rcl_interfaces::msg::SetParametersResult
  on_parameters_set(const std::vector<rclcpp::Parameter>& parameters);
  void setup();
  void run(); // main loop
};

} // namespace adore
