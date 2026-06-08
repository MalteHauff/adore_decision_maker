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
#include "adore_ros2_msgs/msg/odd.hpp"
#include "adore_ros2_msgs/msg/traffic_participant.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/passenger_request.hpp"
#include "behaviors.hpp"
#include "conditions.hpp"

namespace adore
{

DecisionMaker::DecisionMaker( const rclcpp::NodeOptions& opts ) :
  rclcpp::Node{ "decision_maker", opts }

{
  load_parameters();
  setup_subscribers();
  setup_publishers();
}

void DecisionMaker::load_parameters()
{  
  v2x_id = declare_parameter( "v2x_id", v2x_id );

  std::vector<std::string> keys   = declare_parameter( "planner_settings_keys", std::vector<std::string>{} );
  std::vector<double>      values = declare_parameter( "planner_settings_values", std::vector<double>{} );

  std::map<std::string, double> planner_settings;
  if( keys.size() == values.size() )
  {
    for( size_t i = 0; i < keys.size(); ++i )
    {
      planner_settings[keys[i]] = values[i];
    }
  }

  std::string vehicle_model_file = declare_parameter( "vehicle_model_file", "" );

  auto vehicle_model    = std::make_shared<dynamics::PhysicalVehicleModel>( vehicle_model_file, false );
  auto comfort_settings = dynamics::ComfortSettings(); // default value comfort settings

  planner.set_vehicle_parameters( vehicle_model->params );
  planner.set_comfort_settings( comfort_settings );
  planner.set_parameters( planner_settings );
}

void DecisionMaker::setup_subscribers()
{
  timer = create_wall_timer( std::chrono::milliseconds( static_cast<int>( 100 ) ), // 10 Hz
                             std::bind( &DecisionMaker::timer_callback, this ) );

  subscriber_vehicle_state_dynamic = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>( "vehicle_state_dynamic", 1,
                                      [this](const adore_ros2_msgs::msg::VehicleStateDynamic& msg) {  latest_vehicle_state_dynamic = dynamics::conversions::to_cpp_type(msg); });

  subscriber_route = create_subscription<adore_ros2_msgs::msg::Route>( "route", 1,
                                      [this](const adore_ros2_msgs::msg::Route& msg) {  latest_route = map::conversions::to_cpp_type(msg); });

  subscriber_odd = create_subscription<adore_ros2_msgs::msg::Odd>( "odd", 1,
                                      [this](const adore_ros2_msgs::msg::Odd& msg) {  latest_odd = msg; });

  subscriber_traffic_participants = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>( "traffic_participants", 1,
                                      [this](const adore_ros2_msgs::msg::TrafficParticipantSet& msg) 
                                      {  
                                        auto participants = dynamics::conversions::to_cpp_type(msg);
                                        for( const auto& [id, participant] : participants.participants )
                                        {
                                            traffic_participants.update_traffic_participants( participant );
                                        }

                                        double max_participant_age = 1.0;
                                        traffic_participants.remove_old_participants( max_participant_age, now().seconds() );
                                      });

  subscriber_v2x_traffic_participants = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>( "/infrastructure/planned_traffic", 1,
                                      [this](const adore_ros2_msgs::msg::TrafficParticipantSet& msg) 
                                      {  
                                        auto participants = dynamics::conversions::to_cpp_type(msg);

                                        if ( participants.validity_area.has_value() )
                                        {
                                          latest_managed_zone = participants.validity_area;
                                        }
                                        
                                        for( const auto& [id, participant] : participants.participants )
                                        {
                                          if ( !participant.v2x_id.has_value() || v2x_id != participant.v2x_id.value())
                                          {
                                            traffic_participants.update_traffic_participants( participant );
                                            continue;
                                          }

                                          if ( participant.trajectory.has_value() )
                                          {
                                            latest_managed_trajectory = participant.trajectory; 
                                          }
                                        }

                                        double max_participant_age = 1.0;
                                        traffic_participants.remove_old_participants( max_participant_age, now().seconds() );
                                      });

  subscriber_reference_trajectory = create_subscription<adore_ros2_msgs::msg::Trajectory>( "reference_trajectory", 1,
                                      [this](const adore_ros2_msgs::msg::Trajectory& msg) { latest_reference_trajectory = dynamics::conversions::to_cpp_type(msg); });

  subscriber_traffic_signal = create_subscription<adore_ros2_msgs::msg::TrafficSignal>( "traffic_signals", 1,
                                      [this](const adore_ros2_msgs::msg::TrafficSignal& msg) { traffic_signals[msg.signal_group_id] = msg; });

  subscriber_safety_corridor = create_subscription<adore_ros2_msgs::msg::SafetyCorridor>( "safety_corridor", 1,
                                      [this](const adore_ros2_msgs::msg::SafetyCorridor& msg) { latest_safety_corridor = msg; });

  subscriber_caution_zones = create_subscription<adore_ros2_msgs::msg::CautionZone>( "caution_zones", 1,
                                      [this](const adore_ros2_msgs::msg::CautionZone& msg) {  caution_zones[msg.label] = math::conversions::to_cpp_type(msg.polygon); });

  subscriber_weather = create_subscription<adore_ros2_msgs::msg::Weather>( "weather", 1,
                                      [this](const adore_ros2_msgs::msg::Weather& msg) {  latest_weather = msg; });

  subscriber_remote_operator_drive_approval = create_subscription<std_msgs::msg::Bool>( "suggested_trajectory_accepted", 1,
                                      [this](const std_msgs::msg::Bool& msg) {  remote_operator_drive_approval = msg.data; });

  subscriber_suggested_remote_operator_trajectory = create_subscription<adore_ros2_msgs::msg::Trajectory>( "suggested_remote_operator_trajectory", 1,
                                      [this](const adore_ros2_msgs::msg::Trajectory& msg) { 

                                        if ( !latest_vehicle_state_dynamic.has_value() )
                                          return;

                                        suggested_remote_operator_trajectory = dynamics::conversions::to_cpp_type(msg); 
                                        suggested_remote_operator_trajectory.value().adjust_start_time( latest_vehicle_state_dynamic.value().time );
                                       });


  //passenger requests
                                      
  subscriber_passenger_request =
    create_subscription<adore_ros2_msgs::msg::PassengerRequest>(
        "passenger_request", 10,
        [this]( const adore_ros2_msgs::msg::PassengerRequest& msg ) {
            handle_passenger_request( msg );
        } );
  subscriber_mission_command =
    create_subscription<adore_ros2_msgs::msg::MissionCommand>(
        "mission_command", 10,
        [this]( const adore_ros2_msgs::msg::MissionCommand& msg ) {
            handle_mission_command( msg );
        } );
}

void DecisionMaker::setup_publishers()
{
  publisher_trajectory_decision = create_publisher<adore_ros2_msgs::msg::Trajectory>( "trajectory_decision", 1 );
  publisher_alternative_trajectory_decision = create_publisher<adore_ros2_msgs::msg::Trajectory>( "alternative_trajectory_decision", 1 );
  publisher_v2x_traffic_participant = create_publisher<adore_ros2_msgs::msg::TrafficParticipant>( "v2x_traffic_participant", 1 );
}

void DecisionMaker::timer_callback()
{
  auto behavior = choose_and_plan_driving_behavior();
  publisher_trajectory_decision->publish(behavior.trajectory);

  if ( behavior.alternative_trajectory.has_value() )
  {
    publisher_alternative_trajectory_decision->publish(behavior.alternative_trajectory.value());
  }

  // @TODO, add publisher and behavior for signals

  publisher_v2x_traffic_participant->publish( make_default_participant() );
  if (passenger_emergency_stop &&
      latest_vehicle_state_dynamic.has_value() &&
      latest_vehicle_state_dynamic->vx < 0.05)
  {
      passenger_emergency_stop = false;
      RCLCPP_INFO(get_logger(), "Vehicle stopped — passenger emergency cleared.");
  }
  // @TODO, add a cleanup step, that removes old caution zones and old suggested trajectories, old safety corridors
}

behavior::Behavior DecisionMaker::choose_and_plan_driving_behavior()
{
  double time_now = now().seconds();

  bool has_localization = conditions::has_localization(latest_vehicle_state_dynamic, time_now);
  bool has_mission = conditions::has_mission(latest_vehicle_state_dynamic, latest_route);
  bool needs_remote_operator_assitance = conditions::needs_remote_operator_assitance( latest_vehicle_state_dynamic, caution_zones ); 
  bool needs_to_avoid_safety_corridor = conditions::needs_to_avoid_safety_corridor(latest_vehicle_state_dynamic, latest_safety_corridor);
  bool can_drive_managed = conditions::can_drive_managed(latest_vehicle_state_dynamic, time_now, latest_managed_zone, latest_managed_trajectory);
  bool odd_conditions_satisfied = conditions::odd_conditions_satisfied(latest_odd, time_now);



  // if (passenger_emergency_stop)
  // {
  //     return behavior::emergency(planner, latest_vehicle_state_dynamic);
  // }

  if (
    has_localization &&
    needs_to_avoid_safety_corridor
  )
  {
    return behavior::avoiding_safety_corridor(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                traffic_participants,
                                latest_safety_corridor.value()
    );
  }

  if (
      has_localization &&
      has_mission &&
      needs_remote_operator_assitance
    )
  {
    return behavior::remote_operations(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                latest_route.value(),
                                traffic_participants,
                                remote_operator_drive_approval,
                                suggested_remote_operator_trajectory
    );
  }

  if (
    has_localization &&
    has_mission &&
    odd_conditions_satisfied && 
    can_drive_managed

  )
  {
    return behavior::driving_mission_following_managed(
                  planner,
                  latest_vehicle_state_dynamic.value(),
                  latest_managed_trajectory.value(),
                  latest_managed_zone.value()
    );
  }

  if (
      has_localization &&
      has_mission &&
      odd_conditions_satisfied &&
      resume_ride_requested
  )
  {
      resume_ride_requested = false;

      return behavior::resume_ride(
        planner,
        latest_vehicle_state_dynamic.value(),
        latest_route.value(),
        traffic_participants,
        comfort_settings,
        traffic_signals,
        latest_weather
    );
  }


  if (
      has_localization &&
      has_mission &&
      odd_conditions_satisfied
  )
  {
    return behavior::driving_mission(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                latest_route.value(),
                                traffic_participants,
                                comfort_settings,
                                traffic_signals,
                                latest_weather
                              );
  }

  if ( 
      has_localization &&
      odd_conditions_satisfied
  )
  {
    return behavior::waiting_for_mission(
                                planner,
                                latest_vehicle_state_dynamic.value(),
                                traffic_participants
                              );
  }

  if (
    has_localization &&
    has_mission
  )
  {
    return behavior::minimum_risk(
                                  planner, 
                                  latest_vehicle_state_dynamic.value(), 
                                  latest_route.value(), 
                                  traffic_participants,
                                  latest_odd
                              );
  }

  return behavior::emergency(planner, latest_vehicle_state_dynamic);
}

adore_ros2_msgs::msg::TrafficParticipant DecisionMaker::make_default_participant()
{
  dynamics::TrafficParticipant participant;
  if( latest_vehicle_state_dynamic.has_value() )
    participant.state = latest_vehicle_state_dynamic.value();

  if( latest_route.has_value() )
  {
    participant.goal_point = latest_route->destination;
    participant.route      = latest_route.value();
  }

  participant.id                  = v2x_id;
  participant.v2x_id              = v2x_id;
  participant.classification      = dynamics::CAR;
  participant.physical_parameters = planner.get_physical_vehicle_parameters();

  return dynamics::conversions::to_ros_msg( participant );
}

void DecisionMaker::handle_passenger_request(
    const adore_ros2_msgs::msg::PassengerRequest& msg)
{
    using PR = adore_ros2_msgs::msg::PassengerRequest;

    const float increase_factor = (!msg.numerical_detail.empty())
                                  ? static_cast<float>(msg.numerical_detail[0])
                                  : 1.3f;

    const float decrease_factor = (!msg.numerical_detail.empty())
                                      ? static_cast<float>(msg.numerical_detail[0])
                                      : 0.7f;

    auto& cs = comfort_settings;
    bool changed = false;

    switch (msg.type)
    {
        case PR::INCREASE_VELOCITY:
            cs.max_speed *= increase_factor;
            changed = true;
            break;

        case PR::DECREASE_VELOCITY:
            cs.max_speed *= decrease_factor;
            changed = true;
            break;

        case PR::DRIVE_MORE_SPORTILY:
            cs.max_acceleration         *= increase_factor;
            cs.min_acceleration         *= increase_factor;
            cs.max_lateral_acceleration *= increase_factor;
            changed = true;
            break;

        case PR::DRIVE_MORE_COMFORTABLY:
            cs.max_acceleration         *= decrease_factor;
            cs.min_acceleration         *= decrease_factor;
            cs.max_lateral_acceleration *= decrease_factor;
            changed = true;
            break;

        case PR::KEEP_MORE_DISTANCE:
            cs.time_headway     *= increase_factor;
            cs.distance_headway *= increase_factor;
            changed = true;
            break;

        case PR::EXECUTE_EMERGENCY_STOP:
            passenger_emergency_stop = true;
            RCLCPP_WARN(get_logger(), "Passenger emergency stop requested!");
            changed = true;
            break;

        default:
            return;
    }

    if (changed)
    {
        cs.clamp(planner.get_physical_vehicle_parameters()); // enforce physical limits
        //planner.set_comfort_settings(*comfort_settings);     // push into planner
        RCLCPP_INFO(get_logger(),
            "Comfort settings updated: max_speed=%.1f time_headway=%.1f",
            cs.max_speed, cs.time_headway);
    }
}

void DecisionMaker::handle_mission_command(
    const adore_ros2_msgs::msg::MissionCommand& msg )
{
    using MC = adore_ros2_msgs::msg::MissionCommand;

    if( !msg.enable )
    {
        return;
    }

    switch( msg.command )
    {
        case MC::RESUME_RIDE:
            resume_ride_requested = true;

            remote_operator_drive_approval = false;
            suggested_remote_operator_trajectory.reset();

            RCLCPP_INFO( get_logger(), "Resume ride requested." );
            break;

        default:
            break;
    }
}



} // namespace adore

/* Register as component --------------------------------------------- */
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::DecisionMaker )
