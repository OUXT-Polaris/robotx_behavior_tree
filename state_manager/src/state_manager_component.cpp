// Copyright (c) 2021, OUXT-Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <state_manager/state_manager_component.hpp>

namespace state_manager
{
StateManagerComponent::StateManagerComponent(const rclcpp::NodeOptions & options)
: Node("state_manager", options)
{
  list_controller_client_ = create_client<controller_manager_msgs::srv::ListControllers>(
    "/controller_manager/list_controllers");
  switch_controller_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller");
}

std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request>
StateManagerComponent::createSwitchControllerRequest(uint8_t mode)
{
  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  request->start_asap = true;
  request->timeout = rclcpp::Duration(1, 0);
  switch (mode) {
    case robotx_msgs::msg::AutonomousMaritimeSystemStatus::REMOTE_OPERATED:
      request->stop_controllers.emplace_back("usv_twist_controller");
      request->start_controllers.emplace_back("usv_joy_controller");
      break;
    case robotx_msgs::msg::AutonomousMaritimeSystemStatus::AUTONOMOUS:
      request->stop_controllers.emplace_back("usv_joy_controller");
      request->start_controllers.emplace_back("usv_twist_controller");
      break;
    case robotx_msgs::msg::AutonomousMaritimeSystemStatus::KILLED:
      request->stop_controllers.emplace_back("usv_twist_controller");
      request->stop_controllers.emplace_back("usv_joy_controller");
      break;
    default:
      break;
  }
  return request;
}
}  // namespace state_manager
