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

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <memory>
#include <mutex>
#include <robotx_behavior_tree/action_node.hpp>
#include <robotx_msgs/msg/autonomous_maritime_system_status.hpp>
#include <string>

namespace robotx_behavior_tree
{
class ConfigureControllerAction : public ActionROS2Node
{
public:
  ConfigureControllerAction(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROS2Node(name, config)
  {
    switch_controller_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/load_controller");
  }

  static BT::PortsList providedPorts()
  {
    return appendPorts(ActionROS2Node::providedPorts(), {BT::InputPort<uint8_t>("mode")});
  }

protected:
  BT::NodeStatus onStart() override
  {
    mtx_.lock();
    switch_succeed_ = false;
    response_received_ = false;
    mtx_.unlock();
    auto mode = this->getInput<uint8_t>("mode");
    if (!mode) {
      return BT::NodeStatus::FAILURE;
    }
    auto response_received_callback =
      [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
        mtx_.lock();
        switch_succeed_ = future.get()->ok;
        response_received_ = true;
        mtx_.unlock();
      };
    switch_controller_client_->async_send_request(
      createSwitchControllerRequest(mode.value()), response_received_callback);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    mtx_.lock();
    if (!response_received_) {
      mtx_.unlock();
      return BT::NodeStatus::RUNNING;
    }
    if (switch_succeed_) {
      mtx_.unlock();
      return BT::NodeStatus::SUCCESS;
    }
    mtx_.unlock();
    return BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request>
  createSwitchControllerRequest(uint8_t mode)
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
        request->stop_controllers.emplace_back("usv_twist_controller");
        request->start_controllers.emplace_back("usv_joy_controller");
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

  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_client_;
  bool switch_succeed_;
  bool response_received_;
  std::mutex mtx_;
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT
REGISTER_NODES(robotx_behavior_tree, ConfigureControllerAction)
