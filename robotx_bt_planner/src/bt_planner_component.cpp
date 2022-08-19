// Copyright (c) 2020, OUXT-Polaris
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

#include "robotx_bt_planner/bt_planner_component.hpp"

#include <fstream>
#include <memory>
#include <set>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace robotx_bt_planner
{
BTPlannerComponent::BTPlannerComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("robotx_bt_planner", options)
{
  declare_parameter<std::string>("config_package", "robotx_bt_planner");
  get_parameter("config_package", config_package_);
  declare_parameter<std::string>("config_file", "config/example.yaml");
  get_parameter("config_file", config_file_);
  declare_parameter<double>("update_rate", 10.0);
  declare_parameter<std::string>("task_object_topic", "/perception/task_objects");
  get_parameter("task_object_topic", task_object_topic_);
  get_parameter("update_rate", update_rate_);

  std::string config_path =
    ament_index_cpp::get_package_share_directory(config_package_) + "/" + config_file_;
  loadConfig(config_path);
  RCLCPP_INFO(get_logger(), "config file loaded!");

  using std::chrono_literals::operator""ms;

  blackboard_ = BT::Blackboard::create();

  auto client_options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "-r", std::string("__node:=") + get_name() + "_client_node", "--"});
  client_node_ = std::make_shared<rclcpp::Node>("_", client_options);
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);
  blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));

  loadPlugins();
  loadTree();

  publisher_zmq_ = std::make_unique<BT::PublisherZMQ>(tree_);

  using std::literals::chrono_literals::operator""s;
  auto interval = 1s / update_rate_;
  timer_ = create_wall_timer(500ms, std::bind(&BTPlannerComponent::timerCallback, this));
}

void BTPlannerComponent::taskObjectsArrayCallback(
  const robotx_behavior_msgs::msg::TaskObjectsArray::SharedPtr data)
{
  blackboard_->set<robotx_behavior_msgs::msg::TaskObjectsArray::SharedPtr>("task_objects", data);
}
}  // namespace robotx_bt_planner

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(robotx_bt_planner::BTPlannerComponent)
