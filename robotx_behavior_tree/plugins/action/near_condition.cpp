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

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robotx_behavior_tree
{
class NearCondition : public BT::ConditionNode, public rclcpp::Node
{
public:
  NearCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config), rclcpp::Node(name, rclcpp::NodeOptions())
  {
    auto topic_value = this->getInput<std::string>("topic");
    if (topic_value) {
      topic_ = topic_value.value();
      evaluating_point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        topic_, 1, std::bind(&NearCondition::pointCallback, this, std::placeholders::_1));
    }

    auto target_value = this->getInput<geometry_msgs::msg::Point>("target");
    if (target_value) {
      target_ = target_value.value();
    }

    auto threshold_value = this->getInput<double>("threshold");
    if (threshold_value) {
      threshold_ = threshold_value.value();
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Point>("target"), BT::InputPort<double>("threshold"),
      BT::InputPort<std::string>("topic")};
  }

protected:
  BT::NodeStatus tick() override
  {
    double dx, dy, dz;
    dx = evaluating_point_.x - target_.x;
    dy = evaluating_point_.y - target_.y;
    dz = evaluating_point_.z - target_.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance < threshold_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void pointCallback(const geometry_msgs::msg::Point & msg) { evaluating_point_ = msg; }
  std::string topic_;
  double threshold_;
  geometry_msgs::msg::Point target_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr evaluating_point_sub_;
  geometry_msgs::msg::Point evaluating_point_;
};

}  // namespace robotx_behavior_tree
