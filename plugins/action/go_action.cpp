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

#include <string>
#include <memory>
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotx_waypoint_msgs/action/way_point.hpp"
#include "behavior_tree_action_builder/action_node_with_action.hpp"

namespace robotx_behavior_tree
{
using WayPoint = robotx_waypoint_msgs::action::WayPoint;
class GoAction : public behavior_tree_action_builder::ActionNodeWithAction<WayPoint>
{
public:
  GoAction(
    const std::string & name,
    const BT::NodeConfiguration & config)
  :  behavior_tree_action_builder::ActionNodeWithAction<WayPoint>(name, config)
  {

  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("test_input")};
  }

protected:
  BT::NodeStatus tick() override
  {
    if(result_){
      RCLCPP_INFO(get_logger(), "SUCCESS");
      return BT::NodeStatus::SUCCESS;
    } else{
      RCLCPP_INFO(get_logger(), "RUNNING");
//      return BT::NodeStatus::SUCCESS;
      return BT::NodeStatus::RUNNING;
    }
  }
public:
//  virtual void halt() override {}

public:
//  virtual BT::NodeStatus executeTick() override final {
//    return result_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
//  }
protected:
  virtual void callbackGoalResponce(
    std::shared_future<GoalHandle::SharedPtr> future) override
  {
    if (auto goal_handle = future.get()) {
      RCLCPP_INFO(get_logger(), "Goal was accepted");
    } else {
      RCLCPP_INFO(get_logger(), "Goal was rejected");
    }
  }
  virtual void callbackFeedback(
    GoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<WayPoint::Feedback> feedback) override
  {
    if (feedback->is_reached.data) {
      RCLCPP_INFO(
        get_logger(), "Feedback : not reached at %f , %f", feedback->pose_stamped.pose.position.x,
        feedback->pose_stamped.pose.position.y);
    } else {
      RCLCPP_INFO(
        get_logger(), "Feedback : reached at %f , %f", feedback->pose_stamped.pose.position.x,
        feedback->pose_stamped.pose.position.y);
    }
  }
  virtual void callbackResult(
    const rclcpp_action::ClientGoalHandle<WayPoint>::WrappedResult & result)
  override
  {
    result_ = result.result->is_reached.data;
    std::cout << "result" << std::endl;
  }

  bool result_ = false;
//  rclcpp::Node::SharedPtr node_;
};
}  // namespace robotx_behavior_tree

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT
REGISTER_NODES(robotx_behavior_tree,GoAction, go)
