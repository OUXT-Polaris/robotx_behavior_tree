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

#ifndef ROBOTX_BEHAVIOR_TREE__ACTION_NODE_HPP_
#define ROBOTX_BEHAVIOR_TREE__ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <quaternion_operation/quaternion_operation.h>

#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hermite_path_msgs/msg/planner_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robotx_behavior_msgs/msg/task_objects_array_stamped.hpp>
#include <stdexcept>
#include <string>

namespace geometry_msgs
{
namespace msg
{
struct Point2D
{
  double x;
  double y;
};

struct Vector2D
{
  double x;
  double y;
  Vector2D(double x, double y)
  {
    x = x;
    y = y;
  }
  template <typename T1, typename T2>
  Vector2D(const T1 & p1, const T2 & p2)
  {
    x = p2.x - p1.x;
    y = p2.y - p1.y;
  }
};

struct Pose2D
{
  Point2D position;
  Quaternion orientation;
};
}  // namespace msg
}  // namespace geometry_msgs

namespace robotx_behavior_tree
{
class ActionNode : public BT::StatefulActionNode
{
public:
  ActionNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
    setRegistrationID(name);
  }

protected:
  void onHalted() override {}
  std::string name;
};

class ActionROS2Node : public BT::StatefulActionNode, public rclcpp::Node
{
public:
  ActionROS2Node(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config), rclcpp::Node(name, rclcpp::NodeOptions())
  {
    setRegistrationID(name);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<robotx_behavior_msgs::msg::TaskObjectsArrayStamped::SharedPtr>("task_objects"),
      BT::InputPort<hermite_path_msgs::msg::PlannerStatus::SharedPtr>("planner_status"),
      BT::InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("current_pose")};
  }
  static BT::PortsList appendPorts(const BT::PortsList & ports1, const BT::PortsList & ports2)
  {
    BT::PortsList ports = {};
    for (const auto & port : ports1) {
      ports.emplace(port.first, port.second);
    }
    for (const auto & port : ports2) {
      ports.emplace(port.first, port.second);
    }
    return ports;
  }

protected:
  void onHalted() override {}
  std::string name;
  const robotx_behavior_msgs::msg::TaskObjectsArrayStamped::SharedPtr getTaskObjects() const
  {
    const auto ret =
      this->getInput<robotx_behavior_msgs::msg::TaskObjectsArrayStamped::SharedPtr>("task_objects");
    if (ret) {
      ret.value();
    } else {
      throw std::runtime_error("Task objects were not subscribed.");
    }
  }

#define DEFINE_GET_INPUT(NAME, TYPE, BLACKBOARD_KEY) \
  std::optional<TYPE> get##NAME() const              \
  {                                                  \
    const auto ret = getInput<TYPE>(BLACKBOARD_KEY); \
    if (ret) {                                       \
      return ret.value();                            \
    }                                                \
    return std::nullopt;                             \
  }

  DEFINE_GET_INPUT(
    PlannerStatus, hermite_path_msgs::msg::PlannerStatus::SharedPtr, "planner_status");
  DEFINE_GET_INPUT(CurrentPose, geometry_msgs::msg::PoseStamped::SharedPtr, "current_pose");
#undef DEFINE_GET_INPUT

  template <typename T1, typename T2>
  double getDistance(const T1 & p1, const T2 & p2) const
  {
    return std::hypot(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
  }

  template <typename T1, typename T2>
  double get2DDistance(const T1 & p1, const T2 & p2) const
  {
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
  }

  std::vector<robotx_behavior_msgs::msg::TaskObject> filter(
    const std::vector<robotx_behavior_msgs::msg::TaskObject> & task_objects,
    uint8_t object_kind) const
  {
    std::vector<robotx_behavior_msgs::msg::TaskObject> filtered;
    for (const auto & task_object : task_objects) {
      if (task_object.object_kind == object_kind) {
        filtered.emplace_back(task_object);
      }
    }
    return filtered;
  }

  void sortBy2DDistance(
    std::vector<robotx_behavior_msgs::msg::TaskObject> & task_objects,
    const geometry_msgs::msg::Point & origin) const
  {
    std::sort(
      task_objects.begin(), task_objects.end(), [this, origin](auto const & lhs, auto const & rhs) {
        return get2DDistance(getPoint2D(lhs), origin) < get2DDistance(getPoint2D(rhs), origin);
      });
  }

  void sortByDistance(
    std::vector<robotx_behavior_msgs::msg::TaskObject> & task_objects,
    const geometry_msgs::msg::Point & origin) const
  {
    std::sort(
      task_objects.begin(), task_objects.end(), [this, origin](auto const & lhs, auto const & rhs) {
        const auto p1 = getPoint(lhs);
        const auto p2 = getPoint(rhs);
        if (!p1 || !p2) {
          throw std::runtime_error("including 2d task object!!");
        }
        return getDistance(p1.value(), origin) < getDistance(p2.value(), origin);
      });
  }

  std::optional<geometry_msgs::msg::Point> getPoint(
    const robotx_behavior_msgs::msg::TaskObject & task_object) const
  {
    if (task_object.z.empty()) {
      return std::nullopt;
    }
    geometry_msgs::msg::Point p;
    p.x = task_object.x;
    p.y = task_object.y;
    p.z = task_object.z[0];
    return p;
  }

  geometry_msgs::msg::Point2D getPoint2D(
    const robotx_behavior_msgs::msg::TaskObject & task_object) const
  {
    geometry_msgs::msg::Point2D p;
    p.x = task_object.x;
    p.y = task_object.y;
    return p;
  }

  std::optional<geometry_msgs::msg::Pose2D> getPose2D(
    const robotx_behavior_msgs::msg::TaskObject & task_object) const
  {
    if (task_object.theta.empty()) {
      return std::nullopt;
    }
    geometry_msgs::msg::Pose2D p;
    p.position = getPoint2D(task_object);
    geometry_msgs::msg::Vector3 rpy;
    rpy.z = task_object.theta[0];
    p.orientation = quaternion_operation::convertEulerAngleToQuaternion(rpy);
    return p;
  }

  std::optional<geometry_msgs::msg::Pose> getPose(
    const robotx_behavior_msgs::msg::TaskObject & task_object) const
  {
    const auto point = getPoint(task_object);
    if (task_object.theta.empty() || !point) {
      return std::nullopt;
    }
    geometry_msgs::msg::Pose p;
    p.position = point.value();
    geometry_msgs::msg::Vector3 rpy;
    rpy.z = task_object.theta[0];
    p.orientation = quaternion_operation::convertEulerAngleToQuaternion(rpy);
    return p;
  }

  geometry_msgs::msg::Pose between(
    const robotx_behavior_msgs::msg::TaskObject & obj1,
    const robotx_behavior_msgs::msg::TaskObject & obj2,
    const geometry_msgs::msg::Pose & robot_pose) const
  {
    geometry_msgs::msg::Pose p;
    const auto p1 = getPoint2D(obj1);
    const auto p2 = getPoint2D(obj2);
    p.position.x = (p1.x + p2.x) * 0.5;
    p.position.y = (p1.y + p2.y) * 0.5;
    p.position.z = 0.0;
    const auto v = geometry_msgs::msg::Vector2D(p1, p2);
    const auto robot_rpy =
      quaternion_operation::convertQuaternionToEulerAngle(robot_pose.orientation);
    const auto v_robot = geometry_msgs::msg::Vector2D(std::cos(robot_rpy.z), std::sin(robot_rpy.z));
    geometry_msgs::msg::Vector3 goal_rpy;
    if ((v.y * v_robot.x - v.x * v_robot.y) >= (-v.y * v_robot.x + v.x * v_robot.y)) {
      goal_rpy.z = std::atan2(-v.x, v.y);
    } else {
      goal_rpy.z = std::atan2(v.x, -v.y);
    }
    p.orientation = quaternion_operation::convertEulerAngleToQuaternion(goal_rpy);
    return p;
  }
};
}  // namespace robotx_behavior_tree

#endif  // ROBOTX_BEHAVIOR_TREE__ACTION_NODE_HPP_
