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

#ifndef ROBOTX_BT_PLANNER__DESCRIPTOR__OPERATORS_HPP_
#define ROBOTX_BT_PLANNER__DESCRIPTOR__OPERATORS_HPP_

#include <string>

#include "robotx_bt_planner/descriptor/data_structures.hpp"
#include "yaml-cpp/yaml.h"

namespace robotx_bt_planner
{
void operator>>(const YAML::Node & node, Position & position)
{
  try {
    position.x = node["x"].as<double>();
    position.y = node["y"].as<double>();
    position.z = node["z"].as<double>();
  } catch (...) {
    position.x = 0.0;
    position.y = 0.0;
    position.z = 0.0;
    RCLCPP_ERROR(rclcpp::get_logger("robotx_bt_planner"), "parse error : Position");
  }
}
void operator>>(const YAML::Node & node, Quaternion & quaternion)
{
  try {
    quaternion.x = node["x"].as<double>();
    quaternion.y = node["y"].as<double>();
    quaternion.z = node["z"].as<double>();
    quaternion.w = node["w"].as<double>();
  } catch (...) {
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = 0.0;
    quaternion.w = 0.0;
    RCLCPP_ERROR(rclcpp::get_logger("robotx_bt_planner"), "parse error : Quaternion");
  }
}

void operator>>(const YAML::Node & node, Pose & pose)
{
  try {
    node["position"] >> pose.position;
    node["orientation"] >> pose.orientation;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("robotx_bt_planner"), "parse error : Pose");
  }
}

void operator>>(const YAML::Node & node, Pose2D & pose2d)
{
  try {
    pose2d.x = node["x"].as<double>();
    pose2d.y = node["y"].as<double>();
    pose2d.theta = node["theta"].as<double>();
  } catch (...) {
    pose2d.x = 0.0;
    pose2d.y = 0.0;
    pose2d.theta = 0.0;
    RCLCPP_ERROR(rclcpp::get_logger("robotx_bt_planner"), "parse error : Pose2D");
  }
}

void operator>>(const YAML::Node & node, Object & object)
{
  try {
    object.uuid = node["uuid"].as<int>();
    const YAML::Node & attributes_node = node["attributes"];
    for (auto attribute_node : attributes_node) {
      std::string attribute = attribute_node.as<std::string>();
      object.attributes.push_back(attribute);
    }
    node["pose"] >> object.pose;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("robotx_bt_planner"), "parse error : Object");
  }
}

void operator>>(const YAML::Node & node, BlackBoard & blackboard)
{
  try {
    blackboard.input = node["input"].as<std::string>();
    blackboard.eval = node["eval"].as<std::string>();
  } catch (...) {
    blackboard.input = "";
    blackboard.eval = "";
    RCLCPP_ERROR(rclcpp::get_logger("robotx_bt_planner"), "parse error : BlackBoard");
  }
}

void operator>>(const YAML::Node & node, Behavior & behavior)
{
  try {
    //    behavior.description = node["description"].as<std::string>();
    const YAML::Node & blackboards_node = node["blackboard"];
    for (auto blackboard_node : blackboards_node) {
      BlackBoard blackboard;
      blackboard_node >> blackboard;
      behavior.blackboard.push_back(blackboard);
    }
  } catch (...) {
    behavior.description = "";
    RCLCPP_ERROR(rclcpp::get_logger("robotx_bt_planner"), "parse error : Behavior");
  }
}

void operator>>(const YAML::Node & node, Format & format)
{
  try {
    node["behavior"] >> format.behavior;
    const YAML::Node & objects_node = node["objects"];
    for (auto object_node : objects_node) {
      Object object;
      object_node >> object;
      format.objects.push_back(object);
    }
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("robotx_bt_planner"), "parse error : Format");
  }
}
}  // namespace robotx_bt_planner

#endif  // ROBOTX_BT_PLANNER__DESCRIPTOR__OPERATORS_HPP_
