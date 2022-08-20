// Copyright (c) 2022, OUXT-Polaris
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

#include <bits/stdc++.h>
#include <quaternion_operation/quaternion_operation.h>

#include <color_names/color_names.hpp>
#include <robotx_behavior_tree/to_marker.hpp>

namespace robotx_behavior_tree
{
const visualization_msgs::msg::MarkerArray toMarker(
  const robotx_behavior_msgs::msg::TaskObject & object, const std_msgs::msg::Header & header,
  double max_alpha)
{
  visualization_msgs::msg::MarkerArray msg;
  visualization_msgs::msg::Marker model;
  model.header = header;
  model.ns = std::to_string(object.unique_id);
  model.id = 0;
  model.action = visualization_msgs::msg::Marker::ADD;
  geometry_msgs::msg::Vector3 model_rpy;
  switch (object.object_kind) {
    case robotx_behavior_msgs::msg::TaskObject::BUOY_RED:
      model.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      model.mesh_use_embedded_materials = false;
      model.mesh_resource =
        "package://robotx_behavior_tree/models/surmark950410/mesh/surmark950410.dae";
      model.scale.x = 1.0;
      model.scale.y = 1.0;
      model.scale.z = 1.0;
      model.pose.position.x = object.x;
      model.pose.position.y = object.y;
      model.color = color_names::makeColorMsg("red", max_alpha * object.reliability);
      break;
    case robotx_behavior_msgs::msg::TaskObject::BUOY_GREEN:
      model.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      model.mesh_use_embedded_materials = false;
      model.mesh_resource =
        "package://robotx_behavior_tree/models/surmark950400/mesh/surmark950400.dae";
      model.scale.x = 1.0;
      model.scale.y = 1.0;
      model.scale.z = 1.0;
      model.pose.position.x = object.x;
      model.pose.position.y = object.y;
      model.color = color_names::makeColorMsg("green", max_alpha * object.reliability);
      break;
    case robotx_behavior_msgs::msg::TaskObject::BUOY_WHITE:
      model.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      model.mesh_use_embedded_materials = false;
      model.mesh_resource =
        "package://robotx_behavior_tree/models/surmark46104/mesh/surmark46104.dae";
      model.scale.x = 0.6;
      model.scale.y = 0.6;
      model.scale.z = 1.0;
      model.pose.position.x = object.x;
      model.pose.position.y = object.y;
      model.color = color_names::makeColorMsg("white", max_alpha * object.reliability);
      break;
    case robotx_behavior_msgs::msg::TaskObject::BUOY_BLACK:
      model.type = visualization_msgs::msg::Marker::SPHERE;
      model.mesh_use_embedded_materials = false;
      model.scale.x = 0.5;
      model.scale.y = 0.5;
      model.scale.z = 0.5;
      model.pose.position.x = object.x;
      model.pose.position.y = object.y;
      model.color = color_names::makeColorMsg("black", max_alpha * object.reliability);
      break;
  }
  msg.markers.emplace_back(model);
  return msg;
}

const visualization_msgs::msg::MarkerArray toMarker(
  const std::vector<robotx_behavior_msgs::msg::TaskObject> & objects,
  const std_msgs::msg::Header & header, double max_alpha)
{
  visualization_msgs::msg::MarkerArray msg;
  for (const auto & object : objects) {
    const auto marker_array = toMarker(object, header, max_alpha);
    std::copy(
      marker_array.markers.begin(), marker_array.markers.end(), std::back_inserter(msg.markers));
  }
  return msg;
}

const visualization_msgs::msg::MarkerArray toMarker(
  const robotx_behavior_msgs::msg::TaskObjectsArray & objects, const std_msgs::msg::Header & header,
  double max_alpha)
{
  return toMarker(objects.task_objects, header, max_alpha);
}

const visualization_msgs::msg::MarkerArray toMarker(
  const robotx_behavior_msgs::msg::TaskObjectsArrayStamped & objects, double max_alpha)
{
  return toMarker(objects.task_objects, objects.header, max_alpha);
}

const visualization_msgs::msg::MarkerArray toMarker(
  const robotx_behavior_msgs::msg::TaskObjectsArrayStamped::SharedPtr objects, double max_alpha)
{
  return toMarker(objects->task_objects, objects->header, max_alpha);
}
}  // namespace robotx_behavior_tree
