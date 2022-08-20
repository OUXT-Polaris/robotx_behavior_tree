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

#include <robotx_behavior_tree/to_marker.hpp>

const visualization_msgs::msg::MarkerArray toMarker(
  const robotx_behavior_msgs::msg::TaskObject & object, const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray msg;
  visualization_msgs::msg::Marker model;
  model.header = header;
  model.ns = std::to_string(object.unique_id);
  model.id = 0;
  model.action = visualization_msgs::msg::Marker::ADD;
  switch (object.object_kind) {
    case robotx_behavior_msgs::msg::TaskObject::BUOY_RED:
      model.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      model.mesh_resource = "package://robotx_behavior_msgs/models/mb_marker_buoy_red/meshes/mb_marker_buoy.dae";
      model.scale.x = 1.0;
      model.scale.y = 1.0;
      model.scale.z = 1.0;
      break;
    case robotx_behavior_msgs::msg::TaskObject::BUOY_GREEN:
      model.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      model.mesh_resource = "package://robotx_behavior_msgs/models/mb_marker_buoy_green/meshes/mb_marker_buoy.dae";
      model.scale.x = 1.0;
      model.scale.y = 1.0;
      model.scale.z = 1.0;
      break;
    case robotx_behavior_msgs::msg::TaskObject::BUOY_WHITE:
      break;
    case robotx_behavior_msgs::msg::TaskObject::BUOY_BLACK:
      break;
  }
  return msg;
}

const visualization_msgs::msg::MarkerArray toMarker(
  const robotx_behavior_msgs::msg::TaskObjectsArray & objects, const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray msg;
  return msg;
}

const visualization_msgs::msg::MarkerArray toMarker(
  const robotx_behavior_msgs::msg::TaskObjectsArrayStamped & objects)
{
  visualization_msgs::msg::MarkerArray msg;
  return msg;
}
