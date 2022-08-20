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
  const robotx_behavior_msgs::msg::TaskObject & objects)
{
  switch (objects.object_kind) {
    case robotx_behavior_msgs::msg::TaskObject::BUOY_RED:
      break;
    case robotx_behavior_msgs::msg::TaskObject::BUOY_GREEN:
      break;
    case robotx_behavior_msgs::msg::TaskObject::BUOY_WHITE:
      break;
    case robotx_behavior_msgs::msg::TaskObject::BUOY_BLACK:
      break;
  }
}

const visualization_msgs::msg::MarkerArray toMarker(
  const robotx_behavior_msgs::msg::TaskObjectsArray & objects)
{
}
