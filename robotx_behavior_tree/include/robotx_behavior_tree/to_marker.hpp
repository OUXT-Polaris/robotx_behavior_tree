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

#include <robotx_behavior_msgs/msg/task_objects_array.hpp>
#include <robotx_behavior_msgs/msg/task_objects_array_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace robotx_behavior_tree
{
const visualization_msgs::msg::MarkerArray toMarker(
  const robotx_behavior_msgs::msg::TaskObject & objects, const std_msgs::msg::Header & header,
  double max_alpha = 0.9);
const visualization_msgs::msg::MarkerArray toMarker(
  const std::vector<robotx_behavior_msgs::msg::TaskObject> & objects,
  const std_msgs::msg::Header & header, double max_alpha = 0.9);
const visualization_msgs::msg::MarkerArray toMarker(
  const robotx_behavior_msgs::msg::TaskObjectsArray & objects, const std_msgs::msg::Header & header,
  double max_alpha = 0.9);
const visualization_msgs::msg::MarkerArray toMarker(
  const robotx_behavior_msgs::msg::TaskObjectsArrayStamped & objects, double max_alpha = 0.9);
const visualization_msgs::msg::MarkerArray toMarker(
  const robotx_behavior_msgs::msg::TaskObjectsArrayStamped::SharedPtr objects,
  double max_alpha = 0.9);
}  // namespace robotx_behavior_tree
