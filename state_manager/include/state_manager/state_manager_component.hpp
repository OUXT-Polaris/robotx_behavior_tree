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

#ifndef STATE_MANAGER_COMPONENT__STATE_MANAGER_COMPONENT_HPP_
#define STATE_MANAGER_COMPONENT__STATE_MANAGER_COMPONENT_HPP_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define STATE_MANAGER_COMPONENT_EXPORT __attribute__((dllexport))
#define STATE_MANAGER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define STATE_MANAGER_COMPONENT_EXPORT __declspec(dllexport)
#define STATE_MANAGER_COMPONENT_IMPORT __declspec(dllimport)
#endif

#ifdef STATE_MANAGER_COMPONENT_DLL
#define STATE_MANAGER_COMPONENT_PUBLIC STATE_MANAGER_COMPONENT_EXPORT
#else
#define STATE_MANAGER_COMPONENT_PUBLIC STATE_MANAGER_COMPONENT_IMPORT
#endif

#define STATE_MANAGER_COMPONENT_PUBLIC_TYPE STATE_MANAGER_COMPONENT_PUBLIC

#define STATE_MANAGER_COMPONENT_LOCAL

#else

#define STATE_MANAGER_COMPONENT_EXPORT __attribute__((visibility("default")))
#define STATE_MANAGER_COMPONENT_IMPORT

#if __GNUC__ >= 4
#define STATE_MANAGER_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define STATE_MANAGER_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define STATE_MANAGER_COMPONENT_PUBLIC
#define STATE_MANAGER_COMPONENT_LOCAL
#endif

#define STATE_MANAGER_COMPONENT_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#include <rclcpp/rclcpp.hpp>

namespace state_manager
{
class StateManagerComponent : public rclcpp::Node
{
public:
  explicit StateManagerComponent(const rclcpp::NodeOptions & options);
};
}  // namespace state_manager

#endif  // STATE_MANAGER_COMPONENT__STATE_MANAGER_COMPONENT_HPP_
