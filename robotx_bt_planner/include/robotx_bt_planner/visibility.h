// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef ROBOTX_BT_PLANNER__VISIBILITY_H_
#define ROBOTX_BT_PLANNER__VISIBILITY_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define ROBOTX_BT_PLANNER_EXPORT __attribute__((dllexport))
#define ROBOTX_BT_PLANNER_IMPORT __attribute__((dllimport))
#else
#define ROBOTX_BT_PLANNER_EXPORT __declspec(dllexport)
#define ROBOTX_BT_PLANNER_IMPORT __declspec(dllimport)
#endif

#ifdef ROBOTX_BT_PLANNER_DLL
#define ROBOTX_BT_PLANNER_PUBLIC ROBOTX_BT_PLANNER_EXPORT
#else
#define ROBOTX_BT_PLANNER_PUBLIC ROBOTX_BT_PLANNER_IMPORT
#endif

#define ROBOTX_BT_PLANNER_PUBLIC_TYPE ROBOTX_BT_PLANNER_PUBLIC

#define ROBOTX_BT_PLANNER_LOCAL

#else

#define ROBOTX_BT_PLANNER_EXPORT __attribute__((visibility("default")))
#define ROBOTX_BT_PLANNER_IMPORT

#if __GNUC__ >= 4
#define ROBOTX_BT_PLANNER_PUBLIC __attribute__((visibility("default")))
#define ROBOTX_BT_PLANNER_LOCAL __attribute__((visibility("hidden")))
#else
#define ROBOTX_BT_PLANNER_PUBLIC
#define ROBOTX_BT_PLANNER_LOCAL
#endif

#define ROBOTX_BT_PLANNER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROBOTX_BT_PLANNER__VISIBILITY_H_
