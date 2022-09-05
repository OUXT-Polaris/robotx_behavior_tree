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

#ifndef ROBOTX_BT_PLANNER__DESCRIPTOR__DESCRIPTOR_HPP_
#define ROBOTX_BT_PLANNER__DESCRIPTOR__DESCRIPTOR_HPP_

#include <fstream>
#include <hermite_path_msgs/msg/planner_status.hpp>
#include <memory>
#include <robotx_behavior_msgs/msg/task_objects_array_stamped.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "behaviortree_cpp_v3/blackboard.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "robotx_behavior_msgs/srv/evaluation.hpp"
#include "robotx_bt_planner/descriptor/data_structures.hpp"
#include "robotx_bt_planner/descriptor/operators.hpp"

#define SOL_ALL_SAFETIES_ON 1
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "robotx_bt_planner/descriptor/functions.hpp"
#include "sol/sol.hpp"

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

namespace robotx_bt_planner
{
struct EvaluationBlockBase
{
  std::string name;
  std::string evaluation;
  virtual void evaluate(sol::state & state, BT::Blackboard::Ptr board) = 0;
  virtual std::string getDebugString() = 0;
};
template <class T>
struct EvaluationBlock : EvaluationBlockBase
{
  T result;
  using value_type = T;

  void evaluate(sol::state & state, BT::Blackboard::Ptr board) override
  {
    std::string eval = "return " + evaluation;
    auto result = state.script(eval);
    if (result.valid()) {
      this->result = static_cast<T>(result);
      board->set(name, this->result);
      RCLCPP_INFO(rclcpp::get_logger("robotx_bt_planner"), getDebugString().c_str());
    } else {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger("robotx_bt_planner"), "Evaluation " << evaluation << " has failed!");
    }
  }
  std::string getDebugString() override
  {
    std::stringstream ss;
    ss << "Evaluation " << evaluation << " has succeeded! : " << result;
    return ss.str();
  }
};

class BTPlannerComponent : public rclcpp::Node
{
public:
  ROBOTX_BT_PLANNER_PUBLIC
  explicit BTPlannerComponent(const rclcpp::NodeOptions & options);
  ~BTPlannerComponent() {}

  void timerCallback()
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "tick" << tree_.nodes.size());
    evaluationCallback();
    tree_.rootNode()->executeTick();
  }

  void loadConfig(std::string file_path)
  {
    node_ = YAML::LoadFile(file_path);
    node_ >> format_;

    RCLCPP_INFO(get_logger(), "open sol library");
    lua_.open_libraries(sol::lib::base);
    addPresetFunctions(lua_);

    if (node_["behavior"]["blackboard"]) {
      RCLCPP_INFO(get_logger(), "loading blackboard config");
      for (auto board : node_["behavior"]["blackboard"]) {
        if (!board["eval"]) {
          continue;
        }

        // TODO(HansRobo) : make custom type evaluation
        auto evaluation = std::make_shared<EvaluationBlock<double>>();
        evaluation->name = board["input"].as<std::string>();
        evaluation->evaluation = board["eval"].as<std::string>();
        this->evaluation_blocks_.emplace_back(evaluation);
        RCLCPP_INFO_STREAM(get_logger(), "loading evaluation : " << evaluation->name);
      }
    }
  }

  void loadPlugins()
  {
    if (node_["plugins"]) {
      for (auto plugin : node_["plugins"]) {
        auto package_name = plugin["package"].as<std::string>();
        for (auto name : plugin["name"]) {
          std::string plugin_name = name.as<std::string>();
          RCLCPP_INFO_STREAM(
            rclcpp::get_logger("robotx_bt_planner"), "LOAD PLUGIN : " << plugin_name);
          std::string plugin_filename = ament_index_cpp::get_package_share_directory(package_name) +
                                        "/../../lib/lib" + plugin_name + ".so";
          factory_.registerFromPlugin(plugin_filename);
        }
      }
    }
    RCLCPP_INFO(get_logger(), "REGISTERED PLUGINS : ");
    RCLCPP_INFO(get_logger(), "=================================");
    for (auto builder : factory_.builders()) {
      RCLCPP_INFO_STREAM(get_logger(), "" << builder.first);
    }
    RCLCPP_INFO(get_logger(), "=================================");
  }

  bool loadTree()
  {
    if (node_["behavior"]["description"]) {
      auto description = node_["behavior"]["description"];
      std::string package = description["package"].as<std::string>();
      std::string path = description["path"].as<std::string>();
      std::string file_path = ament_index_cpp::get_package_share_directory(package) + "/" + path;
      std::ifstream xml_file(file_path);
      if (!xml_file.good()) {
        RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", file_path.c_str());
        return false;
      }

      auto xml_string =
        std::string(std::istreambuf_iterator<char>(xml_file), std::istreambuf_iterator<char>());
      tree_ = factory_.createTreeFromText(xml_string, blackboard_);
      RCLCPP_INFO(get_logger(), "behavior tree loaded: %s", file_path.c_str());
      return true;
    }
    return false;
  }

  void evaluationCallback()
  {
    //    for(auto key : blackboard_->getKeys()){
    //      std::string key_str(key);
    //      auto a = blackboard_->getAny(key_str)->
    //      auto value = blackboard_->getAny(key_str);
    //      value->type().
    //    }
    for (auto & block : evaluation_blocks_) {
      block->evaluate(lua_, blackboard_);
    }
  }

private:
  std::string config_file_;
  std::string config_package_;
  std::string task_object_topic_;
  std::string marker_topic_;
  float update_rate_;
  bool publish_marker_;
  YAML::Node node_;
  Format format_;
  sol::state lua_;
  std::vector<std::shared_ptr<EvaluationBlockBase>> evaluation_blocks_;
  rclcpp::TimerBase::SharedPtr timer_;
  BT::Blackboard::Ptr blackboard_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<BT::PublisherZMQ> publisher_zmq_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Service<robotx_behavior_msgs::srv::Evaluation>::SharedPtr evaluation_server_;
  rclcpp::Subscription<robotx_behavior_msgs::msg::TaskObjectsArrayStamped>::SharedPtr
    task_objects_array_sub_;
  void taskObjectsArrayCallback(
    const robotx_behavior_msgs::msg::TaskObjectsArrayStamped::SharedPtr data);
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<hermite_path_msgs::msg::PlannerStatus>::SharedPtr planner_status_sub_;

  /*
#define DEFINE_ROS_SUBSCRIPTION(NAME_SNAKE, NAME_CAMEL, TYPE, TOPIC, BLACKBOARD_KEY)           \
  rclcpp::Subscription<TYPE>::SharedPtr NAME_SNAKE_sub_;                                       \
  void register##NAME_CAMELSubscription()                                                      \
  {                                                                                            \
    NAME_SNAKE_sub_ =                                                                          \
      this->create_subscription<TYPE::SharedPtr>(TOPIC, 1, [this](const TYPE::SharedPtr msg) { \
        blackboard_->set<TYPE::SharedPtr>(BLACKBOARD_KEY, msg);                                \
      });                                                                                      \
  }

  DEFINE_ROS_SUBSCRIPTION(
    planner_status, PlannerStatus, hermite_path_msgs::msg::PlannerStatus,
    "/local_waypoint_server/planner_status", "planner_status")
#undef DEFINE_ROS_SUBSCRIPTION
*/
};  // namespace robotx_bt_planner
}  // namespace robotx_bt_planner

#endif  // ROBOTX_BT_PLANNER__DESCRIPTOR__DESCRIPTOR_HPP_
