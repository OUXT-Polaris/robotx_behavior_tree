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

#include "robotx_bt_planner/bt_planner_component.hpp"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <fstream>
#include <memory>
#include <pugixml.hpp>
#include <robotx_behavior_tree/action_node.hpp>
#include <robotx_behavior_tree/to_marker.hpp>
#include <set>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace robotx_bt_planner
{
BTPlannerComponent::BTPlannerComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("robotx_bt_planner", options)
{
  declare_parameter<std::string>("config_package", "robotx_bt_planner");
  get_parameter("config_package", config_package_);
  declare_parameter<std::string>("config_file", "config/example.yaml");
  get_parameter("config_file", config_file_);
  declare_parameter<double>("update_rate", 10.0);
  get_parameter("update_rate", update_rate_);
  declare_parameter<std::string>("task_object_topic", "/perception/task_objects");
  get_parameter("task_object_topic", task_object_topic_);
  declare_parameter<std::string>("task_object_marker_topic", "/perception/task_objects/marker");
  get_parameter("task_object_marker_topic", task_object_marker_topic_);
  declare_parameter<bool>("publish_marker", true);
  get_parameter("publish_marker", publish_marker_);

  std::string config_path =
    ament_index_cpp::get_package_share_directory(config_package_) + "/" + config_file_;
  RCLCPP_INFO_STREAM(get_logger(), "start loading config file : " << config_path);
  if (!loadConfig(config_path)) {
    std::ostringstream oss;
    oss << "Failed to load yaml config file for behavior, Please check " << config_file_
        << " is really exists or valid yaml file.";
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock, 1000, oss.str().c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "config file loaded!");

  using std::chrono_literals::operator""ms;

  blackboard_ = BT::Blackboard::create();

  auto client_options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "-r", std::string("__node:=") + get_name() + "_client_node", "--"});
  client_node_ = std::make_shared<rclcpp::Node>("_", client_options);
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);
  blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));

  if (publish_marker_) {
    task_object_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(task_object_marker_topic_, 1);
  }
  task_objects_array_sub_ =
    this->create_subscription<robotx_behavior_msgs::msg::TaskObjectsArrayStamped>(
      task_object_topic_, 1,
      std::bind(&BTPlannerComponent::taskObjectsArrayCallback, this, std::placeholders::_1));
#define CONNECT_TO_BLACKBOARD(TYPE, SUBSCRIPTION, TOPIC, BLACKBOARD_KEY)                       \
  SUBSCRIPTION = this->create_subscription<TYPE>(TOPIC, 1, [this](const TYPE::SharedPtr msg) { \
    blackboard_->set<TYPE::SharedPtr>(BLACKBOARD_KEY, msg);                                    \
  });

  CONNECT_TO_BLACKBOARD(
    hermite_path_msgs::msg::PlannerStatus, planner_status_sub_,
    "/local_waypoint_server/planner_status", "planner_status");
  CONNECT_TO_BLACKBOARD(
    geometry_msgs::msg::PoseStamped, current_pose_sub_, "/current_pose", "current_pose");
#undef CONNECT_TO_BLACKBOARD

  // registerPlannerStatusSubscription();
  // planner_status_sub_;
  loadPlugins();
  loadTree();

  logging_event_ptr_ =
    std::make_unique<robotx_bt_planner::LoggingEvent>(tree_.rootNode(), get_logger());

  publisher_zmq_ = std::make_unique<BT::PublisherZMQ>(tree_);

  using std::literals::chrono_literals::operator""s;
  auto interval = 1s / update_rate_;
  timer_ = create_wall_timer(500ms, std::bind(&BTPlannerComponent::timerCallback, this));
}

void BTPlannerComponent::taskObjectsArrayCallback(
  const robotx_behavior_msgs::msg::TaskObjectsArrayStamped::SharedPtr data)
{
  if (publish_marker_) {
    task_object_marker_pub_->publish(robotx_behavior_tree::toMarker(data));
  }
  blackboard_->set<robotx_behavior_msgs::msg::TaskObjectsArrayStamped::SharedPtr>(
    "task_objects", data);
}

bool BTPlannerComponent::loadConfig(const std::string & file_path)
{
  RCLCPP_INFO_STREAM(get_logger(), "loading yaml files from  : " << file_path);
  const boost::filesystem::path path(file_path);
  boost::system::error_code error;
  const bool result = boost::filesystem::exists(path, error);
  if (!result || error) {
    return false;
  }
  try {
    node_ = YAML::LoadFile(file_path);
  } catch (...) {
    return false;
  }
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
  return true;
}

void BTPlannerComponent::timerCallback()
{
  evaluationCallback();
  tree_.rootNode()->executeTick();
}

void BTPlannerComponent::loadPlugins()
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

bool BTPlannerComponent::loadTree()
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
    xml_string = addRosPorts(xml_string);
    RCLCPP_INFO_STREAM(get_logger(), "behavior xml : \n\n" << xml_string);
    tree_ = factory_.createTreeFromText(xml_string, blackboard_);
    RCLCPP_INFO(get_logger(), "behavior tree loaded: %s", file_path.c_str());
    return true;
  }
  return false;
}

void BTPlannerComponent::evaluationCallback()
{
  for (auto & block : evaluation_blocks_) {
    block->evaluate(lua_, blackboard_);
  }
}

struct Walker : pugi::xml_tree_walker
{
  Walker(const std::vector<std::string> & ports) : ports(ports) {}

  const std::vector<std::string> ports;

  virtual bool for_each(pugi::xml_node & node)
  {
    if (node.type() != pugi::node_element) return true;
    for (int i = 0; i < depth(); ++i) {
      if (node.name() == std::string("Action")) {
        for (const auto & port : ports) {
          if (node.attribute(port.c_str()).as_string() != std::string("{" + port + "}")) {
            node.append_attribute(port.c_str()) = std::string("{" + port + "}").c_str();
          }
        }
      }
    }
    return true;  // continue traversal
  }
};

std::vector<std::string> BTPlannerComponent::getRosPorts() const
{
  std::vector<std::string> ret;
  for (const auto & port : robotx_behavior_tree::ActionROS2Node::providedPorts()) {
    ret.emplace_back(port.first);
  }
  return ret;
}

std::string BTPlannerComponent::addRosPorts(const std::string & xml_string) const
{
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_string(xml_string.c_str());
  if (!result) {
    throw std::runtime_error("Failed to parse xml string, \n" + xml_string);
  }
  Walker walker(getRosPorts());
  doc.traverse(walker);
  const std::string xml_behavior_filename =
    "/tmp/robotx_bt_planner/" +
    boost::lexical_cast<std::string>(boost::uuids::random_generator()()) + "_generated.xml";
  RCLCPP_INFO_STREAM(get_logger(), xml_behavior_filename);
  if (!boost::filesystem::exists("/tmp/robotx_bt_planner")) {
    boost::filesystem::create_directory(boost::filesystem::path("/tmp/robotx_bt_planner"));
  }
  doc.save_file(xml_behavior_filename.c_str());
  std::ifstream ifs(xml_behavior_filename);
  return std::string(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
}

}  // namespace robotx_bt_planner

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(robotx_bt_planner::BTPlannerComponent)
