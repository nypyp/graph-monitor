// Copyright 2024, Bonsai Robotics, Inc - All Rights Reserved
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

#ifndef ROSGRAPH_MONITOR__VALIDATOR_HPP_
#define ROSGRAPH_MONITOR__VALIDATOR_HPP_

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_monitor/validator_parameters.hpp>
#include <unordered_map>

#include "rosgraph_monitor_msgs/msg/graph.hpp"
#include "rosgraph_monitor_msgs/msg/node_info.hpp"

namespace rosgraph_monitor
{
enum class QosMatchStrategy
{
  EXACT,
  COMPATIBLE,
  KEY_FIELDS_ONLY
};

struct TopicEndpoint
{
  std::string name;
  std::string type;
  rosgraph_monitor_msgs::msg::QosProfile qos;
  // Difine Qos match rules
  bool operator==(const TopicEndpoint & other) const
  {
    if (name != other.name || type != other.type) return false;
    return qos.reliability == other.qos.reliability && qos.durability == other.qos.durability;
  }
  bool operator!=(const TopicEndpoint & other) const { return !(*this == other); }
  // Difine Topic match rules
  bool operator<(const TopicEndpoint & other) const
  {
    if (name != other.name) return name < other.name;
    return type < other.type;
  }
};

struct NodeView
{
  std::set<TopicEndpoint> publishers;
  std::set<TopicEndpoint> subscribers;
};

class GraphValidator : public rclcpp::Node
{
public:
  explicit GraphValidator(const rclcpp::NodeOptions & options);

  static std::vector<std::pair<std::string, uint8_t>> compute_diff(
    const std::unordered_map<std::string, NodeView> & config,
    const std::unordered_map<std::string, NodeView> & live);

private:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  std::shared_ptr<validator_parameters::ParamListener> param_listener_;
  validator_parameters::Params params_;

  void on_graph(const rosgraph_monitor_msgs::msg::Graph::SharedPtr msg);
  void validate_nodes(diagnostic_updater::DiagnosticStatusWrapper & status);
  void validate_topics(diagnostic_updater::DiagnosticStatusWrapper & status);
  void validate_qos(diagnostic_updater::DiagnosticStatusWrapper & status);

  void load_config(const std::string & file_path);
  static NodeView normalize_node(const rosgraph_monitor_msgs::msg::NodeInfo & node);

  std::unordered_map<std::string, NodeView> config_graph_;
  std::unordered_map<std::string, NodeView> live_graph_;
  diagnostic_updater::Updater updater_;
  rclcpp::Subscription<rosgraph_monitor_msgs::msg::Graph>::SharedPtr graph_sub_;
};

}  // namespace rosgraph_monitor

#endif  //  ROSGRAPH_MONITOR__VALIDATOR_HPP_