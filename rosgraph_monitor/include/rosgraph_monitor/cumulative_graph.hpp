// Copyright 2025, nypyp
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

#ifndef ROSGRAPH_MONITOR__CUMULATIVE_GRAPH_HPP_
#define ROSGRAPH_MONITOR__CUMULATIVE_GRAPH_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rosgraph_monitor_msgs/msg/graph.hpp"
#include "rosgraph_monitor_msgs/msg/node_info.hpp"

namespace rosgraph_monitor
{

/// @brief Maintains cumulative information about the ROS graph over time.
/// Keeps track of all nodes and endpoints that have ever appeared in the graph.
class CumulativeGraph
{
public:
  /// @brief Constructor
  CumulativeGraph() = default;

  /// @brief Update the cumulative graph with current graph information from a Graph message
  /// @param current_graph_msg The current graph state from RosGraphMonitor
  void update_from_graph_msg(const rosgraph_monitor_msgs::msg::Graph & current_graph_msg);

  /// @brief Fill a Graph message containing cumulative graph state
  void fill_rosgraph_msg(rosgraph_monitor_msgs::msg::Graph & msg) const;

protected:
  struct CumulativeEndpoint
  {
    std::string topic_name;
    std::string topic_type;
    rosgraph_monitor_msgs::msg::QosProfile qos_profile;
  };

  struct CumulativeNodeInfo
  {
    std::string name;
    std::vector<CumulativeEndpoint> publishers;
    std::vector<CumulativeEndpoint> subscriptions;
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameters;
  };

  /// @brief Add a new publisher to a node's history
  /// @param node_name Name of the node
  /// @param topic The topic information
  void add_publisher_to_history(
    const std::string & node_name,
    const rosgraph_monitor_msgs::msg::Topic & topic);

  /// @brief Add a new subscription to a node's history
  /// @param node_name Name of the node
  /// @param topic The topic information
  void add_subscription_to_history(
    const std::string & node_name,
    const rosgraph_monitor_msgs::msg::Topic & topic);

  /// @brief Update parameters for a node
  /// @param node_name Name of the node
  /// @param parameters Latest parameter descriptors
  void update_node_parameters(
    const std::string & node_name,
    const std::vector<rcl_interfaces::msg::ParameterDescriptor> & parameters);

  /// @brief Convert rclcpp::QoS to message format
  /// @param qos_profile The QoS profile to convert
  /// @return Message format of the QoS profile
  rosgraph_monitor_msgs::msg::QosProfile to_msg(const rclcpp::QoS & qos_profile) const;

private:
  // Map of node names to their cumulative information
  std::unordered_map<std::string, CumulativeNodeInfo> node_history_;
};

}  // namespace rosgraph_monitor

#endif  // ROSGRAPH_MONITOR__CUMULATIVE_GRAPH_HPP_
