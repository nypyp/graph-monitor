// Copyright 2025, Bonsai Robotics, Inc - All Rights Reserved
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

#include "rosgraph_monitor/cumulative_graph.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace rosgraph_monitor
{

void CumulativeGraph::update_from_graph_msg(const rosgraph_monitor_msgs::msg::Graph & current_graph_msg)
{
  // Process each node in the current graph message
  for (const auto & node_msg : current_graph_msg.nodes) {
    // Add node to history if not already present
    if (node_history_.find(node_msg.name) == node_history_.end()) {
      node_history_.emplace(
        node_msg.name,
        CumulativeNodeInfo{
          node_msg.name,
          std::vector<CumulativeEndpoint>(),
          std::vector<CumulativeEndpoint>(),
          std::vector<rcl_interfaces::msg::ParameterDescriptor>()});
    }

    // Process publishers
    for (const auto & publisher : node_msg.publishers) {
      add_publisher_to_history(node_msg.name, publisher);
    }

    // Process subscriptions
    for (const auto & subscription : node_msg.subscriptions) {
      add_subscription_to_history(node_msg.name, subscription);
    }

    // Update parameters (always keep latest)
    update_node_parameters(node_msg.name, node_msg.parameters);
  }
}

void CumulativeGraph::add_publisher_to_history(
  const std::string & node_name,
  const rosgraph_monitor_msgs::msg::Topic & topic)
{
  auto & node_info = node_history_.at(node_name);

  // Check if this publisher already exists in history
  auto it = std::find_if(
    node_info.publishers.begin(), node_info.publishers.end(),
    [&topic](const CumulativeEndpoint & endpoint) {
      return endpoint.topic_name == topic.name && endpoint.topic_type == topic.type;
    });

  if (it == node_info.publishers.end()) {
    // New publisher, add to history
    node_info.publishers.push_back(
      {topic.name, topic.type, topic.qos});
  } else {
    // Existing publisher, update QoS profile to latest
    it->qos_profile = topic.qos;
  }
}

void CumulativeGraph::add_subscription_to_history(
  const std::string & node_name,
  const rosgraph_monitor_msgs::msg::Topic & topic)
{
  auto & node_info = node_history_.at(node_name);

  // Check if this subscription already exists in history
  auto it = std::find_if(
    node_info.subscriptions.begin(), node_info.subscriptions.end(),
    [&topic](const CumulativeEndpoint & endpoint) {
      return endpoint.topic_name == topic.name && endpoint.topic_type == topic.type;
    });

  if (it == node_info.subscriptions.end()) {
    // New subscription, add to history
    node_info.subscriptions.push_back(
      {topic.name, topic.type, topic.qos});
  } else {
    // Existing subscription, update QoS profile to latest
    it->qos_profile = topic.qos;
  }
}

void CumulativeGraph::update_node_parameters(
  const std::string & node_name,
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & parameters)
{
  if (node_history_.find(node_name) != node_history_.end()) {
    auto & node_info = node_history_.at(node_name);
    node_info.parameters = parameters;
  }
}


void CumulativeGraph::fill_rosgraph_msg(rosgraph_monitor_msgs::msg::Graph & msg) const
{
  // Clear and prepare the message
  msg.nodes.clear();
  msg.nodes.reserve(node_history_.size());

  // Fill in node information
  for (const auto & [node_name, node_info] : node_history_) {
    rosgraph_monitor_msgs::msg::NodeInfo node_msg;
    node_msg.name = node_name;

    // Add publishers
    node_msg.publishers.reserve(node_info.publishers.size());
    for (const auto & publisher : node_info.publishers) {
      rosgraph_monitor_msgs::msg::Topic topic_msg;
      topic_msg.name = publisher.topic_name;
      topic_msg.type = publisher.topic_type;
      topic_msg.qos = publisher.qos_profile;
      node_msg.publishers.push_back(topic_msg);
    }

    // Add subscriptions
    node_msg.subscriptions.reserve(node_info.subscriptions.size());
    for (const auto & subscription : node_info.subscriptions) {
      rosgraph_monitor_msgs::msg::Topic topic_msg;
      topic_msg.name = subscription.topic_name;
      topic_msg.type = subscription.topic_type;
      topic_msg.qos = subscription.qos_profile;
      node_msg.subscriptions.push_back(topic_msg);
    }

    // Add parameters
    node_msg.parameters = node_info.parameters;
    // Note: parameter_values is left empty as in the original implementation

    msg.nodes.push_back(node_msg);
  }
}

}  // namespace rosgraph_monitor
