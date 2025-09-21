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

#include "rosgraph_monitor/cumulative_graph_node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

namespace rosgraph_monitor
{

CumulativeGraphNode::CumulativeGraphNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("rosgraph_cumulative", options),
  sub_graph_(
    create_subscription<rosgraph_monitor_msgs::msg::Graph>(
      "/rosgraph",
      rclcpp::QoS(10),
      std::bind(&CumulativeGraphNode::on_graph_msg, this, std::placeholders::_1))),
  pub_cumulative_graph_(
    create_publisher<rosgraph_monitor_msgs::msg::Graph>(
      "/rosgraph_cumulative",
      rclcpp::QoS(1).transient_local().reliable()))
{
}

void CumulativeGraphNode::on_graph_msg(const rosgraph_monitor_msgs::msg::Graph::SharedPtr msg)
{
  // Update cumulative graph with the new graph information
  update_from_graph_msg(*msg);

  // Publish cumulative graph
  publish_cumulative_graph();
}

void CumulativeGraphNode::publish_cumulative_graph()
{
  auto msg = std::make_unique<rosgraph_monitor_msgs::msg::Graph>();
  fill_rosgraph_msg(*msg);
  msg->timestamp = get_clock()->now();
  pub_cumulative_graph_->publish(std::move(msg));
}

void CumulativeGraphNode::update_from_graph_msg(const rosgraph_monitor_msgs::msg::Graph & current_graph_msg)
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
          std::vector<CumulativeService>(),
          std::vector<CumulativeService>(),
          std::vector<CumulativeAction>(),
          std::vector<CumulativeAction>(),
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

    // Process services
    for (const auto & service : node_msg.services) {
      add_service_to_history(node_msg.name, service);
    }

    // Process clients
    for (const auto & client : node_msg.clients) {
      add_client_to_history(node_msg.name, client);
    }

    // Process action servers
    for (const auto & action_server : node_msg.action_servers) {
      add_action_server_to_history(node_msg.name, action_server);
    }

    // Process action clients
    for (const auto & action_client : node_msg.action_clients) {
      add_action_client_to_history(node_msg.name, action_client);
    }

    // Update parameters (always keep latest)
    update_node_parameters(node_msg.name, node_msg.parameters);
  }
}

void CumulativeGraphNode::add_publisher_to_history(
  const std::string & node_name,
  const rosgraph_monitor_msgs::msg::Topic & topic)
{
  // Ensure node exists in history
  if (node_history_.find(node_name) == node_history_.end()) {
    return;
  }

  auto & node_info = node_history_.at(node_name);

  // Check if this publisher already exists in history
  auto it = std::find_if(
    node_info.publishers.begin(), node_info.publishers.end(),
    [&topic](const CumulativeEndpoint & endpoint) {
      return endpoint.topic_name == topic.name && endpoint.topic_type == topic.type;
    });

  if (it == node_info.publishers.end()) {
    // New publisher, add to history
    node_info.publishers.emplace_back(
      topic.name, topic.type, topic.qos);
  } else {
    // Existing publisher, update QoS profile to latest
    it->qos_profile = topic.qos;
  }
}

void CumulativeGraphNode::add_subscription_to_history(
  const std::string & node_name,
  const rosgraph_monitor_msgs::msg::Topic & topic)
{
  // Ensure node exists in history
  if (node_history_.find(node_name) == node_history_.end()) {
    return;
  }

  auto & node_info = node_history_.at(node_name);

  // Check if this subscription already exists in history
  auto it = std::find_if(
    node_info.subscriptions.begin(), node_info.subscriptions.end(),
    [&topic](const CumulativeEndpoint & endpoint) {
      return endpoint.topic_name == topic.name && endpoint.topic_type == topic.type;
    });

  if (it == node_info.subscriptions.end()) {
    // New subscription, add to history
    node_info.subscriptions.emplace_back(
      topic.name, topic.type, topic.qos);
  } else {
    // Existing subscription, update QoS profile to latest
    it->qos_profile = topic.qos;
  }
}

void CumulativeGraphNode::add_service_to_history(
  const std::string & node_name,
  const rosgraph_monitor_msgs::msg::Service & service)
{
  // Ensure node exists in history
  if (node_history_.find(node_name) == node_history_.end()) {
    return;
  }

  auto & node_info = node_history_.at(node_name);

  // Check if this service already exists in history
  auto it = std::find_if(
    node_info.services.begin(), node_info.services.end(),
    [&service](const CumulativeService & s) {
      return s.service_name == service.name && s.service_type == service.type;
    });

  if (it == node_info.services.end()) {
    // New service, add to history
    node_info.services.emplace_back(
      service.name, service.type);
  }
}

void CumulativeGraphNode::add_client_to_history(
  const std::string & node_name,
  const rosgraph_monitor_msgs::msg::Service & client)
{
  // Ensure node exists in history
  if (node_history_.find(node_name) == node_history_.end()) {
    return;
  }

  auto & node_info = node_history_.at(node_name);

  // Check if this client already exists in history
  auto it = std::find_if(
    node_info.clients.begin(), node_info.clients.end(),
    [&client](const CumulativeService & c) {
      return c.service_name == client.name && c.service_type == client.type;
    });

  if (it == node_info.clients.end()) {
    // New client, add to history
    node_info.clients.emplace_back(
      client.name, client.type);
  }
}

void CumulativeGraphNode::add_action_server_to_history(
  const std::string & node_name,
  const rosgraph_monitor_msgs::msg::Action & action)
{
  // Ensure node exists in history
  if (node_history_.find(node_name) == node_history_.end()) {
    return;
  }

  auto & node_info = node_history_.at(node_name);

  // Check if this action server already exists in history
  auto it = std::find_if(
    node_info.action_servers.begin(), node_info.action_servers.end(),
    [&action](const CumulativeAction & a) {
      return a.action_name == action.name;
    });

  if (it == node_info.action_servers.end()) {
    // New action server, add to history
    node_info.action_servers.emplace_back(
      action.name);
  }
}

void CumulativeGraphNode::add_action_client_to_history(
  const std::string & node_name,
  const rosgraph_monitor_msgs::msg::Action & action)
{
  // Ensure node exists in history
  if (node_history_.find(node_name) == node_history_.end()) {
    return;
  }

  auto & node_info = node_history_.at(node_name);

  // Check if this action client already exists in history
  auto it = std::find_if(
    node_info.action_clients.begin(), node_info.action_clients.end(),
    [&action](const CumulativeAction & a) {
      return a.action_name == action.name;
    });

  if (it == node_info.action_clients.end()) {
    // New action client, add to history
    node_info.action_clients.emplace_back(
      action.name);
  }
}

void CumulativeGraphNode::update_node_parameters(
  const std::string & node_name,
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & parameters)
{
  if (node_history_.find(node_name) != node_history_.end()) {
    auto & node_info = node_history_.at(node_name);
    node_info.parameters = parameters;
  }
}

void CumulativeGraphNode::fill_rosgraph_msg(rosgraph_monitor_msgs::msg::Graph & msg) const
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

    // Add services
    node_msg.services.reserve(node_info.services.size());
    for (const auto & service : node_info.services) {
      rosgraph_monitor_msgs::msg::Service service_msg;
      service_msg.name = service.service_name;
      service_msg.type = service.service_type;
      node_msg.services.push_back(service_msg);
    }

    // Add clients
    node_msg.clients.reserve(node_info.clients.size());
    for (const auto & client : node_info.clients) {
      rosgraph_monitor_msgs::msg::Service client_msg;
      client_msg.name = client.service_name;
      client_msg.type = client.service_type;
      node_msg.clients.push_back(client_msg);
    }

    // Add action servers
    node_msg.action_servers.reserve(node_info.action_servers.size());
    for (const auto & action_server : node_info.action_servers) {
      rosgraph_monitor_msgs::msg::Action action_msg;
      action_msg.name = action_server.action_name;
      node_msg.action_servers.push_back(action_msg);
    }

    // Add action clients
    node_msg.action_clients.reserve(node_info.action_clients.size());
    for (const auto & action_client : node_info.action_clients) {
      rosgraph_monitor_msgs::msg::Action action_msg;
      action_msg.name = action_client.action_name;
      node_msg.action_clients.push_back(action_msg);
    }

    // Add parameters
    node_msg.parameters = node_info.parameters;
    // Note: parameter_values is left empty as in the original implementation

    msg.nodes.push_back(node_msg);
  }
}

}  // namespace rosgraph_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(rosgraph_monitor::CumulativeGraphNode)
