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

#include "rosgraph_cumulator/cumulator.hpp"

#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rosgraph_cumulator
{

GraphCumulator::GraphCumulator() : last_update_time_(0, 0, RCL_ROS_TIME) {}

void GraphCumulator::process_graph_message(const rosgraph_monitor_msgs::msg::Graph & graph_msg)
{
  // Update the last update time with the timestamp from the incoming message
  last_update_time_ =
    rclcpp::Time(graph_msg.timestamp.sec, graph_msg.timestamp.nanosec, RCL_ROS_TIME);

  // Update cumulative graph with the new graph information
  update_from_graph_message(graph_msg);
}

rosgraph_monitor_msgs::msg::Graph GraphCumulator::generate_cumulative_graph() const
{
  rosgraph_monitor_msgs::msg::Graph graph_msg;
  graph_msg.timestamp = last_update_time_;

  // Clear and prepare the message
  graph_msg.nodes.clear();
  graph_msg.nodes.reserve(node_history_.size());

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

    graph_msg.nodes.push_back(node_msg);
  }

  return graph_msg;
}

void GraphCumulator::clear()
{
  node_history_.clear();
  last_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

size_t GraphCumulator::get_node_count() const { return node_history_.size(); }

const rclcpp::Time & GraphCumulator::get_last_update_time() const { return last_update_time_; }

void GraphCumulator::set_last_update_time(const rclcpp::Time & time) { last_update_time_ = time; }

void GraphCumulator::update_from_graph_message(
  const rosgraph_monitor_msgs::msg::Graph & current_graph_msg)
{
  // Process each node in the current graph message
  for (const auto & node_msg : current_graph_msg.nodes) {
    // Add node to history if not already present
    if (node_history_.find(node_msg.name) == node_history_.end()) {
      node_history_.emplace(
        node_msg.name,
        CumulativeNodeInfo{
          node_msg.name, std::vector<CumulativeTopicEndpoint>(),
          std::vector<CumulativeTopicEndpoint>(), std::vector<CumulativeServiceEndpoint>(),
          std::vector<CumulativeServiceEndpoint>(), std::vector<CumulativeActionEndpoint>(),
          std::vector<CumulativeActionEndpoint>(),
          std::vector<rcl_interfaces::msg::ParameterDescriptor>(),
          rclcpp::Time(
            current_graph_msg.timestamp.sec, current_graph_msg.timestamp.nanosec, RCL_ROS_TIME)});
    }

    // Update the last seen time for this node
    node_history_.at(node_msg.name).last_seen = rclcpp::Time(
      current_graph_msg.timestamp.sec, current_graph_msg.timestamp.nanosec, RCL_ROS_TIME);

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

void GraphCumulator::add_publisher_to_history(
  const std::string & node_name, const rosgraph_monitor_msgs::msg::Topic & topic)
{
  // Ensure node exists in history
  auto node_it = node_history_.find(node_name);
  if (node_it == node_history_.end()) {
    return;
  }

  auto & node_info = node_it->second;

  // Create the endpoint to add
  CumulativeTopicEndpoint endpoint(topic.name, topic.type, topic.qos);

  // Add or update the endpoint
  upsert_topic_endpoint(node_info.publishers, endpoint);
}

void GraphCumulator::add_subscription_to_history(
  const std::string & node_name, const rosgraph_monitor_msgs::msg::Topic & topic)
{
  // Ensure node exists in history
  auto node_it = node_history_.find(node_name);
  if (node_it == node_history_.end()) {
    return;
  }

  auto & node_info = node_it->second;

  // Create the endpoint to add
  CumulativeTopicEndpoint endpoint(topic.name, topic.type, topic.qos);

  // Add or update the endpoint
  upsert_topic_endpoint(node_info.subscriptions, endpoint);
}

void GraphCumulator::add_service_to_history(
  const std::string & node_name, const rosgraph_monitor_msgs::msg::Service & service)
{
  // Ensure node exists in history
  auto node_it = node_history_.find(node_name);
  if (node_it == node_history_.end()) {
    return;
  }

  auto & node_info = node_it->second;

  // Create the endpoint to add
  CumulativeServiceEndpoint endpoint(service.name, service.type);

  // Add the endpoint if it doesn't already exist
  add_unique_service_endpoint(node_info.services, endpoint);
}

void GraphCumulator::add_client_to_history(
  const std::string & node_name, const rosgraph_monitor_msgs::msg::Service & client)
{
  // Ensure node exists in history
  auto node_it = node_history_.find(node_name);
  if (node_it == node_history_.end()) {
    return;
  }

  auto & node_info = node_it->second;

  // Create the endpoint to add
  CumulativeServiceEndpoint endpoint(client.name, client.type);

  // Add the endpoint if it doesn't already exist
  add_unique_service_endpoint(node_info.clients, endpoint);
}

void GraphCumulator::add_action_server_to_history(
  const std::string & node_name, const rosgraph_monitor_msgs::msg::Action & action)
{
  // Ensure node exists in history
  auto node_it = node_history_.find(node_name);
  if (node_it == node_history_.end()) {
    return;
  }

  auto & node_info = node_it->second;

  // Create the endpoint to add
  CumulativeActionEndpoint endpoint(action.name);

  // Add the endpoint if it doesn't already exist
  add_unique_action_endpoint(node_info.action_servers, endpoint);
}

void GraphCumulator::add_action_client_to_history(
  const std::string & node_name, const rosgraph_monitor_msgs::msg::Action & action)
{
  // Ensure node exists in history
  auto node_it = node_history_.find(node_name);
  if (node_it == node_history_.end()) {
    return;
  }

  auto & node_info = node_it->second;

  // Create the endpoint to add
  CumulativeActionEndpoint endpoint(action.name);

  // Add the endpoint if it doesn't already exist
  add_unique_action_endpoint(node_info.action_clients, endpoint);
}

void GraphCumulator::update_node_parameters(
  const std::string & node_name,
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & parameters)
{
  auto node_it = node_history_.find(node_name);
  if (node_it != node_history_.end()) {
    auto & node_info = node_it->second;
    node_info.parameters = parameters;
  }
}

std::vector<CumulativeTopicEndpoint>::const_iterator GraphCumulator::find_topic_endpoint(
  const std::vector<CumulativeTopicEndpoint> & endpoints,
  const CumulativeTopicEndpoint & endpoint) const
{
  return std::find_if(
    endpoints.begin(), endpoints.end(),
    [&endpoint](const CumulativeTopicEndpoint & existing_endpoint) {
      return existing_endpoint.topic_name == endpoint.topic_name &&
             existing_endpoint.topic_type == endpoint.topic_type;
    });
}

std::vector<CumulativeServiceEndpoint>::const_iterator GraphCumulator::find_service_endpoint(
  const std::vector<CumulativeServiceEndpoint> & endpoints,
  const CumulativeServiceEndpoint & endpoint) const
{
  return std::find_if(
    endpoints.begin(), endpoints.end(),
    [&endpoint](const CumulativeServiceEndpoint & existing_endpoint) {
      return existing_endpoint.service_name == endpoint.service_name &&
             existing_endpoint.service_type == endpoint.service_type;
    });
}

std::vector<CumulativeActionEndpoint>::const_iterator GraphCumulator::find_action_endpoint(
  const std::vector<CumulativeActionEndpoint> & endpoints,
  const CumulativeActionEndpoint & endpoint) const
{
  return std::find_if(
    endpoints.begin(), endpoints.end(),
    [&endpoint](const CumulativeActionEndpoint & existing_endpoint) {
      return existing_endpoint.action_name == endpoint.action_name;
    });
}

void GraphCumulator::upsert_topic_endpoint(
  std::vector<CumulativeTopicEndpoint> & endpoints, const CumulativeTopicEndpoint & endpoint)
{
  auto it = find_topic_endpoint(endpoints, endpoint);

  if (it == endpoints.end()) {
    // New endpoint, add to history
    endpoints.emplace_back(endpoint);
  } else {
    // Existing endpoint, update QoS profile to latest
    const_cast<CumulativeTopicEndpoint &>(*it).qos_profile = endpoint.qos_profile;
  }
}

void GraphCumulator::add_unique_service_endpoint(
  std::vector<CumulativeServiceEndpoint> & endpoints,
  const CumulativeServiceEndpoint & endpoint) const
{
  auto it = find_service_endpoint(endpoints, endpoint);

  if (it == endpoints.end()) {
    // New endpoint, add to history
    endpoints.emplace_back(endpoint);
  }
  // If it already exists, do nothing (avoid duplicates)
}

void GraphCumulator::add_unique_action_endpoint(
  std::vector<CumulativeActionEndpoint> & endpoints,
  const CumulativeActionEndpoint & endpoint) const
{
  auto it = find_action_endpoint(endpoints, endpoint);

  if (it == endpoints.end()) {
    // New endpoint, add to history
    endpoints.emplace_back(endpoint);
  }
  // If it already exists, do nothing (avoid duplicates)
}

}  // namespace rosgraph_cumulator
