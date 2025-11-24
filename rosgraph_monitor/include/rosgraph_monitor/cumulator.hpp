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

#ifndef ROSGRAPH_MONITOR__CUMULATOR_HPP_
#define ROSGRAPH_MONITOR__CUMULATOR_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rosgraph_monitor_msgs/msg/graph.hpp"
#include "rosgraph_monitor_msgs/msg/topic.hpp"
#include "rosgraph_monitor_msgs/msg/service.hpp"
#include "rosgraph_monitor_msgs/msg/action.hpp"
#include "rosgraph_monitor_msgs/msg/qos_profile.hpp"

namespace rosgraph_monitor
{

/**
 * @brief Represents a cumulative topic endpoint (publisher or subscription)
 */
struct CumulativeTopicEndpoint
{
  std::string topic_name;
  std::string topic_type;
  rosgraph_monitor_msgs::msg::QosProfile qos_profile;

  /**
   * @brief Default constructor
   */
  CumulativeTopicEndpoint()
  : topic_name(""), topic_type(""), qos_profile() {}

  /**
   * @brief Constructor with parameters
   * @param name Topic name
   * @param type Topic type
   * @param qos QoS profile
   */
  CumulativeTopicEndpoint(
    const std::string & name,
    const std::string & type,
    const rosgraph_monitor_msgs::msg::QosProfile & qos)
  : topic_name(name), topic_type(type), qos_profile(qos) {}

  /**
   * @brief Equality operator for comparison
   */
  bool operator==(const CumulativeTopicEndpoint & other) const
  {
    return topic_name == other.topic_name && topic_type == other.topic_type;
  }
};

/**
 * @brief Represents a cumulative service endpoint (server or client)
 */
struct CumulativeServiceEndpoint
{
  std::string service_name;
  std::string service_type;

  /**
   * @brief Default constructor
   */
  CumulativeServiceEndpoint()
  : service_name(""), service_type("") {}

  /**
   * @brief Constructor with parameters
   * @param name Service name
   * @param type Service type
   */
  CumulativeServiceEndpoint(const std::string & name, const std::string & type)
  : service_name(name), service_type(type) {}

  /**
   * @brief Equality operator for comparison
   */
  bool operator==(const CumulativeServiceEndpoint & other) const
  {
    return service_name == other.service_name && service_type == other.service_type;
  }
};

/**
 * @brief Represents a cumulative action endpoint (server or client)
 */
struct CumulativeActionEndpoint
{
  std::string action_name;

  /**
   * @brief Default constructor
   */
  CumulativeActionEndpoint()
  : action_name("") {}

  /**
   * @brief Constructor with parameters
   * @param name Action name
   */
  explicit CumulativeActionEndpoint(const std::string & name)
  : action_name(name) {}

  /**
   * @brief Equality operator for comparison
   */
  bool operator==(const CumulativeActionEndpoint & other) const
  {
    return action_name == other.action_name;
  }
};

/**
 * @brief Stores cumulative information for a single node
 */
struct CumulativeNodeInfo
{
  std::string name;
  std::vector<CumulativeTopicEndpoint> publishers;
  std::vector<CumulativeTopicEndpoint> subscriptions;
  std::vector<CumulativeServiceEndpoint> services;
  std::vector<CumulativeServiceEndpoint> clients;
  std::vector<CumulativeActionEndpoint> action_servers;
  std::vector<CumulativeActionEndpoint> action_clients;
  std::vector<rcl_interfaces::msg::ParameterDescriptor> parameters;
  rclcpp::Time last_seen;

  /**
   * @brief Default constructor
   */
  CumulativeNodeInfo()
  : name(""),
    publishers(),
    subscriptions(),
    services(),
    clients(),
    action_servers(),
    action_clients(),
    parameters(),
    last_seen(0, 0, RCL_ROS_TIME) {}

  /**
   * @brief Constructor with parameters
   * @param node_name Name of the node
   * @param pubs Publishers list
   * @param subs Subscriptions list
   * @param srvs Services list
   * @param clnts Clients list
   * @param act_srvs Action servers list
   * @param act_clnts Action clients list
   * @param params Parameters list
   * @param seen_time Time when node was last seen
   */
  CumulativeNodeInfo(
    const std::string & node_name,
    const std::vector<CumulativeTopicEndpoint> & pubs,
    const std::vector<CumulativeTopicEndpoint> & subs,
    const std::vector<CumulativeServiceEndpoint> & srvs,
    const std::vector<CumulativeServiceEndpoint> & clnts,
    const std::vector<CumulativeActionEndpoint> & act_srvs,
    const std::vector<CumulativeActionEndpoint> & act_clnts,
    const std::vector<rcl_interfaces::msg::ParameterDescriptor> & params,
    const rclcpp::Time & seen_time = rclcpp::Time(0, 0, RCL_ROS_TIME))
  : name(node_name),
    publishers(pubs),
    subscriptions(subs),
    services(srvs),
    clients(clnts),
    action_servers(act_srvs),
    action_clients(act_clnts),
    parameters(params),
    last_seen(seen_time) {}
};

/**
 * @brief Processes and maintains cumulative ROS graph information
 * 
 * This class processes incoming graph messages and maintains a cumulative view
 * of the ROS graph, preserving information about all discovered nodes and their
 * communication endpoints (topics, services, actions) over time.
 */
/**
 * @brief ROS 2 node that processes incoming graph messages and maintains cumulative graph information
 * 
 * This node subscribes to the graph messages and maintains a cumulative view
 * of the ROS graph, preserving information about all discovered nodes and 
 * their communication endpoints over time.
 */
class GraphCumulator : public rclcpp::Node
{
public:
  explicit GraphCumulator(const rclcpp::NodeOptions & options);

  /**
   * @brief Process a new graph message and update cumulative state
   * @param graph_msg The incoming graph message to process
   */
  void process_graph_message(const rosgraph_monitor_msgs::msg::Graph::SharedPtr graph_msg);

  /**
   * @brief Generate a cumulative graph message with all discovered information
   * @return A graph message containing cumulative information
   */
  rosgraph_monitor_msgs::msg::Graph generate_cumulative_graph() const;

  /**
   * @brief Clear all accumulated graph information
   */
  void clear();

  /**
   * @brief Get the current count of nodes in the cumulative graph
   * @return Number of nodes currently tracked
   */
  size_t get_node_count() const;

  /**
   * @brief Get the timestamp of the most recently processed graph message
   * @return Timestamp of last update
   */
  rclcpp::Time get_last_update_time() const;

protected:
  /**
   * @brief Callback for incoming graph messages
   * @param msg The incoming graph message to process
   */
  void on_graph_message(const rosgraph_monitor_msgs::msg::Graph::SharedPtr msg);

  /**
   * @brief Publish the current cumulative graph
   */
  void publish_cumulative_graph();

  /**
   * @brief Add a publisher to a node's history if it doesn't already exist
   * @param node_name Name of the node
   * @param topic The topic information
   */
  void add_publisher_to_history(
    const std::string & node_name,
    const rosgraph_monitor_msgs::msg::Topic & topic);

  /**
   * @brief Add a subscription to a node's history if it doesn't already exist
   * @param node_name Name of the node
   * @param topic The topic information
   */
  void add_subscription_to_history(
    const std::string & node_name,
    const rosgraph_monitor_msgs::msg::Topic & topic);

  /**
   * @brief Add a service to a node's history if it doesn't already exist
   * @param node_name Name of the node
   * @param service The service information
   */
  void add_service_to_history(
    const std::string & node_name,
    const rosgraph_monitor_msgs::msg::Service & service);

  /**
   * @brief Add a client to a node's history if it doesn't already exist
   * @param node_name Name of the node
   * @param client The client information
   */
  void add_client_to_history(
    const std::string & node_name,
    const rosgraph_monitor_msgs::msg::Service & client);

  /**
   * @brief Add an action server to a node's history if it doesn't already exist
   * @param node_name Name of the node
   * @param action The action information
   */
  void add_action_server_to_history(
    const std::string & node_name,
    const rosgraph_monitor_msgs::msg::Action & action);

  /**
   * @brief Add an action client to a node's history if it doesn't already exist
   * @param node_name Name of the node
   * @param action The action information
   */
  void add_action_client_to_history(
    const std::string & node_name,
    const rosgraph_monitor_msgs::msg::Action & action);

  /**
   * @brief Update parameters for a node
   * @param node_name Name of the node
   * @param parameters Latest parameter descriptors
   */
  void update_node_parameters(
    const std::string & node_name,
    const std::vector<rcl_interfaces::msg::ParameterDescriptor> & parameters);

  /**
   * @brief Update the cumulative graph with current graph information from a Graph message
   * @param current_graph_msg The current graph state from RosGraphMonitor
   */
  void update_from_graph_message(const rosgraph_monitor_msgs::msg::Graph & current_graph_msg);

  /**
   * @brief Check if a topic endpoint already exists in the list
   * @param endpoints List of endpoints to search
   * @param endpoint Endpoint to find
   * @return Iterator to the endpoint if found, end() otherwise
   */
  std::vector<CumulativeTopicEndpoint>::const_iterator find_topic_endpoint(
    const std::vector<CumulativeTopicEndpoint> & endpoints,
    const CumulativeTopicEndpoint & endpoint) const;

  /**
   * @brief Check if a service endpoint already exists in the list
   * @param endpoints List of endpoints to search
   * @param endpoint Endpoint to find
   * @return Iterator to the endpoint if found, end() otherwise
   */
  std::vector<CumulativeServiceEndpoint>::const_iterator find_service_endpoint(
    const std::vector<CumulativeServiceEndpoint> & endpoints,
    const CumulativeServiceEndpoint & endpoint) const;

  /**
   * @brief Check if an action endpoint already exists in the list
   * @param endpoints List of endpoints to search
   * @param endpoint Endpoint to find
   * @return Iterator to the endpoint if found, end() otherwise
   */
  std::vector<CumulativeActionEndpoint>::const_iterator find_action_endpoint(
    const std::vector<CumulativeActionEndpoint> & endpoints,
    const CumulativeActionEndpoint & endpoint) const;

  /**
   * @brief Add or update a topic endpoint in the list
   * @param endpoints List of endpoints to modify
   * @param endpoint Endpoint to add or update
   */
  void upsert_topic_endpoint(
    std::vector<CumulativeTopicEndpoint> & endpoints,
    const CumulativeTopicEndpoint & endpoint);

  /**
   * @brief Add a service endpoint to the list if it doesn't exist
   * @param endpoints List of endpoints to modify
   * @param endpoint Endpoint to add
   */
  void add_unique_service_endpoint(
    std::vector<CumulativeServiceEndpoint> & endpoints,
    const CumulativeServiceEndpoint & endpoint);

  /**
   * @brief Add an action endpoint to the list if it doesn't exist
   * @param endpoints List of endpoints to modify
   * @param endpoint Endpoint to add
   */
  void add_unique_action_endpoint(
    std::vector<CumulativeActionEndpoint> & endpoints,
    const CumulativeActionEndpoint & endpoint);

  // Map of node names to their cumulative information
  std::unordered_map<std::string, CumulativeNodeInfo> node_history_;

  // Timestamp of the most recent graph update
  rclcpp::Time last_update_time_;

  // Subscriptions and publishers
  rclcpp::Subscription<rosgraph_monitor_msgs::msg::Graph>::SharedPtr sub_graph_;
  rclcpp::Publisher<rosgraph_monitor_msgs::msg::Graph>::SharedPtr pub_cumulative_graph_;
};

}  // namespace rosgraph_monitor

#endif  // ROSGRAPH_MONITOR__CUMULATOR_HPP_