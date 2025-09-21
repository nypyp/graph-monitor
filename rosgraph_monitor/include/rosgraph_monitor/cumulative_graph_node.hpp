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

#ifndef ROSGRAPH_MONITOR__CUMULATIVE_GRAPH_NODE_HPP_
#define ROSGRAPH_MONITOR__CUMULATIVE_GRAPH_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rosgraph_monitor_msgs/msg/graph.hpp"
#include "rosgraph_monitor_msgs/msg/topic.hpp"
#include "rosgraph_monitor_msgs/msg/service.hpp"
#include "rosgraph_monitor_msgs/msg/action.hpp"
#include "rosgraph_monitor_msgs/msg/qos_profile.hpp"

namespace rosgraph_monitor
{

class CumulativeGraphNode : public rclcpp::Node
{
public:
  explicit CumulativeGraphNode(const rclcpp::NodeOptions & options);

protected:
  void on_graph_msg(const rosgraph_monitor_msgs::msg::Graph::SharedPtr msg);
  void publish_cumulative_graph();

  struct CumulativeEndpoint
  {
    std::string topic_name;
    std::string topic_type;
    rosgraph_monitor_msgs::msg::QosProfile qos_profile;

    CumulativeEndpoint()
    : topic_name(""), topic_type(""), qos_profile() {}

    CumulativeEndpoint(
      const std::string & name,
      const std::string & type,
      const rosgraph_monitor_msgs::msg::QosProfile & qos)
    : topic_name(name), topic_type(type), qos_profile(qos) {}
  };

  struct CumulativeService
  {
    std::string service_name;
    std::string service_type;

    CumulativeService()
    : service_name(""), service_type("") {}

    CumulativeService(const std::string & name, const std::string & type)
    : service_name(name), service_type(type) {}
  };

  struct CumulativeAction
  {
    std::string action_name;

    CumulativeAction()
    : action_name("") {}

    explicit CumulativeAction(const std::string & name)
    : action_name(name) {}
  };

  struct CumulativeNodeInfo
  {
    std::string name;
    std::vector<CumulativeEndpoint> publishers;
    std::vector<CumulativeEndpoint> subscriptions;
    std::vector<CumulativeService> services;
    std::vector<CumulativeService> clients;
    std::vector<CumulativeAction> action_servers;
    std::vector<CumulativeAction> action_clients;
    std::vector<rcl_interfaces::msg::ParameterDescriptor> parameters;

    CumulativeNodeInfo()
    : name(""),
      publishers(),
      subscriptions(),
      services(),
      clients(),
      action_servers(),
      action_clients(),
      parameters() {}

    CumulativeNodeInfo(
      const std::string & node_name,
      const std::vector<CumulativeEndpoint> & pubs,
      const std::vector<CumulativeEndpoint> & subs,
      const std::vector<CumulativeService> & srvs,
      const std::vector<CumulativeService> & clnts,
      const std::vector<CumulativeAction> & act_srvs,
      const std::vector<CumulativeAction> & act_clnts,
      const std::vector<rcl_interfaces::msg::ParameterDescriptor> & params)
    : name(node_name),
      publishers(pubs),
      subscriptions(subs),
      services(srvs),
      clients(clnts),
      action_servers(act_srvs),
      action_clients(act_clnts),
      parameters(params) {}
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

  /// @brief Add a new service to a node's history
  /// @param node_name Name of the node
  /// @param service The service information
  void add_service_to_history(
    const std::string & node_name,
    const rosgraph_monitor_msgs::msg::Service & service);

  /// @brief Add a new client to a node's history
  /// @param node_name Name of the node
  /// @param client The client information
  void add_client_to_history(
    const std::string & node_name,
    const rosgraph_monitor_msgs::msg::Service & client);

  /// @brief Add a new action server to a node's history
  /// @param node_name Name of the node
  /// @param action The action information
  void add_action_server_to_history(
    const std::string & node_name,
    const rosgraph_monitor_msgs::msg::Action & action);

  /// @brief Add a new action client to a node's history
  /// @param node_name Name of the node
  /// @param action The action information
  void add_action_client_to_history(
    const std::string & node_name,
    const rosgraph_monitor_msgs::msg::Action & action);

  /// @brief Update parameters for a node
  /// @param node_name Name of the node
  /// @param parameters Latest parameter descriptors
  void update_node_parameters(
    const std::string & node_name,
    const std::vector<rcl_interfaces::msg::ParameterDescriptor> & parameters);

  /// @brief Fill a Graph message containing cumulative graph state
  void fill_rosgraph_msg(rosgraph_monitor_msgs::msg::Graph & msg) const;

  /// @brief Update the cumulative graph with current graph information from a Graph message
  /// @param current_graph_msg The current graph state from RosGraphMonitor
  void update_from_graph_msg(const rosgraph_monitor_msgs::msg::Graph & current_graph_msg);

  // Map of node names to their cumulative information
  std::unordered_map<std::string, CumulativeNodeInfo> node_history_;

  rclcpp::Subscription<rosgraph_monitor_msgs::msg::Graph>::SharedPtr sub_graph_;
  rclcpp::Publisher<rosgraph_monitor_msgs::msg::Graph>::SharedPtr pub_cumulative_graph_;
  rclcpp::TimerBase::SharedPtr timer_publish_report_;
};

}  // namespace rosgraph_monitor

#endif  // ROSGRAPH_MONITOR__CUMULATIVE_GRAPH_NODE_HPP_
