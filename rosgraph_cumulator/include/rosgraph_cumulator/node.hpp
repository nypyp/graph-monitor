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

#ifndef ROSGRAPH_CUMULATOR__NODE_HPP_
#define ROSGRAPH_CUMULATOR__NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rosgraph_cumulator/cumulator.hpp"
#include "rosgraph_monitor_msgs/msg/graph.hpp"

namespace rosgraph_cumulator
{

/**
 * @brief ROS 2 node that processes incoming graph messages and maintains cumulative graph information
 * 
 * This node subscribes to the graph messages and maintains a cumulative view
 * of the ROS graph, preserving information about all discovered nodes and 
 * their communication endpoints over time.
 */
class Node : public rclcpp::Node
{
public:
  explicit Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Callback for incoming graph messages
   * @param msg The incoming graph message to process
   */
  void on_graph_message(const rosgraph_monitor_msgs::msg::Graph::SharedPtr msg);

  /**
   * @brief Publish the current cumulative graph
   */
  void publish_cumulative_graph();

  // Graph cumulator instance for processing
  std::unique_ptr<GraphCumulator> cumulator_;

  // Subscriptions and publishers
  rclcpp::Subscription<rosgraph_monitor_msgs::msg::Graph>::SharedPtr sub_graph_;
  rclcpp::Publisher<rosgraph_monitor_msgs::msg::Graph>::SharedPtr pub_cumulative_graph_;
};

}  // namespace rosgraph_cumulator

#endif  // ROSGRAPH_CUMULATOR__NODE_HPP_
