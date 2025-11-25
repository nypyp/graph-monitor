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

#include "rosgraph_cumulator/node.hpp"

#include <memory>

namespace rosgraph_cumulator
{

Node::Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("rosgraph_cumulative", options),
  cumulator_(std::make_unique<GraphCumulator>()),
  sub_graph_(create_subscription<rosgraph_monitor_msgs::msg::Graph>(
    "/rosgraph", rclcpp::QoS(10), std::bind(&Node::on_graph_message, this, std::placeholders::_1))),
  pub_cumulative_graph_(create_publisher<rosgraph_monitor_msgs::msg::Graph>(
    "/rosgraph_cumulative", rclcpp::QoS(1).transient_local().reliable()))
{
  RCLCPP_INFO(get_logger(), "GraphCumulator node initialized");
}

void Node::on_graph_message(const rosgraph_monitor_msgs::msg::Graph::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(get_logger(), "Received null graph message, skipping processing");
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Processing incoming graph message with %zu nodes", msg->nodes.size());

  // Process the graph message using the cumulative processor
  cumulator_->process_graph_message(*msg);

  // Publish cumulative graph
  publish_cumulative_graph();
}

void Node::publish_cumulative_graph()
{
  // Generate the cumulative graph from the processor
  auto cumulative_graph = cumulator_->generate_cumulative_graph();

  // Update the timestamp to current time
  cumulative_graph.timestamp = get_clock()->now();

  // Publish the cumulative graph message
  pub_cumulative_graph_->publish(cumulative_graph);

  RCLCPP_DEBUG(
    get_logger(), "Published cumulative graph with %zu nodes", cumulator_->get_node_count());
}

}  // namespace rosgraph_cumulator

RCLCPP_COMPONENTS_REGISTER_NODE(rosgraph_cumulator::Node)
