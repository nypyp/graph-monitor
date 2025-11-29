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

#include "rosgraph_validator/validator.hpp"

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <exception>
#include <functional>
#include <unordered_map>

#include "rosgraph_monitor_msgs/msg/graph.hpp"
#include "rosgraph_monitor_msgs/msg/node_info.hpp"
#include "rosgraph_monitor_msgs/msg/qos_profile.hpp"
#include "rosgraph_monitor_msgs/msg/topic.hpp"

namespace
{
rosgraph_monitor_msgs::msg::QosProfile parse_qos(const YAML::Node & qos_node)
{
  rosgraph_monitor_msgs::msg::QosProfile qos;
  qos.reliability = qos_node["reliability"].as<uint8_t>();
  qos.durability = qos_node["durability"].as<uint8_t>();
  qos.history = qos_node["history"].as<uint8_t>();
  qos.depth = qos_node["depth"].as<uint32_t>();
  return qos;
}
}  // namespace

namespace rosgraph_validator
{

GraphValidator::GraphValidator(const rclcpp::NodeOptions & options)
: rclcpp::Node("validator", options), updater_(this)
{
  RCLCPP_INFO(get_logger(), "Validator initialized");
  try {
    param_listener_ = std::make_shared<rosgraph_validator_parameters::ParamListener>(
      this->get_node_parameters_interface());
    params_ = param_listener_->get_params();

    load_config(params_.config_file);
    RCLCPP_INFO(get_logger(), "Loaded config from %s", params_.config_file.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize graph validator: %s", e.what());
  }

  updater_.setHardwareID("health");
  updater_.add("nodes", this, &GraphValidator::validate_nodes);
  updater_.add("topic", this, &GraphValidator::validate_topics);
  updater_.add("qoS", this, &GraphValidator::validate_qos);

  graph_sub_ = create_subscription<rosgraph_monitor_msgs::msg::Graph>(
    params_.graph_sub, rclcpp::QoS(1).transient_local(),
    std::bind(&GraphValidator::on_graph, this, std::placeholders::_1));
}

void GraphValidator::load_config(const std::string & file_path)
{
  try {
    std::stringstream ss;
    ss << "Parsing config file: " << file_path.c_str() << std::endl;
    YAML::Node config = YAML::LoadFile(file_path);
    if (!config["nodes"]) {
      RCLCPP_WARN(get_logger(), "No nodes found in config file");
      return;
    }

    for (const auto & node : config["nodes"]) {
      NodeView view;
      ss << "\t" << "Node: " << node["name"].as<std::string>() << std::endl;

      for (const auto & pub : node["publishers"]) {
        view.publishers.insert(
          {pub["name"].as<std::string>(), pub["type"].as<std::string>(), parse_qos(pub["qos"])});
        ss << "\t\t" << "Publisher: " << pub["name"].as<std::string>() << std::endl;
      }
      for (const auto & sub : node["subscribers"]) {
        view.subscribers.insert(
          {sub["name"].as<std::string>(), sub["type"].as<std::string>(), parse_qos(sub["qos"])});
        ss << "\t\t" << "Subscriber: " << sub["name"].as<std::string>() << std::endl;
      }
      config_graph_[node["name"].as<std::string>()] = view;
    }
    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load config: %s", e.what());
  }
}

void GraphValidator::on_graph(const rosgraph_monitor_msgs::msg::Graph::SharedPtr msg)
{
  live_graph_.clear();
  for (const auto & node : msg->nodes) {
    live_graph_[node.name] = normalize_node(node);
  }
  if (live_graph_.empty()) {
    RCLCPP_WARN(get_logger(), "No graph data received");
    return;
  }
  updater_.force_update();
}

void GraphValidator::validate_nodes(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int level = DiagStatus::OK;
  int missing_nodes = 0;
  int extra_nodes = 0;

  // Check for missing nodes
  for (const auto & [name, _] : config_graph_) {
    if (!live_graph_.count(name)) {
      stat.add("Missing node: ", name);
      level = std::max(level, static_cast<int>(DiagStatus::ERROR));
      missing_nodes++;
    }
  }

  // Check for extra nodes
  for (const auto & [name, _] : live_graph_) {
    if (!config_graph_.count(name)) {
      stat.add("Extra node: ", name);
      level = std::max(level, static_cast<int>(DiagStatus::WARN));
      extra_nodes++;
    }
  }

  if (level == DiagStatus::OK) {
    stat.summary(level, "Node availability OK");
  } else {
    stat.summaryf(level, "%d missing node(s), %d extra node(s) detect", missing_nodes, extra_nodes);
  }
}

void GraphValidator::validate_topics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int level = DiagStatus::OK;
  int missing_endpoints = 0;
  int extra_endpoints = 0;

  // Check for missing publishers and subscribers
  for (const auto & [name, config_node] : config_graph_) {
    auto live_it = live_graph_.find(name);
    if (live_it == live_graph_.end()) continue;

    const auto & live_node = live_it->second;

    for (const auto & pub : config_node.publishers) {
      if (!live_node.publishers.count(pub)) {
        stat.add("Node: " + name + " missing publisher: ", pub.name);
        level = std::max(level, static_cast<int>(DiagStatus::ERROR));
        missing_endpoints++;
      }
    }

    for (const auto & sub : config_node.subscribers) {
      if (!live_node.subscribers.count(sub)) {
        stat.add("Node: " + name + " missing subscriber: ", sub.name);
        level = std::max(level, static_cast<int>(DiagStatus::ERROR));
        missing_endpoints++;
      }
    }

    // Check for extra publishers and subscribers
    for (const auto & pub : live_node.publishers) {
      if (!config_node.publishers.count(pub)) {
        stat.add("Node: " + name + " extra publisher: ", pub.name);
        level = std::max(level, static_cast<int>(DiagStatus::WARN));
        extra_endpoints++;
      }
    }

    for (const auto & sub : live_node.subscribers) {
      if (!config_node.subscribers.count(sub)) {
        stat.add("Node: " + name + " extra subscriber: ", sub.name);
        level = std::max(level, static_cast<int>(DiagStatus::WARN));
        extra_endpoints++;
      }
    }
  }

  if (level == DiagStatus::OK) {
    stat.summary(level, "Topic availability OK");
  } else {
    stat.summaryf(
      level, "%d missing endpoint(s), %d extra endpoint(s) detect", missing_endpoints,
      extra_endpoints);
  }
}

void GraphValidator::validate_qos(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int level = DiagStatus::OK;
  int issue_count = 0;

  // Check for mismatched QoS profiles
  for (const auto & [name, config_node] : config_graph_) {
    auto live_it = live_graph_.find(name);
    if (live_it == live_graph_.end()) continue;
    const auto & live_node = live_it->second;

    for (const auto & pub : config_node.publishers) {
      const auto live_pub_it = live_node.publishers.find(pub);
      if (live_pub_it != live_node.publishers.end()) {
        if (*live_pub_it != pub) {
          stat.addf(
            "Mismatch: " + name,
            "Publisher %s: reliability %d, "
            "durability %d, expected %d, %d",
            pub.name.c_str(), live_pub_it->qos.reliability, live_pub_it->qos.durability,
            pub.qos.reliability, pub.qos.durability);
          level = std::max(level, static_cast<int>(DiagStatus::ERROR));
          issue_count++;
        }
      }
    }

    for (const auto & sub : config_node.subscribers) {
      auto live_sub_it = live_node.subscribers.find(sub);
      if (live_sub_it != live_node.subscribers.end()) {
        if (*live_sub_it != sub) {
          stat.addf(
            name,
            "Subscriber %s missmatch QoS: reliability %d, "
            "durability %d, expected %d, %d",
            sub.name.c_str(), live_sub_it->qos.reliability, live_sub_it->qos.durability,
            sub.qos.reliability, sub.qos.durability);
          level = std::max(level, static_cast<int>(DiagStatus::ERROR));
          issue_count++;
        }
      }
    }
  }

  if (level == DiagStatus::OK) {
    stat.summary(level, "QoS availability OK");
  } else {
    stat.summaryf(level, "%d QoS issue(s) detect", issue_count);
  }
}

NodeView GraphValidator::normalize_node(const rosgraph_monitor_msgs::msg::NodeInfo & node)
{
  NodeView view;
  for (const auto & pub : node.publishers) view.publishers.insert({pub.name, pub.type, pub.qos});
  for (const auto & sub : node.subscriptions)
    view.subscribers.insert({sub.name, sub.type, sub.qos});
  return view;
}

}  // namespace rosgraph_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rosgraph_validator::GraphValidator)