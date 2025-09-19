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

#include "rosgraph_monitor/monitor.hpp"

#include <cstdio>
#include <functional>
#include <string>
#include <utility>
#include <vector>
#include <memory>

#include "rclcpp/logging.hpp"


std::size_t std::hash<RosRmwGid>::operator()(
  const RosRmwGid & id) const noexcept
{
  constexpr std::size_t u64s = sizeof(uint64_t);
  static_assert(sizeof(RosRmwGid) == (3 * u64s) || sizeof(RosRmwGid) == (2 * u64s));
  if (sizeof(RosRmwGid) == (2 * u64s)) {
    uint64_t d0, d1;
    std::memcpy(&d0, id.data() + (0 * u64s), u64s);
    std::memcpy(&d1, id.data() + (1 * u64s), u64s);
    return d0 + d1;
  } else if (sizeof(RosRmwGid) == (3 * u64s)) {
    uint64_t d0, d1, d2;
    std::memcpy(&d0, id.data() + (0 * u64s), u64s);
    std::memcpy(&d1, id.data() + (1 * u64s), u64s);
    std::memcpy(&d2, id.data() + (2 * u64s), u64s);
    return d0 + d1 + d2;
  }
  return 0;
}

std::size_t std::hash<std::pair<std::string, std::string>>::operator()(
  const std::pair<std::string, std::string> & value) const noexcept
{
  std::size_t h1 = std::hash<std::string>{}(value.first);
  std::size_t h2 = std::hash<std::string>{}(value.second);
  // Cribbed from boost::hash_combine
  return h1 ^ (h2 << 1);
}

namespace
{

bool match_any_prefixes(const std::vector<std::string> & prefixes, const std::string & value)
{
  for (const std::string & prefix : prefixes) {
    if (value.compare(0, prefix.size(), prefix) == 0) {
      return true;
    }
  }
  return false;
}

bool match_any_suffix(const std::vector<std::string> & suffixs, const std::string & value)
{
  for (auto & suffix : suffixs) {
    if (value.size() > suffix.size() &&
        value.compare(
          value.size() - suffix.size(),
          value.size(),
          suffix) == 0) {
        return true;
    }
  }
  return false;
}

}  // namespace


namespace rosgraph_monitor
{

std::string gid_to_str(const uint8_t gid[RMW_GID_STORAGE_SIZE])
{
  std::string result;
  result.resize(24 * 2 + 23);
  snprintf(&result[0], 3, "%02x", gid[0]);  // NOLINT(runtime/printf)
  size_t pos = 2;
  for (size_t i = 1; i < 24; i++) {
    snprintf(&result[pos], 4, ".%02x", gid[i]);  // NOLINT(runtime/printf)
    pos += 3;
  }
  return result;
}

std::string gid_to_str(const RosRmwGid & gid)
{
  return gid_to_str(&gid[0]);
}

void convert_maybe_inifite_durations(
  const rclcpp::Duration & duration,
  builtin_interfaces::msg::Duration & msg_duration)
{
  auto rmw_time = duration.to_rmw_time();
  if (rmw_time_equal(rmw_time, RMW_DURATION_INFINITE) ||
    rmw_time_equal(rmw_time, RMW_DURATION_UNSPECIFIED))
  {
    msg_duration.sec = 0;
    msg_duration.nanosec = 0;
  } else {
    msg_duration.sec = rmw_time.sec;
    msg_duration.nanosec = rmw_time.nsec;
  }
}

rosgraph_monitor_msgs::msg::QosProfile to_msg(
  const rclcpp::QoS & qos_profile)
{
  rosgraph_monitor_msgs::msg::QosProfile qos_msg;

  qos_msg.history = static_cast<uint8_t>(qos_profile.history());
  qos_msg.reliability = static_cast<uint8_t>(qos_profile.reliability());
  qos_msg.durability = static_cast<uint8_t>(qos_profile.durability());
  qos_msg.liveliness = static_cast<uint8_t>(qos_profile.liveliness());

  qos_msg.depth = qos_profile.depth();
  // Convert Duration fields - handle infinite durations
  convert_maybe_inifite_durations(qos_profile.deadline(), qos_msg.deadline);
  convert_maybe_inifite_durations(qos_profile.lifespan(), qos_msg.lifespan);
  convert_maybe_inifite_durations(
    qos_profile.liveliness_lease_duration(), qos_msg.liveliness_lease_duration);

  return qos_msg;
}

rcl_interfaces::msg::ParameterDescriptor RosGraphMonitor::ParameterTracking::to_msg() const
{
  rcl_interfaces::msg::ParameterDescriptor param_msg;
  param_msg.name = name;
  // TODO(troy): Actual type info will be populated in future PR
  param_msg.type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
  return param_msg;
}


RosGraphMonitor::NodeTracking::NodeTracking(const std::string & name)
: name(name) {}

RosGraphMonitor::EndpointTracking::EndpointTracking(
  const std::string & topic_name,
  const rclcpp::TopicEndpointInfo & info,
  const rclcpp::Time & now)
: topic_name(topic_name),
  node_name(info.node_namespace() == "/" ?
    info.node_namespace() + info.node_name() :
    info.node_namespace() + "/" + info.node_name()),
  info(info),
  last_stats_timestamp(now)
{
}

rosgraph_monitor_msgs::msg::Topic RosGraphMonitor::EndpointTracking::to_msg()
{
  rosgraph_monitor_msgs::msg::Topic topic_msg;
  topic_msg.name = topic_name;
  topic_msg.type = info.topic_type();
  topic_msg.qos = rosgraph_monitor::to_msg(info.qos_profile());
  return topic_msg;
}

RosGraphMonitor::ServiceTracking::ServiceTracking(
  std::string name,
  std::string type,
  std::string nodename
)
: service_name(std::move(name)),
  service_type(std::move(type)),
  node_name(std::move(nodename)) {}

RosGraphMonitor::ActionTracking::ActionTracking(
  std::string name,
  std::string nodename
)
: action_name(std::move(name)),
  node_name(std::move(nodename)) {}

RosGraphMonitor::RosGraphMonitor(
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  std::function<rclcpp::Time()> now_fn,
  rclcpp::Logger logger,
  QueryParams query_params,
  GraphMonitorConfiguration config
)
: config_(config),
  now_fn_(now_fn),
  node_graph_(node_graph),
  logger_(logger),
  graph_change_event_(node_graph->get_graph_event()),
  query_params_(query_params)
{
  update_graph();
  watch_thread_ = std::thread(std::bind(&RosGraphMonitor::watch_for_updates, this));
}

RosGraphMonitor::~RosGraphMonitor()
{
  shutdown_.store(true);
  // Set the predicate and notify the condition_variable to wake up the watch thread immediately
  graph_change_event_->set();
  node_graph_->notify_shutdown();
  update_event_.set();

  params_futures.clear();

  watch_thread_.join();
}

void RosGraphMonitor::update_graph()
{
  auto node_names = node_graph_->get_node_names();
  track_node_updates(node_names);

  const auto topics_and_types = node_graph_->get_topic_names_and_types();
  track_endpoint_updates(topics_and_types);

  auto node_names_and_namespaces = node_graph_->get_node_names_and_namespaces();
  track_service_update(node_names_and_namespaces);
  track_client_update(node_names_and_namespaces);
}

bool RosGraphMonitor::ignore_node(const std::string & node_name)
{
  if (node_name == "_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_") {
    return true;
  }
  if (match_any_prefixes(config_.nodes.ignore_prefixes, node_name)) {
    auto [it, inserted] = ignored_nodes_.insert(node_name);
    if (inserted) {
      RCLCPP_DEBUG(logger_, "Ignoring new node: %s", node_name.c_str());
    }
    return true;
  }
  return false;
}

bool RosGraphMonitor::ignore_service(const std::string & service_name)
{
  if (match_any_suffix(config_.services.ignore_suffixes, service_name)) {
    auto [it, inserted] = ignored_services_.emplace(service_name);
    if (inserted) {
      RCLCPP_DEBUG(logger_, "Ignoring service: %s", service_name.c_str());
    }
    return true;
  }
  return false;
}

bool RosGraphMonitor::ignore_client(const std::string & service_name)
{
  if (match_any_suffix(config_.services.ignore_suffixes, service_name)) {
    auto [it, inserted] = ignored_clients_.emplace(service_name);
    if (inserted) {
      RCLCPP_DEBUG(logger_, "Ignoring client: %s", service_name.c_str());
    }
    return true;
  }
  return false;
}

void RosGraphMonitor::track_node_updates(
  const std::vector<std::string> & observed_node_names)
{
  // Mark all stale as base state
  for (auto & [node_name, tracking] : nodes_) {
    tracking.stale = true;
  }
  // Look at current node list, detect new and returned
  for (const auto & node_name : observed_node_names) {
    if (ignore_node(node_name)) {
      continue;
    }

    NodeTracking tracking{node_name};
    auto [it, inserted] = nodes_.emplace(
      node_name, tracking);

    if (inserted) {
      RCLCPP_DEBUG(logger_, "New node: %s", node_name.c_str());
      query_node_parameters(node_name);
    } else {
      NodeTracking & tracking = it->second;
      tracking.stale = false;
      if (tracking.missing) {
        RCLCPP_INFO(logger_, "Node %s came back", node_name.c_str());
        tracking.missing = false;
        returned_nodes_.insert(node_name);
        query_node_parameters(node_name);
      }
    }
  }
  // Check which nodes are still stale - they weren't observed
  for (auto & [node_name, tracking] : nodes_) {
    if (tracking.stale && !tracking.missing) {
      RCLCPP_WARN(logger_, "Node %s went missing", node_name.c_str());
      tracking.missing = true;
      returned_nodes_.erase(node_name);
    }
  }

  // Call graph change callback if set
  if (graph_change_callback_) {
    graph_change_callback_();
  }
}

std::optional<RosGraphMonitor::EndpointTrackingMap::iterator> RosGraphMonitor::add_publisher(
  const std::string & topic_name, const rclcpp::TopicEndpointInfo & info)
{
  EndpointTracking proposed_tracking(topic_name, info, now_fn_());
  if (ignore_node(proposed_tracking.node_name)) {
    return std::nullopt;
  }
  auto [it, inserted] = publishers_.emplace(info.endpoint_gid(), proposed_tracking);
  auto & [gid, tracking] = *it;
  publisher_lookup_.insert_or_assign(std::make_pair(tracking.node_name, tracking.topic_name), gid);
  if (inserted) {
    RCLCPP_DEBUG(
      logger_, "New Publisher: %s::%s (%s)",
      tracking.node_name.c_str(), tracking.topic_name.c_str(),
      gid_to_str(tracking.info.endpoint_gid()).c_str());
  }
  return it;
}

std::optional<RosRmwGid> RosGraphMonitor::lookup_publisher(
  const std::string & node_name, const std::string & topic_name) const
{
  try {
    return publisher_lookup_.at(std::make_pair(node_name, topic_name));
  } catch (const std::out_of_range &) {
    return std::nullopt;
  }
}

std::optional<RosGraphMonitor::EndpointTrackingMap::iterator> RosGraphMonitor::add_subscription(
  const std::string & topic_name, const rclcpp::TopicEndpointInfo & info)
{
  EndpointTracking proposed_tracking(topic_name, info, now_fn_());
  if (ignore_node(proposed_tracking.node_name)) {
    return std::nullopt;
  }
  auto [it, inserted] = subscriptions_.emplace(info.endpoint_gid(), proposed_tracking);
  auto & [gid, tracking] = *it;
  subscription_lookup_.insert_or_assign(
    std::make_pair(tracking.node_name, tracking.topic_name),
    gid);
  if (inserted) {
    RCLCPP_DEBUG(
      logger_, "New Subscription: %s::%s (%s)",
      tracking.node_name.c_str(), tracking.topic_name.c_str(),
      gid_to_str(tracking.info.endpoint_gid()).c_str());
  }
  return it;
}

std::optional<RosRmwGid> RosGraphMonitor::lookup_subscription(
  const std::string & node_name, const std::string & topic_name) const
{
  try {
    return subscription_lookup_.at(std::make_pair(node_name, topic_name));
  } catch (const std::out_of_range &) {
    return std::nullopt;
  }
}

bool RosGraphMonitor::topic_period_ok(
  const rosgraph_monitor_msgs::msg::TopicStatistic & stat, const rclcpp::Duration & deadline) const
{
  const std::chrono::nanoseconds measured_period =
    rclcpp::Duration(stat.mean).to_chrono<std::chrono::nanoseconds>();
  const std::chrono::nanoseconds chrono_deadline = deadline.to_chrono<std::chrono::nanoseconds>();
  if (measured_period < chrono_deadline) {
    return true;
  }
  const auto period_error = measured_period - chrono_deadline;
  const auto allowed_error = chrono_deadline * config_.topic_statistics.deadline_allowed_error;
  return period_error <= allowed_error;
}


void RosGraphMonitor::track_endpoint_updates(const TopicsToTypes & observed_topics_and_types)
{
  // Mark all stale as base state
  for (auto & [gid, tracking] : publishers_) {
    tracking.stale = true;
  }
  for (auto & [gid, tracking] : subscriptions_) {
    tracking.stale = true;
  }
  for (auto & [topic_name, counts] : topic_endpoint_counts_) {
    counts.pubs = 0;
    counts.subs = 0;
  }

  // Look over all currently observed topics for endpoint changes
  for (const auto & [topic_name, topic_types] : observed_topics_and_types) {
    // Assumption: "multiple types on the topic" is an error already handled elsewhere
    bool count_topic =
      config_.continuity.ignore_topic_names.count(topic_name) == 0 &&
      config_.continuity.ignore_topic_types.count(topic_types[0]) == 0;
    auto & endpoint_counts = topic_endpoint_counts_[topic_name];

    // Check all publishers
    for (const auto & endpoint_info : node_graph_->get_publishers_info_by_topic(topic_name)) {
      auto maybe_it = add_publisher(topic_name, endpoint_info);
      if (!maybe_it.has_value()) {
        continue;
      }
      auto & [gid, tracking] = **maybe_it;
      if (count_topic) {
        endpoint_counts.pubs++;
      }
      tracking.stale = false;
    }

    // Check all subscriptions
    for (const auto & endpoint_info : node_graph_->get_subscriptions_info_by_topic(topic_name)) {
      auto maybe_it = add_subscription(topic_name, endpoint_info);
      if (!maybe_it.has_value()) {
        continue;
      }
      auto & [gid, tracking] = **maybe_it;

      bool count_subs_from_node =
        config_.continuity.ignore_subscriber_nodes.count(tracking.node_name) == 0;
      if (count_topic && count_subs_from_node) {
        endpoint_counts.subs++;
      }
      tracking.stale = false;
    }
  }

  // Check for any stale endpoints (not seen this iteration) - they're missing.
  // For now just delete them, there isn't a meaningful health case to track at this point
  // Also remove endpoints from missing node, that node missing is the important error.
  for (EndpointTrackingMap * endpoints : {&publishers_, &subscriptions_}) {
    for (auto it = endpoints->begin(); it != endpoints->end(); ) {
      auto & [gid, tracking] = *it;
      const auto node_it = nodes_.find(tracking.node_name);
      bool node_not_tracked = node_it == nodes_.end();
      bool node_missing = node_not_tracked ? true : node_it->second.missing;
      if (node_missing || tracking.stale) {
        it = endpoints->erase(it);
      } else {
        it++;
      }
    }
  }

  // Super basic graph continuity test - does not yet account for QoS mismatch
  if (config_.continuity.enable) {
    for (auto it = topic_endpoint_counts_.begin(); it != topic_endpoint_counts_.end(); ) {
      auto & [topic_name, counts] = *it;
      // Check counts to see if any pubs or subs don't have matches
      if (counts.pubs > 0 && counts.subs == 0) {
        pubs_with_no_subs_.insert(topic_name);
      }
      if (counts.subs > 0 && counts.pubs == 0) {
        subs_with_no_pubs_.insert(topic_name);
      }
      // Delete any lingering tracking with no matches at all, the topic doesn't exist anymore
      if (counts.pubs == 0 && counts.subs == 0) {
        pubs_with_no_subs_.erase(topic_name);
        subs_with_no_pubs_.erase(topic_name);
        it = topic_endpoint_counts_.erase(it);
      } else {
        it++;
      }
    }
  }
}

bool RosGraphMonitor::track_action_update(const std::string & service_name, const std::string & node_name) {
  size_t action_pos = service_name.find("/_action");
  if (action_pos != std::string::npos) {
    ActionTracking tracking{
      service_name.substr(0, action_pos),
      node_name
    };
    tracking.stale = false;

    auto [it, inserted] = action_servers_.emplace(tracking.action_name, tracking);
    if (inserted) {
      RCLCPP_DEBUG(logger_, "New action: %s", tracking.action_name.c_str());
    } else {
      auto & tracking = it->second;
      tracking.stale = false;
      if (tracking.missing == true) {
        RCLCPP_INFO(logger_, "Action %s is back", tracking.action_name.c_str());
        tracking.missing = false;
      }
    }
    return true;
  }
  return false;
}

bool RosGraphMonitor::track_action_client_update(const std::string & service_name, const std::string & node_name) {
  size_t action_pos = service_name.find("/_action");
  if (action_pos != std::string::npos) {
    ActionTracking tracking{
      service_name.substr(0, action_pos),
      node_name
    };
    tracking.stale = false;

    auto [it, inserted] = action_clients_.emplace(tracking.action_name, tracking);
    if (inserted) {
      RCLCPP_DEBUG(logger_, "New action client: %s", tracking.action_name.c_str());
    } else {
      auto & tracking = it->second;
      tracking.stale = false;
      if (tracking.missing == true) {
        RCLCPP_INFO(logger_, "Action client %s is back", tracking.action_name.c_str());
        tracking.missing = false;
      }
    }
    return true;
  }
  return false;
}

void RosGraphMonitor::track_service_update(
  std::vector<std::pair<std::string, std::string>> node_names_and_namespaces
)
{
  // Mark all stale as base state
  for (auto & [service, service_tracking] : services_) {
    service_tracking.stale = true;
  }
  // Mark all action stale as base state
  for (auto & [action, action_tracking] : action_servers_) {
    action_tracking.stale = true;
  }

  // Look in the current services of node
  for (auto & [node_name, name_space] : node_names_and_namespaces) {
    auto services_and_types = node_graph_->get_service_names_and_types_by_node(node_name, name_space);
    // Track services
    for (auto & [service, types] : services_and_types) {
      auto node_name_with_namespace = name_space == "/" ?
        name_space + node_name :
        name_space + "/" + node_name;

      // Ignore services
      if (ignore_service(service)) {
        continue;
      }
      // Track actions from services
      if (track_action_update(service, node_name_with_namespace)) {
        continue;
      }

      ServiceTracking tracking{service, types[0], node_name_with_namespace};
      tracking.stale = false;
      auto [it, inserted] = services_.emplace(
        service, tracking);

      if (inserted) {
          RCLCPP_DEBUG(logger_, "New service: %s", service.c_str());
      } else {
        auto & tracking = it->second;
        tracking.stale = false;
        if (tracking.missing == true) {
            RCLCPP_INFO(logger_, "Service came back: %s", service.c_str());
            tracking.missing = false;
        }
        if (tracking.node_name != node_name_with_namespace) {
          RCLCPP_WARN(
            logger_,
            "Service [%s] advised to node [%s] now",
            service.c_str(),
            node_name_with_namespace.c_str());
        }
      }
    }
  }
  // Check which service are still stale - they weren't observed
  for (auto & [service, tracking] : services_) {
    if (tracking.stale) {
      RCLCPP_WARN(logger_, "Service is missing: %s", service.c_str());
      tracking.missing = true;
    }
  }
  // Check which action are still stale - they weren't observed
  for (auto & [action, tracking] : action_servers_) {
    if (tracking.stale) {
      RCLCPP_WARN(logger_, "Action is missing: %s", action.c_str());
      tracking.missing = true;
    }
  }
}

void RosGraphMonitor::track_client_update(
  std::vector<std::pair<std::string, std::string>> node_names_and_namespaces
)
{
  // Mark all clients stale as base state
  for (auto & [client_key, client_tracking] : clients_) {
    client_tracking.stale = true;
  }

  // Mark all action clients stale as base state
  for (auto & [action, action_tracking] : action_clients_) {
    action_tracking.stale = true;
  }

  // Look in the current clients of node
  for (auto & [node_name, name_space] : node_names_and_namespaces) {
    auto clients_and_types = node_graph_->get_client_names_and_types_by_node(node_name, name_space);
    auto node_name_with_namespace = name_space == "/" ?
      name_space + node_name :
      name_space + "/" + node_name;

    // Track clients
    for (auto & [service, types] : clients_and_types) {
      // Ignore clients
      if (ignore_client(service)) {
        continue;
      }

      // Track actions from clients
      if (track_action_client_update(service, node_name_with_namespace)) {
        continue;
      }

      // Regular service clients
      std::pair<std::string, std::string> client_key{service, node_name_with_namespace};
      auto client_it = clients_.find(client_key);
      if (client_it != clients_.end()) {
        // Existing client tracking
        auto & tracking = client_it->second;
        tracking.stale = false;
        if (tracking.missing) {
          RCLCPP_INFO(logger_, "Client %s came back for node %s",
                     service.c_str(), node_name_with_namespace.c_str());
          tracking.missing = false;
        }
      } else {
        // New client tracking
        ServiceTracking client_tracking{service, types[0], node_name_with_namespace};
        client_tracking.stale = false;
        clients_.emplace(client_key, client_tracking);
        RCLCPP_DEBUG(logger_, "New client %s on node %s",
                    service.c_str(), node_name_with_namespace.c_str());
      }
    }
  }

  // Check which clients are still stale - they weren't observed
  for (auto & [client_key, tracking] : clients_) {
    if (tracking.stale && !tracking.missing) {
      RCLCPP_WARN(logger_, "Client %s is missing on node %s",
                 client_key.first.c_str(), client_key.second.c_str());
      tracking.missing = true;
    }
  }

  // Check which action clients are still stale - they weren't observed
  for (auto & [action_client, tracking] : action_clients_) {
    if (tracking.stale && !tracking.missing) {
      RCLCPP_WARN(logger_, "Action client is missing: %s", action_client.c_str());
      tracking.missing = true;
    }
  }
}

void RosGraphMonitor::evaluate(std::vector<diagnostic_msgs::msg::DiagnosticStatus> & status)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  auto now = now_fn_();

  // Nodes
  {
    diagnostic_updater::DiagnosticStatusWrapper nodes_status;
    statusWrapper(nodes_status, DiagnosticStatus::OK, "Nodes OK", "nodes");
    size_t missing_optional_nodes = 0;
    size_t missing_required_nodes = 0;
    for (const auto & [node_name, node_info] : nodes_) {
      if (node_info.missing) {
        if (match_any_prefixes(config_.nodes.warn_only_prefixes, node_name)) {
          nodes_status.add("Optional node missing", node_name);
          missing_optional_nodes++;
        } else {
          nodes_status.add("Required node missing", node_name);
          missing_required_nodes++;
        }
      }
    }
    for (const std::string & node_name : returned_nodes_) {
      nodes_status.addf("Node came back: %s", node_name.c_str());
    }
    if (missing_required_nodes > 0) {
      nodes_status.summaryf(
        DiagnosticStatus::ERROR, "%d required node(s) missing.", missing_required_nodes);
    }
    if (missing_optional_nodes > 0) {
      nodes_status.mergeSummaryf(
        DiagnosticStatus::WARN, "%d optional node(s) missing.", missing_optional_nodes);
    }
    status.push_back(nodes_status);
  }

  // Continuity
  {
    diagnostic_updater::DiagnosticStatusWrapper continuity_status;
    statusWrapper(continuity_status, DiagnosticStatus::OK, "Graph continuity OK", "continuity");
    for (const auto & [topic_name, counts] : topic_endpoint_counts_) {
      if (counts.pubs > 0 && subs_with_no_pubs_.erase(topic_name) > 0) {
        continuity_status.add("Dead sink cleared. Topic now has publisher(s).", topic_name);
      }
      if (counts.subs > 0 && pubs_with_no_subs_.erase(topic_name) > 0) {
        continuity_status.add("Leaf topic cleared. Topic now has subscriber(s).", topic_name);
      }
    }
    size_t continuity_issues = 0;
    for (const auto & topic_name : pubs_with_no_subs_) {
      continuity_status.add("Leaf topic (No subscriptions): Topic", topic_name);
      continuity_issues++;
    }
    for (const auto & topic_name : subs_with_no_pubs_) {
      continuity_status.add("Dead sink (No publishers): Topic", topic_name);
      continuity_issues++;
    }
    if (continuity_issues > 0) {
      continuity_status.summaryf(
        DiagnosticStatus::WARN,
        "%d continuity issues detected.",
        continuity_issues);
    }
    status.push_back(continuity_status);
  }

  // Frequency
  auto deadline_not_set = [](const rclcpp::Duration & dur) {
      return rmw_time_equal(dur.to_rmw_time(), RMW_DURATION_INFINITE) ||
             rmw_time_equal(dur.to_rmw_time(), RMW_DURATION_UNSPECIFIED);
    };
  {
    diagnostic_updater::DiagnosticStatusWrapper pub_freq_status;
    statusWrapper(
      pub_freq_status, DiagnosticStatus::OK, "Publish frequencies OK", "publish_frequency");

    size_t pub_freq_errors = 0;
    size_t pub_freq_warns = 0;
    for (const auto & [gid, tracking] : publishers_) {
      auto deadline = tracking.info.qos_profile().deadline();
      const std::string & topic = tracking.topic_name;
      bool stale = (now - tracking.last_stats_timestamp) > config_.topic_statistics.stale_timeout;

      bool no_deadline = deadline_not_set(deadline);
      bool ignore_deadline = config_.topic_statistics.ignore_topics.count(topic) > 0;
      bool mandatory_deadline = config_.topic_statistics.mandatory_topics.count(topic) > 0;
      bool deadline_not_required = (no_deadline && !mandatory_deadline) || ignore_deadline;

      if (deadline_not_required) {
        // No deadline, don't care
      } else if (stale) {
        // Haven't received topic statistics recently enough, likely this means it's not running
        pub_freq_status.add("Stale topic statistics for publisher with deadline", topic);
        pub_freq_errors++;
      } else if (!tracking.period_stat.has_value()) {
        // Not stale yet, but also not yet received statistics info. Just waiting.
      } else if (!topic_period_ok(*tracking.period_stat, deadline)) {
        // Have received topic stats and it isn't in good range
        pub_freq_status.add("Publisher sending too slowly", topic);
        pub_freq_warns++;
      } else {
        pub_freq_status.add("Publisher frequency OK", topic);
      }
    }
    if (pub_freq_errors > 0) {
      pub_freq_status.summaryf(
        DiagnosticStatus::ERROR,
        "Frequency errors detected.",
        pub_freq_errors);
    }
    if (pub_freq_warns > 0) {
      pub_freq_status.summaryf(
        DiagnosticStatus::WARN,
        "Frequency warnings detected.",
        pub_freq_warns);
    }
    status.push_back(pub_freq_status);
  }
  {
    diagnostic_updater::DiagnosticStatusWrapper sub_freq_status;
    statusWrapper(
      sub_freq_status, DiagnosticStatus::OK, "Receive frequencies OK", "receive_frequency");
    size_t sub_freq_errors = 0;
    size_t sub_freq_warns = 0;
    for (const auto & [gid, tracking] : subscriptions_) {
      auto deadline = tracking.info.qos_profile().deadline();
      const std::string & topic = tracking.topic_name;
      bool stale = (now - tracking.last_stats_timestamp) > config_.topic_statistics.stale_timeout;
      if (deadline_not_set(deadline)) {
        // No deadline, don't care
      } else if (stale) {
        // Haven't received topic statistics recently enough, likely this means it's not running
        sub_freq_status.add("Stale topic statistics for subscription with deadline", topic);
        sub_freq_errors++;
      } else if (!tracking.period_stat.has_value()) {
        // Not stale yet, but also not yet received statistics info. Just waiting.
      } else if (!topic_period_ok(*tracking.period_stat, deadline)) {
        // Have received topic stats and it isn't in good range
        sub_freq_status.add("Subscription receiving too slowly", topic);
        sub_freq_warns++;
      } else {
        sub_freq_status.add("Subscription receive frequency OK", topic);
      }
    }
    if (sub_freq_errors > 0) {
      sub_freq_status.summaryf(
        DiagnosticStatus::ERROR,
        "Frequency errors detected.",
        sub_freq_errors);
    }
    if (sub_freq_warns > 0) {
      sub_freq_status.summaryf(
        DiagnosticStatus::WARN,
        "Frequency warnings detected.",
        sub_freq_warns);
    }
    status.push_back(sub_freq_status);
  }
}

void RosGraphMonitor::watch_for_updates()
{
  const auto wait_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::milliseconds(100));
  while (!shutdown_) {
    if (graph_change_event_->check_and_clear()) {
      update_graph();
      update_event_.set();
    }
    node_graph_->wait_for_graph_change(graph_change_event_, wait_ns);
  }
}

bool RosGraphMonitor::wait_for_update(std::chrono::milliseconds timeout)
{
  update_event_.wait_for(timeout);
  return update_event_.check_and_clear();
}

GraphMonitorConfiguration & RosGraphMonitor::config()
{
  return config_;
}


const GraphMonitorConfiguration & RosGraphMonitor::config() const
{
  return config_;
}

void RosGraphMonitor::on_topic_statistics(const rosgraph_monitor_msgs::msg::TopicStatistics & msg)
{
  for (const auto & stat : msg.statistics) {
    std::optional<RosRmwGid> maybe_gid;
    EndpointTrackingMap * endpoints = nullptr;
    if (stat.statistic_type == rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD) {
      maybe_gid = lookup_publisher(stat.node_name, stat.topic_name);
      endpoints = &publishers_;
    } else if (stat.statistic_type == rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD) {
      maybe_gid = lookup_subscription(stat.node_name, stat.topic_name);
      endpoints = &subscriptions_;
    } else {
      continue;
    }

    if (!maybe_gid.has_value()) {
      continue;
    }
    assert(endpoints != nullptr);
    auto it = endpoints->find(*maybe_gid);
    if (it == endpoints->end()) {
      continue;
    }
    auto & [gid, tracking] = *it;
    tracking.last_stats_timestamp = rclcpp::Time(msg.timestamp, RCL_ROS_TIME);
    tracking.period_stat = stat;
  }
}

void RosGraphMonitor::statusWrapper(
  diagnostic_updater::DiagnosticStatusWrapper & msg,
  uint8_t level,
  const std::string & message,
  const std::string & subname) const
{
  msg.summary(level, message);
  msg.name = config_.diagnostic_namespace;
  if (!subname.empty()) {
    msg.name += "/" + subname;
  }
  msg.hardware_id = "health";
}

void RosGraphMonitor::fill_rosgraph_msg(rosgraph_monitor_msgs::msg::Graph & msg)
{
  msg.timestamp = now_fn_();
  msg.nodes.clear();

  RCLCPP_DEBUG(logger_, "EVENT rosgraph message with %zu nodes", nodes_.size());

  for (const auto & [node_name, node_info] : nodes_) {
    if (ignore_node(node_name) || node_info.missing || node_info.stale) {
      continue;
    }

    rosgraph_monitor_msgs::msg::NodeInfo node_msg;
    node_msg.name = node_name;

    for (const auto & param : node_info.params) {
      node_msg.parameters.push_back(param.to_msg());
    }

    // Add publishers for this node
    for (auto & [gid, tracking] : publishers_) {
      if (tracking.node_name == node_name) {
        auto topic_msg = tracking.to_msg();
        node_msg.publishers.push_back(topic_msg);
      }
    }

    // Add subscriptions for this node
    for (auto & [gid, tracking] : subscriptions_) {
      if (tracking.node_name == node_name) {
        auto topic_msg = tracking.to_msg();
        node_msg.subscriptions.push_back(topic_msg);
      }
    }

  // Add services to nodes
  for (const auto & [service, tracking] : services_) {
    if (tracking.node_name == node_name && !tracking.missing) {
      rosgraph_monitor_msgs::msg::Service service_msg;
      service_msg.name = tracking.service_name;
      service_msg.type = tracking.service_type;
      node_msg.services.push_back(service_msg);
    }
  }

  // Add service clients to nodes
  for (const auto & [client_key, client_tracking] : clients_) {
    if (client_key.second == node_name && !client_tracking.missing) {
      rosgraph_monitor_msgs::msg::Service client_msg;
      client_msg.name = client_tracking.service_name;
      client_msg.type = client_tracking.service_type;
      node_msg.clients.push_back(client_msg);
    }
  }

  // Add action servers to nodes
  for (const auto & [action, tracking] : action_servers_) {
    if (tracking.node_name == node_name && !tracking.missing) {
      rosgraph_monitor_msgs::msg::Action action_msg;
      action_msg.name = tracking.action_name;
      node_msg.action_servers.push_back(action_msg);
    }
  }

  // Add action clients to nodes
  for (const auto & [action, tracking] : action_clients_) {
    if (tracking.node_name == node_name && !tracking.missing) {
      rosgraph_monitor_msgs::msg::Action action_msg;
      action_msg.name = tracking.action_name;
      node_msg.action_clients.push_back(action_msg);
    }
  }

    msg.nodes.push_back(node_msg);
  }
}

void RosGraphMonitor::set_graph_change_callback(
  std::function<void(rosgraph_monitor_msgs::msg::Graph &)> callback)
{
  graph_change_callback_ = [callback, this]() {
      rosgraph_monitor_msgs::msg::Graph msg;
      fill_rosgraph_msg(msg);
      callback(msg);
    };
}

void RosGraphMonitor::query_node_parameters(const std::string & node_name)
{
  // Non-blocking async call for parameter query. Hold onto the future to track completion.
  params_futures[node_name] = query_params_(
    node_name,
    [this, node_name_copy = std::string(node_name)](
      const rcl_interfaces::msg::ListParametersResult & result) {
      RCLCPP_INFO(
        logger_, "Got parameters for node %s: %zu", node_name_copy.c_str(),
        result.names.size());
      auto it = nodes_.find(node_name_copy);
      if (it == nodes_.end()) {
        RCLCPP_WARN(logger_, "Node %s not found in tracking map", node_name_copy.c_str());
        return;
      }
      auto & tracking = it->second;
      tracking.params.clear();
      tracking.params.reserve(result.names.size());
      for (const auto & param_name : result.names) {
        tracking.params.push_back(
          ParameterTracking{param_name,
            rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET});
      }
      if (!tracking.params.empty()) {
        graph_change_callback_();
      }
    });
}


}  // namespace rosgraph_monitor
