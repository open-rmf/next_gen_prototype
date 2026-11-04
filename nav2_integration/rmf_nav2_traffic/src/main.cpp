#include <chrono>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rmf_prototype_msgs/msg/participant.hpp>
#include <rmf_prototype_msgs/msg/participant_list.hpp>

#include "rmf_nav2_traffic/bt_nodes.hpp"

namespace
{

std::vector<std::string> split_csv(const std::string & input)
{
  std::vector<std::string> out;
  std::stringstream ss(input);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const auto trimmed = rmf_nav2_traffic::ProgressTracker::trim_ascii(item);
    if (!trimmed.empty()) {
      out.push_back(trimmed);
    }
  }
  return out;
}

std::vector<std::string> parse_agent_names(int argc, char ** argv)
{
  if (const char * env_agents = std::getenv("RMF_NAV2_AGENTS")) {
    const auto names = split_csv(env_agents);
    if (!names.empty()) {
      return names;
    }
  }

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--agents" && i + 1 < argc) {
      const auto names = split_csv(argv[i + 1]);
      if (!names.empty()) {
        return names;
      }
    } else if (arg.rfind("--agents=", 0) == 0) {
      const auto names = split_csv(arg.substr(9));
      if (!names.empty()) {
        return names;
      }
    }
  }

  return {"robot0", "robot1"};
}

BT::Tree build_agent_tree(const std::shared_ptr<rmf_nav2_traffic::AgentContext> & ctx)
{
  BT::BehaviorTreeFactory factory;

  factory.registerBuilder<rmf_nav2_traffic::HandleOuterGoalRequests>(
    "HandleOuterGoalRequests",
    [ctx](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<rmf_nav2_traffic::HandleOuterGoalRequests>(name, config, ctx);
    });

  factory.registerBuilder<rmf_nav2_traffic::ProcessSafeZoneAndCostmap>(
    "ProcessSafeZoneAndCostmap",
    [ctx](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<rmf_nav2_traffic::ProcessSafeZoneAndCostmap>(name, config, ctx);
    });

  factory.registerBuilder<rmf_nav2_traffic::WaitForCostmapAck>(
    "WaitForCostmapAck",
    [ctx](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<rmf_nav2_traffic::WaitForCostmapAck>(name, config, ctx);
    });

  factory.registerBuilder<rmf_nav2_traffic::ExecuteInnerNavigation>(
    "ExecuteInnerNavigation",
    [ctx](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<rmf_nav2_traffic::ExecuteInnerNavigation>(name, config, ctx);
    });

  factory.registerBuilder<rmf_nav2_traffic::MonitorOuterGoalCompletion>(
    "MonitorOuterGoalCompletion",
    [ctx](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<rmf_nav2_traffic::MonitorOuterGoalCompletion>(name, config, ctx);
    });

  return factory.createTreeFromText(rmf_nav2_traffic::kAgentBehaviorTreeXml);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto agent_names = parse_agent_names(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("nav_traffic_node");

  const auto transient_qos = rclcpp::QoS(10).transient_local().reliable();
  auto pub_discovery =
    node->create_publisher<rmf_prototype_msgs::msg::ParticipantList>(
      "/destination/discovery", transient_qos);

  rmf_prototype_msgs::msg::ParticipantList discovery_msg;
  for (const auto & name : agent_names) {
    rmf_prototype_msgs::msg::Participant p;
    p.name = name;
    discovery_msg.participants.push_back(std::move(p));
  }
  pub_discovery->publish(discovery_msg);

  std::vector<std::shared_ptr<rmf_nav2_traffic::AgentContext>> agents;
  std::vector<BT::Tree> trees;
  agents.reserve(agent_names.size());
  trees.reserve(agent_names.size());

  for (std::size_t i = 0; i < agent_names.size(); ++i) {
    auto ctx = std::make_shared<rmf_nav2_traffic::AgentContext>(i, agent_names[i], node);
    trees.push_back(build_agent_tree(ctx));
    agents.push_back(std::move(ctx));
  }

  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(20), [&trees]() {
      for (auto & tree : trees) {
        tree.tickOnce();
      }
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
