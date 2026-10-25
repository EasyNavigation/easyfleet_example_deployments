// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the project EasyFleet
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

// Mission controller of the Warehouse A fleet: discovers what each robot can
// do (EasyFleet capabilities), turns it into the PlanSys2 problem
// (pddl/warehouse.pddl) and reaches each goal of the mission in turn by
// planning and running the plan. The actions' behavior trees call the
// robots' capabilities (easyfleet_bt_nodes).

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "easyfleet_mission_manager/fleet_session.hpp"
#include "easyfleet_mission_manager/robot_handle.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

using namespace std::chrono_literals;

namespace
{

struct Point
{
  double x{0.0};
  double y{0.0};
};

/// Everything the controller is configured with.
struct Config
{
  std::vector<std::string> robots;
  std::vector<std::string> homes;
  std::map<std::string, Point> locations;
  std::set<std::string> docks;
  std::vector<std::pair<std::string, std::string>> packages;  // package, location
  std::vector<std::string> goals;
  double discovery_timeout{60.0};
};

/// "a:b" -> {a, b}; empty strings if there is no ':'.
std::pair<std::string, std::string> split_pair(const std::string & item)
{
  const auto colon = item.find(':');
  if (colon == std::string::npos) {
    return {"", ""};
  }
  return {item.substr(0, colon), item.substr(colon + 1)};
}

bool read_config(rclcpp::Node & node, Config & config)
{
  config.robots = node.declare_parameter("robots", std::vector<std::string>());
  config.homes = node.declare_parameter("homes", std::vector<std::string>());
  const auto docks = node.declare_parameter("docks", std::vector<std::string>());
  config.docks = std::set<std::string>(docks.begin(), docks.end());
  config.goals = node.declare_parameter("goals", std::vector<std::string>());
  config.discovery_timeout = node.declare_parameter("discovery_timeout", 60.0);
  for (const auto & item : node.declare_parameter("packages", std::vector<std::string>())) {
    const auto [package, location] = split_pair(item);
    if (package.empty() || location.empty()) {
      RCLCPP_ERROR(node.get_logger(), "packages: '%s' is not <package>:<location>", item.c_str());
      return false;
    }
    config.packages.emplace_back(package, location);
  }
  // Waypoints of the navigation capability (config/navigation_params.yaml).
  const auto ids = node.declare_parameter("navigation.waypoint_ids", std::vector<std::string>());
  for (const auto & id : ids) {
    config.locations[id] = Point{
      node.declare_parameter("navigation.waypoints." + id + ".x", 0.0),
      node.declare_parameter("navigation.waypoints." + id + ".y", 0.0)};
  }

  bool ok = !config.robots.empty() && config.robots.size() == config.homes.size() &&
    !config.goals.empty();
  if (!ok) {
    RCLCPP_ERROR(node.get_logger(), "needs 'robots', as many 'homes' and some 'goals'");
  }
  auto known = [&](const std::string & what, const std::string & location) {
      if (config.locations.count(location) == 0) {
        RCLCPP_ERROR(
          node.get_logger(), "%s '%s' is not a waypoint", what.c_str(), location.c_str());
        ok = false;
      }
    };
  for (const auto & home : config.homes) {
    known("home", home);
  }
  for (const auto & dock : config.docks) {
    known("dock", dock);
  }
  for (const auto & [package, location] : config.packages) {
    known("location of " + package, location);
  }
  return ok;
}

/// Waits until every robot announces navigation plus perception or
/// manipulation; the robots that never do are left out of the problem.
std::map<std::string, std::set<std::string>> discover(
  easyfleet::FleetSession & session, const Config & config)
{
  std::map<std::string, std::set<std::string>> found;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config.discovery_timeout));
  std::cout << "Waiting for the capabilities of " << config.robots.size() << " robots..." <<
    std::endl;
  while (rclcpp::ok()) {
    found.clear();
    bool all = true;
    for (const auto & robot : config.robots) {
      for (const auto & info : session.capabilities_of(robot)) {
        if (info.active) {
          found[robot].insert(info.capability);
        }
      }
      const auto & caps = found[robot];
      all = all && caps.count("navigation") &&
        (caps.count("perception") || caps.count("manipulation"));
    }
    if (all || std::chrono::steady_clock::now() > deadline) {
      break;
    }
    session.spin_for(1s);
  }
  for (const auto & robot : config.robots) {
    std::cout << "  " << robot << ":";
    for (const auto & capability : found[robot]) {
      std::cout << " " << capability;
    }
    std::cout << (found[robot].count("navigation") ? "" : "  (no navigation: left out)") <<
      std::endl;
  }
  return found;
}

/// Distance between waypoints along the warehouse's grid of aisles and corridors.
double route_distance(const Point & a, const Point & b)
{
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

bool build_problem(
  plansys2::ProblemExpertClient & problem, const Config & config,
  const std::map<std::string, std::set<std::string>> & capabilities)
{
  problem.clearKnowledge();
  bool ok = true;
  auto add = [&](const std::string & predicate) {
      ok = problem.addPredicate(plansys2::Predicate(predicate)) && ok;
    };

  for (const auto & [location, point] : config.locations) {
    (void)point;
    ok = problem.addInstance(plansys2::Instance{location, "location"}) && ok;
  }
  std::set<std::string> occupied;
  for (std::size_t i = 0; i < config.robots.size(); ++i) {
    const auto & robot = config.robots[i];
    const auto it = capabilities.find(robot);
    if (it == capabilities.end() || it->second.count("navigation") == 0) {
      continue;
    }
    ok = problem.addInstance(plansys2::Instance{robot, "robot"}) && ok;
    add("(robot_at " + robot + " " + config.homes[i] + ")");
    add("(idle " + robot + ")");
    occupied.insert(config.homes[i]);
    if (it->second.count("perception")) {
      add("(can_perceive " + robot + ")");
    }
    if (it->second.count("manipulation")) {
      add("(can_manipulate " + robot + ")");
      add("(hand_empty " + robot + ")");
    }
  }
  for (const auto & [location, point] : config.locations) {
    (void)point;
    if (occupied.count(location) == 0) {
      add("(free " + location + ")");
    }
  }
  for (const auto & dock : config.docks) {
    add("(is_dock " + dock + ")");
  }
  for (const auto & [package, location] : config.packages) {
    ok = problem.addInstance(plansys2::Instance{package, "package"}) && ok;
    add("(package_at " + package + " " + location + ")");
  }
  for (const auto & [from, a] : config.locations) {
    for (const auto & [to, b] : config.locations) {
      if (from != to) {
        ok = problem.addFunction(
          plansys2::Function(
            "(= (distance " + from + " " + to + ") " +
            std::to_string(route_distance(a, b)) + ")")) && ok;
      }
    }
  }
  return ok;
}

/// Prints each action of the plan when it starts and ends.
void report(
  const plansys2_msgs::action::ExecutePlan::Feedback & feedback,
  std::map<std::string, int8_t> & last)
{
  using Info = plansys2_msgs::msg::ActionExecutionInfo;
  // Endings first, so an action is reported done before the one it lets start.
  for (const bool endings : {true, false}) {
    for (const auto & action : feedback.action_execution_status) {
      const bool ending = action.status == Info::SUCCEEDED || action.status == Info::FAILED;
      if (ending != endings || last[action.action_full_name] == action.status) {
        continue;
      }
      last[action.action_full_name] = action.status;
      // action_full_name ends with ":<plan time in ms>"
      const std::string name = action.action_full_name.substr(
        0, action.action_full_name.rfind(':'));
      switch (action.status) {
        case Info::EXECUTING:
          std::cout << "  > " << name << std::endl;
          break;
        case Info::SUCCEEDED:
          std::cout << "  ok " << name << std::endl;
          break;
        case Info::FAILED:
          std::cout << "  FAILED " << name << ": " << action.message_status << std::endl;
          break;
        default:
          break;
      }
    }
  }
}

/// Plans for `goal` and runs the plan; true if it succeeds.
bool reach(
  const std::string & goal, plansys2::DomainExpertClient & domain,
  plansys2::ProblemExpertClient & problem, plansys2::PlannerClient & planner,
  plansys2::ExecutorClient & executor)
{
  if (!problem.setGoal(plansys2::Goal(goal))) {
    std::cout << "Invalid goal " << goal << std::endl;
    return false;
  }
  const auto start = std::chrono::steady_clock::now();
  const auto plan = planner.getPlan(domain.getDomain(), problem.getProblem());
  if (!plan.has_value()) {
    std::cout << "No plan reaches " << goal << std::endl;
    return false;
  }
  std::cout << "Plan (" << plan->items.size() << " actions, found in " <<
    std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start).count() << " ms):" << std::endl;
  for (const auto & item : plan->items) {
    std::cout << "  " << item.time << ": " << item.action << " [" << item.duration << "]" <<
      std::endl;
  }
  if (!executor.start_plan_execution(plan.value())) {
    std::cout << "The executor did not start the plan" << std::endl;
    return false;
  }

  std::map<std::string, int8_t> last;
  rclcpp::Rate rate(2);
  while (rclcpp::ok() && executor.execute_and_check_plan()) {
    report(executor.getFeedBack(), last);
    rate.sleep();
  }
  report(executor.getFeedBack(), last);
  const auto result = executor.getResult();
  const bool succeeded = result.has_value() &&
    result->result == plansys2_msgs::action::ExecutePlan::Result::SUCCESS;
  std::cout << (succeeded ? "Reached " : "Did not reach ") << goal << " in " <<
    std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::steady_clock::now() - start).count() << " s" << std::endl;
  return succeeded;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("warehouse_controller");

  Config config;
  if (!read_config(*node, config)) {
    rclcpp::shutdown();
    return 1;
  }

  easyfleet::FleetSession session;
  std::vector<std::unique_ptr<easyfleet::RobotHandle>> robots;
  for (const auto & name : config.robots) {
    robots.push_back(std::make_unique<easyfleet::RobotHandle>(name));
    session.add_robot(*robots.back());
  }
  session.discover_capabilities();
  const auto capabilities = discover(session, config);

  plansys2::DomainExpertClient domain(*node);
  plansys2::ProblemExpertClient problem(*node);
  plansys2::PlannerClient planner(*node);
  plansys2::ExecutorClient executor(*node);

  if (!build_problem(problem, config, capabilities)) {
    std::cout << "Could not build the problem" << std::endl;
    session.shutdown();
    return 1;
  }

  int reached = 0;
  try {
    for (std::size_t i = 0; i < config.goals.size() && rclcpp::ok(); ++i) {
      std::cout << std::endl << "== Goal " << i + 1 << "/" << config.goals.size() << ": " <<
        config.goals[i] << std::endl;
      if (!reach(config.goals[i], domain, problem, planner, executor)) {
        break;
      }
      ++reached;
    }
  } catch (const rclcpp::exceptions::RCLError & e) {
    // Shutting down: the middleware went away under a pending request.
    std::cout << "Stopped: " << e.what() << std::endl;
  }
  std::cout << std::endl << "Mission: " << reached << " of " << config.goals.size() <<
    " goals reached" << std::endl;

  robots.clear();
  session.shutdown();
  return reached == static_cast<int>(config.goals.size()) ? 0 : 1;
}
