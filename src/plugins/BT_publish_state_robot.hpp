#pragma once

#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep

#include "std_msgs/msg/detail/string__struct.hpp"
#include "std_msgs/msg/string.hpp"

#include <behaviortree_cpp/action_node.h>

using namespace std::placeholders;

class PublishStateRobot : public BT::SyncActionNode {
public:
  PublishStateRobot(const std::string &name) : BT::SyncActionNode(name, {}) {

    node_ = rclcpp::Node::make_shared("publish_state_robot");

    pub_state_robot_ = node_->create_publisher<std_msgs::msg::String>(
        "/robot_state_figure", 10);
  }

  virtual BT::NodeStatus tick() override {

    std_msgs::msg::String state;
    state.data = "only_robot";
    pub_state_robot_->publish(state);

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_robot_;
  rclcpp::Node::SharedPtr node_;
};