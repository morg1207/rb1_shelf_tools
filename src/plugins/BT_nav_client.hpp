#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "behaviortree_cpp/action_node.h"

class Nav2Client : public BT::ThreadedAction {
public:
  Nav2Client(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ThreadedAction(name, config), _aborted(false) {
    // Initialize node and action client once in the constructor
    node_ = rclcpp::Node::make_shared("nav2_client");
    action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "navigate_to_pose");
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<geometry_msgs::msg::Pose>("goal")};
  }

  virtual BT::NodeStatus tick() override {
    // if no server is present, fail after 20 seconds
    if (!action_client_->wait_for_action_server(std::chrono::seconds(20))) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Action /navigate_to_pose server not available after waiting");
      return BT::NodeStatus::FAILURE;
    }

    // Take the goal from the InputPort of the Node
    geometry_msgs::msg::Pose goal;
    if (!getInput<geometry_msgs::msg::Pose>("goal", goal)) {
      // If I can't get this, there is something wrong with your BT.
      // For this reason throw an exception instead of returning FAILURE
      throw BT::RuntimeError("missing required input [goal]");
    }

    _aborted = false;

    RCLCPP_INFO(node_->get_logger(), "Sending goal %f %f %f %f",
                goal.position.x, goal.position.y, goal.orientation.z,
                goal.orientation.w);

    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->get_clock()->now();
    goal_msg.pose.pose.position.x = goal.position.x;
    goal_msg.pose.pose.position.y = goal.position.y;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation.x = 0;
    goal_msg.pose.pose.orientation.y = 0;
    goal_msg.pose.pose.orientation.z = goal.orientation.z;
    goal_msg.pose.pose.orientation.w = goal.orientation.w;

    auto goal_handle_future = action_client_->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "send goal call failed");
      return BT::NodeStatus::FAILURE;
    }

    goal_handle_ = goal_handle_future.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      return BT::NodeStatus::FAILURE;
    }

    auto result_future = action_client_->async_get_result(goal_handle_);

    RCLCPP_INFO(node_->get_logger(), "Waiting for result");
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "get result call failed ");
      return BT::NodeStatus::FAILURE;
    }

    rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::WrappedResult wrapped_result =
        result_future.get();

    switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
      return BT::NodeStatus::FAILURE;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
      return BT::NodeStatus::FAILURE;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return BT::NodeStatus::FAILURE;
    }

    if (_aborted) {
      // This happens only if method halt() was invoked
    }

    RCLCPP_INFO(node_->get_logger(), "Result received");
    return BT::NodeStatus::SUCCESS;
  }

  virtual void halt() override {
    RCLCPP_INFO(node_->get_logger(), "Navigation aborted");
    _aborted = true;
  }

private:
  bool _aborted;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
      goal_handle_;
};
