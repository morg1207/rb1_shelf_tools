#pragma once

#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep

#include "rb1_shelf_msgs/srv/find_nav_poses.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "behaviortree_cpp/basic_types.h"
#include <behaviortree_cpp/action_node.h>

using namespace std::placeholders;

class FindNavPointsClient : public BT::SyncActionNode {
public:
  FindNavPointsClient(const std::string &name,
                      const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {

    node_ = rclcpp::Node::make_shared("find_point_for_nav");
    client_ = node_->create_client<rb1_shelf_msgs::srv::FindNavPoses>(
        "/find_nav_poses");
    ;
  }

  static BT::PortsList providedPorts() {
    return {
        BT::OutputPort<geometry_msgs::msg::Pose>("pose_target_nav_deep"),
        BT::InputPort<geometry_msgs::msg::Point>("position_deep_shelf"),
        BT::InputPort<float>("direction"),

    };
  }

  virtual BT::NodeStatus tick() override {
    if (!getInput<geometry_msgs::msg::Point>("position_deep_shelf",
                                             position_deep_shelf_)) {
      // if I can't get this, there is something wrong with your BT.
      // For this reason throw an exception instead of returning FAILURE
      throw BT::RuntimeError("missing required input [position_deep_shelf]");
    }
    if (!getInput<float>("direction", direction_)) {
      // if I can't get this, there is something wrong with your BT.
      // For this reason throw an exception instead of returning FAILURE
      throw BT::RuntimeError("missing required input [direction]");
    }
    // Waiting for service /save
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Interrupted while waiting for service /find_nav_poses.");
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(node_->get_logger(),
                  "Waiting for service /find_nav_poses to appear...");
    }

    auto request =
        std::make_shared<rb1_shelf_msgs::srv::FindNavPoses::Request>();

    request->shelf_position = position_deep_shelf_;
    request->direction = direction_;

    auto result_future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Unable to call /find_nav_poses");
      return BT::NodeStatus::FAILURE;
    } else {

      auto result = result_future.get();
      if (result->success != true) {

        RCLCPP_ERROR(
            node_->get_logger(),
            "No se encontro puntos de navegacion er la direccion [%3f]",
            direction_);
        return BT::NodeStatus::FAILURE;
      } else {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "Se encontro estos puntos de navegacion en la direccion [%.3]",
            direction_);
        geometry_msgs::msg::Point point_nav = result->nav_position;
        geometry_msgs::msg::Pose pose_target_nav;

        pose_target_nav.position.x = point_nav.x;
        pose_target_nav.position.y = point_nav.y;
        pose_target_nav.position.z = 0.0;
        pose_target_nav.orientation.x = 0;
        pose_target_nav.orientation.y = 0;
        pose_target_nav.orientation.z = 0;
        pose_target_nav.orientation.w = 1;

        setOutput<geometry_msgs::msg::Pose>("pose_target_nav_deep",
                                            pose_target_nav);
        // setOutput("position_shelf_found",pose_shelf);

        return BT::NodeStatus::SUCCESS;
      }
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<rb1_shelf_msgs::srv::FindNavPoses>::SharedPtr client_;
  geometry_msgs::msg::Point position_deep_shelf_;
  float direction_;
};