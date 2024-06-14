#pragma once

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep

#include "rb1_shelf_msgs/srv/init_localization.hpp"

#include "geometry_msgs/msg/point.hpp"

#include "behaviortree_cpp/basic_types.h"
#include <behaviortree_cpp/action_node.h>

using namespace std::placeholders;

class InitLocalizationClient : public BT::SyncActionNode {
public:
  InitLocalizationClient(const std::string &name,
                         const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {

    node_ = rclcpp::Node::make_shared("init_localization_client");
    client_ = node_->create_client<rb1_shelf_msgs::srv::InitLocalization>(
        "/init_localization_server");
    ;
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Point>("position_charge_station"),
    };
  }

  virtual BT::NodeStatus tick() override {
    if (!getInput<geometry_msgs::msg::Point>("position_charge_station",
                                             position_charge_station_)) {
      // if I can't get this, there is something wrong with your BT.
      // For this reason throw an exception instead of returning FAILURE
      throw BT::RuntimeError(
          "missing required input [position_charge_station]");
    }
    // Waiting for service /save
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Interrupted while waiting for service /init_localization_server.");
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(node_->get_logger(),
                  "Waiting for service /init_localization_server to appear...");
    }

    auto request =
        std::make_shared<rb1_shelf_msgs::srv::InitLocalization::Request>();
    // Le paso la position de la estacion de carga
    request->station_position = position_charge_station_;
    auto result_future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Unable to call /init_localization_server");
      return BT::NodeStatus::FAILURE;
    } else {
      auto result = result_future.get();
      if (result->success != true) {

        RCLCPP_ERROR(node_->get_logger(), "Init localozation fallido");
        return BT::NodeStatus::FAILURE;
      } else {
        RCLCPP_DEBUG(node_->get_logger(), "Localizacion exitosa !!!!");

        return BT::NodeStatus::SUCCESS;
      }
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<rb1_shelf_msgs::srv::InitLocalization>::SharedPtr client_;
  geometry_msgs::msg::Point position_charge_station_;
};