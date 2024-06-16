#pragma once

#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>

#include "std_msgs/msg/string.hpp"

#include <behaviortree_cpp/action_node.h>

using namespace std::placeholders;

class CheckNavGoal : public BT::SyncActionNode {
public:
  CheckNavGoal(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {

    node_ = rclcpp::Node::make_shared("node_name");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    t.transform.translation.z = std::numeric_limits<double>::max();

    check_nav_goal_ = false;
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("target_frame_check"),
            BT::OutputPort<bool>("check_nav_goal"),
            BT::InputPort<float>("error_min_check")};
  }

  virtual BT::NodeStatus tick() override {
    if (!getInput<std::string>("target_frame_check", target_frame_check_)) {
      // if I can't get this, there is something wrong with your BT.
      // For this reason throw an exception instead of returning FAILURE
      throw BT::RuntimeError("missing required input [target_frame_check]");
    }
    if (!getInput<float>("error_min_check", error_min_check_)) {
      // if I can't get this, there is something wrong with your BT.
      // For this reason throw an exception instead of returning FAILURE
      throw BT::RuntimeError("missing required input [error_min_check]");
    }
    checkGoal();
    if (check_nav_goal_) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  geometry_msgs::msg::TransformStamped t;

  std::string target_frame_check_;
  bool check_nav_goal_;
  float error_min_check_;

  void checkGoal() {
    // init transform variable
    t.transform.translation.z = std::numeric_limits<double>::max();

    float x_current;
    float y_current;
    rclcpp::Rate rate(20);
    while (t.transform.translation.z == std::numeric_limits<double>::max() &&
           rclcpp::ok()) {
      try {
        RCLCPP_ERROR(node_->get_logger(), "Error al obtener la transformacion");
        t = tf_buffer_->lookupTransform(target_frame_check_, "robot_base_link",
                                        tf2::TimePointZero);
        RCLCPP_ERROR(node_->get_logger(), "Llame a lookupTransform");
      } catch (tf2::TransformException &ex) {
      }
      rclcpp::spin_some(node_);
      rate.sleep();
    }
    x_current = t.transform.translation.x;
    y_current = t.transform.translation.y;
    RCLCPP_DEBUG(node_->get_logger(), "Se obtuvo la transformacion");

    float error_nav =
        std::sqrt(std::pow(x_current, 2) + std::pow(y_current, 2));
    RCLCPP_INFO(node_->get_logger(), "Error de navegacion [%.3f]", error_nav);

    if (error_nav < error_min_check_) {
      check_nav_goal_ = true;
      setOutput<bool>("check_nav_goal", check_nav_goal_);
    } else {
      check_nav_goal_ = false;
      setOutput<bool>("check_nav_goal", check_nav_goal_);
    }
  }
};