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

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <behaviortree_cpp/action_node.h>

using namespace std::placeholders;

class CancelNav : public BT::SyncActionNode {
public:
  CancelNav(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {

    node_ = rclcpp::Node::make_shared("cancel_nave");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    t.transform.translation.z = std::numeric_limits<double>::max();
  }

  static BT::PortsList providedPorts() {
    return {

        BT::OutputPort<geometry_msgs::msg::Pose>("position_robot"),

    };
  }

  virtual BT::NodeStatus tick() override {

    calcular_pos_robot();
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  geometry_msgs::msg::TransformStamped t;

  void calcular_pos_robot() {
    // init transform variable
    t.transform.translation.z = std::numeric_limits<double>::max();

    rclcpp::Rate rate(20);
    while (t.transform.translation.z == std::numeric_limits<double>::max() &&
           rclcpp::ok()) {
      try {
        RCLCPP_ERROR(node_->get_logger(), "Error al obtener la transformacion");
        t = tf_buffer_->lookupTransform("map", "robot_base_link",
                                        tf2::TimePointZero);
        RCLCPP_ERROR(node_->get_logger(), "Llame a lookupTransform");
      } catch (tf2::TransformException &ex) {
      }
      rclcpp::spin_some(node_);
      rate.sleep();
    }
    RCLCPP_DEBUG(node_->get_logger(), "Se obtuvo la transformacion");

    geometry_msgs::msg::Pose position_robot;

    position_robot.position.x = t.transform.translation.x;
    position_robot.position.y = t.transform.translation.y;

    position_robot.orientation.z = t.transform.rotation.z;
    position_robot.orientation.w = t.transform.rotation.w;

    setOutput<geometry_msgs::msg::Pose>("position_robot", position_robot);
  }
};