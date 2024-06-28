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

class AligtShelf : public BT::SyncActionNode {
public:
  AligtShelf(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {

    node_ = rclcpp::Node::make_shared("node_name");

    pub_cmd_vel_ =
        node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    t.transform.translation.z = std::numeric_limits<double>::max();
    kp_angular_ = 3.0;
  }

  static BT::PortsList providedPorts() {
    return {

    };
  }

  virtual BT::NodeStatus tick() override {

    get_transform();
    return BT::NodeStatus::SUCCESS;
  }

private:
  geometry_msgs::msg::TransformStamped t;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  geometry_msgs::msg::Twist msg_cmd_vel;
  float theta_target;
  float kp_angular_;

  float saturateVel(float vel, float vel_lower_limit, float vel_upper_limit) {
    if (vel > 0) {
      return std::max(vel_lower_limit, std::min(vel, vel_upper_limit));
    } else {
      vel = std::max(vel_lower_limit, std::min(-vel, vel_upper_limit));
      return -vel;
    }
  }
  void get_transform() {
    // init transform variable
    t.transform.translation.z = std::numeric_limits<double>::max();

    float x;
    float y;
    float z;
    double roll, pitch, yaw;

    rclcpp::Rate rate(20);
    while (t.transform.translation.z == std::numeric_limits<double>::max() &&
           rclcpp::ok()) {
      try {
        RCLCPP_ERROR(node_->get_logger(), "Error al obtener la transformacion");
        t = tf_buffer_->lookupTransform("robot_base_link", "shelf_deep_frame",
                                        tf2::TimePointZero);
        RCLCPP_ERROR(node_->get_logger(), "Llame a lookupTransform");
      } catch (tf2::TransformException &ex) {
      }
      rclcpp::spin_some(node_);
      rate.sleep();
    }

    RCLCPP_DEBUG(node_->get_logger(), "Se obtuvo la transformacion");

    // hallo el error
    theta_target =
        std::atan2(t.transform.translation.y, t.transform.translation.x);
    rclcpp::Rate rate1(20);
    while (abs(theta_target) > 0.2) {
      msg_cmd_vel.angular.z = theta_target * kp_angular_;
      msg_cmd_vel.linear.x = 0.0;
      msg_cmd_vel.angular.z = saturateVel(msg_cmd_vel.angular.z, 0.1, 0.5);
      pub_cmd_vel_->publish(msg_cmd_vel);
      rate1.sleep();
    }
    // stop
    msg_cmd_vel.angular.z = 0.0;
    msg_cmd_vel.linear.x = 0.0;
    pub_cmd_vel_->publish(msg_cmd_vel);
    pub_cmd_vel_->publish(msg_cmd_vel);
  };