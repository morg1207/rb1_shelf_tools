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

class PublishTransform : public BT::SyncActionNode {
public:
  PublishTransform(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {

    node_ = rclcpp::Node::make_shared("node_name");

    // transfor broadcaster
    broadcaster_1 =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    broadcaster_2 =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    broadcaster_3 =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    broadcaster_4 =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    t.transform.translation.z = std::numeric_limits<double>::max();

    // params
    node_->declare_parameter("distance_aproach_target_shelf", 0.10);
    distance_aproach_target_shelf_ =
        node_->get_parameter("distance_aproach_target_shelf").as_double();

    node_->declare_parameter("radius_rb1", 0.25);
    radius_rb1_ = node_->get_parameter("radius_rb1").as_double();

    node_->declare_parameter("large_shelf", 0.65);
    large_shelf_ = node_->get_parameter("large_shelf").as_double();

    node_->declare_parameter("frame_head_to_shelf", "robot_odom");
    frame_head_to_shelf_ =
        node_->get_parameter("frame_head_to_shelf").as_string();

    RCLCPP_INFO(node_->get_logger(), "distance aproach target shelf [%.3f] ",
                distance_aproach_target_shelf_);
    RCLCPP_INFO(node_->get_logger(), "radius rb1 [%.3f] ", radius_rb1_);
    RCLCPP_INFO(node_->get_logger(), "large shelf [%.3f] ", large_shelf_);
    RCLCPP_INFO(node_->get_logger(), "frame_head_to_shelf_ [%s] ",
                frame_head_to_shelf_.c_str());
    get_target_goal_ = false;
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Point>("position_shelf"),
        BT::OutputPort<geometry_msgs::msg::Pose>("target_goal"),
        BT::InputPort<bool>("get_target_goal"),

    };
  }

  virtual BT::NodeStatus tick() override {
    if (!getInput<geometry_msgs::msg::Point>("position_shelf",
                                             position_shelf_)) {
      // if I can't get this, there is something wrong with your BT.
      // For this reason throw an exception instead of returning FAILURE
      throw BT::RuntimeError("missing required input [position_shelf]");
    }
    // get target goal for nav
    getInput<bool>("get_target_goal", get_target_goal_);
    publish_transfor();
    publish_target_goal_frame();
    publish_target_shelf_frame();
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_1;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_2;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_3;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_4;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  geometry_msgs::msg::TransformStamped t;

  // ports bt
  geometry_msgs::msg::Point position_shelf_;
  bool get_target_goal_;

  std::string frame_head_to_shelf_;

  double distance_aproach_target_shelf_;
  double radius_rb1_;
  double large_shelf_;

  void publish_transfor() {
    // init transform variable
    t.transform.translation.z = std::numeric_limits<double>::max();

    float ptr_data[3];
    ptr_data[0] = position_shelf_.x;
    ptr_data[1] = position_shelf_.y;
    ptr_data[2] = position_shelf_.z;

    RCLCPP_DEBUG(node_->get_logger(), "dt [%.3f]", ptr_data[0]);
    RCLCPP_DEBUG(node_->get_logger(), "theta_total [%.3f]", ptr_data[1]);
    float x;
    float y;
    float z;
    double roll, pitch, yaw;

    rclcpp::Rate rate(20);
    while (t.transform.translation.z == std::numeric_limits<double>::max() &&
           rclcpp::ok()) {
      try {
        RCLCPP_ERROR(node_->get_logger(), "Error al obtener la transformacion");
        t = tf_buffer_->lookupTransform(frame_head_to_shelf_,
                                        "robot_front_laser_base_link",
                                        tf2::TimePointZero);
        RCLCPP_ERROR(node_->get_logger(), "Llame a lookupTransform");
      } catch (tf2::TransformException &ex) {
      }
      rclcpp::spin_some(node_);
      rate.sleep();
    }
    RCLCPP_DEBUG(node_->get_logger(), "Se obtuvo la transformacion");
    try {
      x = t.transform.translation.x;
      y = t.transform.translation.y;
      z = t.transform.translation.z;
      // Obtener los valores de traslación y rotación

      tf2::Quaternion quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                 t.transform.rotation.z,
                                 t.transform.rotation.w);
      tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

      // Imprimir los valores obtenidos
      RCLCPP_DEBUG(node_->get_logger(), "Transform from frame1 to frame2:");
      RCLCPP_DEBUG(node_->get_logger(), "Translation: x=%.2f, y=%.2f, z=%.2f",
                   x, y, z);
      RCLCPP_DEBUG(node_->get_logger(),
                   "Rotation: roll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch,
                   yaw);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get transform: %s",
                   ex.what());
    }

    geometry_msgs::msg::TransformStamped transform_;
    transform_.header.frame_id =
        frame_head_to_shelf_;                 // Marco de referencia fuente
    transform_.child_frame_id = "cart_frame"; // Marco de referencia objetivo
    transform_.header.stamp = node_->get_clock()->now();
    // Definir la transformación estática (traslación y rotación)
    /*transform_.transform.translation.x = 0;
    transform_.transform.translation.y = 0;*/
    transform_.transform.translation.x =
        x + ptr_data[0] * std::sin(ptr_data[1]) * std::sin(yaw) +
        ptr_data[0] * std::cos(ptr_data[1]) * std::cos(yaw);
    transform_.transform.translation.y =
        y + ptr_data[0] * std::cos(ptr_data[1]) * std::sin(yaw) -
        ptr_data[0] * std::sin(ptr_data[1]) * std::cos(yaw);

    transform_.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(roll, pitch - M_PI, yaw - ptr_data[2] + M_PI); // Sin rotación

    tf2::Quaternion rotation_quat;
    rotation_quat.setRPY(0, M_PI, 0); // 90 degrees in radians
    tf2::Quaternion new_quat = rotation_quat * q;

    // Normalize the resulting quaternion
    new_quat.normalize();

    transform_.transform.rotation.x = q.x();
    transform_.transform.rotation.y = q.y();
    transform_.transform.rotation.z = q.z();
    transform_.transform.rotation.w = q.w();
    // Publicar la transformación estática
    broadcaster_1->sendTransform(transform_);

    RCLCPP_DEBUG(node_->get_logger(),
                 "Transform traslation  x  [%.2f] y  [%.2f] z  [%.2f] ",
                 transform_.transform.translation.x,
                 transform_.transform.translation.y,
                 transform_.transform.translation.z);
    RCLCPP_DEBUG(node_->get_logger(),
                 "Se publico correctamente el frame [cart_frame]  !!!");
  }

  void publish_target_goal_frame() {

    geometry_msgs::msg::TransformStamped transform_;
    transform_.header.frame_id = "cart_frame"; // Marco de referencia fuente
    transform_.child_frame_id =
        "target_goal_frame"; // Marco de referencia objetivo
    transform_.header.stamp = node_->get_clock()->now();

    transform_.transform.translation.x =
        -(distance_aproach_target_shelf_ + radius_rb1_);
    transform_.transform.translation.y = 0.0;
    transform_.transform.translation.z = 0.0;
    transform_.transform.rotation.x = 0;
    transform_.transform.rotation.y = 0;
    transform_.transform.rotation.z = 0;
    transform_.transform.rotation.w = 1;

    // Publicar la transformación estática
    broadcaster_2->sendTransform(transform_);

    RCLCPP_DEBUG(node_->get_logger(),
                 "Transform cart_frame to target_goal_frame traslation  x  "
                 "[%.2f] y  [%.2f] z  [%.2f] ",
                 transform_.transform.translation.x,
                 transform_.transform.translation.y,
                 transform_.transform.translation.z);
    RCLCPP_DEBUG(node_->get_logger(),
                 "Se publico correctamente el frame [target_goal_frame] !!!");

    if (get_target_goal_ == true) {

      geometry_msgs::msg::TransformStamped transform_1;
      transform_1.header.frame_id = "cart_frame"; // Marco de referencia fuente
      transform_1.child_frame_id =
          "target_goal_nav_frame"; // Marco de referencia objetivo
      transform_1.header.stamp = node_->get_clock()->now();

      transform_1.transform.translation.x =
          -(distance_aproach_target_shelf_ + radius_rb1_) - 0.3;
      transform_1.transform.translation.y = 0.0;
      transform_1.transform.translation.z = 0.0;
      transform_1.transform.rotation.x = 0;
      transform_1.transform.rotation.y = 0;
      transform_1.transform.rotation.z = 0;
      transform_1.transform.rotation.w = 1;

      // Publicar la transformación estática
      broadcaster_4->sendTransform(transform_1);

      // init transform variable
      t.transform.translation.z = std::numeric_limits<double>::max();
      rclcpp::Rate rate(20);
      while (t.transform.translation.z == std::numeric_limits<double>::max() &&
             rclcpp::ok()) {
        try {
          RCLCPP_ERROR(node_->get_logger(),
                       "Error al obtener la transformacion de [%s]  a [%s]",
                       "map", "target_goal_frame");
          t = tf_buffer_->lookupTransform("map", "target_goal_nav_frame",
                                          tf2::TimePointZero);
          RCLCPP_ERROR(node_->get_logger(), "Llame a lookupTransform");
        } catch (tf2::TransformException &ex) {
        }
        rclcpp::spin_some(node_);
        rate.sleep();
      }
      RCLCPP_INFO(node_->get_logger(),
                  "Se obtuvo la transformacion de map a target_goal_frame");

      geometry_msgs::msg::Pose target_goal;
      target_goal.position.x = t.transform.translation.x;
      target_goal.position.y = t.transform.translation.y;
      target_goal.position.z = t.transform.translation.z;
      target_goal.orientation.x = t.transform.rotation.x;
      target_goal.orientation.y = t.transform.rotation.y;
      target_goal.orientation.z = t.transform.rotation.z;
      target_goal.orientation.w = t.transform.rotation.w;

      RCLCPP_INFO(node_->get_logger(),
                  "Transform traslation map a target_goal x  [%.2f] y  [%.2f] "
                  "z  [%.2f]  w[%.2f]",
                  target_goal.position.x, target_goal.position.y,
                  target_goal.orientation.z, target_goal.orientation.w);
      setOutput<geometry_msgs::msg::Pose>("target_goal", target_goal);
    }
  }
  void publish_target_shelf_frame() {

    geometry_msgs::msg::TransformStamped transform_;
    transform_.header.frame_id = "cart_frame"; // Marco de referencia fuente
    transform_.child_frame_id =
        "target_shelf_frame"; // Marco de referencia objetivo
    transform_.header.stamp = node_->get_clock()->now();

    transform_.transform.translation.x = large_shelf_ / 2;
    transform_.transform.translation.y = 0.0;
    transform_.transform.translation.z = 0.0;
    transform_.transform.rotation.x = 0;
    transform_.transform.rotation.y = 0;
    transform_.transform.rotation.z = 0;
    transform_.transform.rotation.w = 1;

    // Publicar la transformación estática
    broadcaster_3->sendTransform(transform_);

    RCLCPP_DEBUG(node_->get_logger(),
                 "Se publico correctamente el frame [target_shelf_frame] !!!");
  }
};