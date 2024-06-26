
#pragma once

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/point_stamped__struct.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep

#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>

#include <behaviortree_cpp/action_node.h>

using namespace std::placeholders;

struct Point {
  double x;
  double y;
};

class TurnRobot : public BT::StatefulActionNode {
public:
  // Any TreeNode with ports must have a constructor with this signature
  TurnRobot(const std::string &name, const BT::NodeConfiguration &config)
      : StatefulActionNode(name, config) {
    node_ = rclcpp::Node::make_shared("bt_turn_robot_node");

    pub_cmd_vel_ =
        node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_position_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
        "position_shelf", 10);

    pub_market_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "position_shelf_market", 10);

    auto sensor_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom", sensor_qos, std::bind(&TurnRobot::odomCallback, this, _1));

    node_->declare_parameter("vel_turn", 0.4);

    vel_turn_ = node_->get_parameter("vel_turn").as_double();
    yaw_init = std::numeric_limits<double>::max();
    flag_odom = false;
    calcular_deep_shelf_ = false;
    // init variables
    position_deep_shelf_.x = 0;
    position_deep_shelf_.y = 0;

    // transfor broadcaster
    broadcaster_1 =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("angle_rotate"),
        BT::InputPort<geometry_msgs::msg::Point>("position_deep_shelf"),
        BT::OutputPort<geometry_msgs::msg::Point>("position_deep_shelf_end"),
        BT::InputPort<bool>("calcular_deep_shelf"),
        BT::InputPort<bool>("found_shelf"),
    };
  }

  // this function is invoked once at the beginning.
  BT::NodeStatus onStart() override;

  // If onStart() returned RUNNING, we will keep calling
  // this method until it return something different from RUNNING
  BT::NodeStatus onRunning() override;

  // callback to execute if the action was aborted by another node
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_position_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_market_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_1;

  geometry_msgs::msg::Twist cmd_vel_msg;
  float angle_rotate_;
  float vel_turn_;
  geometry_msgs::msg::Point position_deep_shelf_;
  std::vector<Point> position_deep_shelf_points;
  Point position_deep_shelf_point;
  bool calcular_deep_shelf_;
  bool found_shelf_;
  double yaw;
  double yaw_init;
  double flag_odom;

  int contador_error = 0;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    RCLCPP_INFO(node_->get_logger(), "Odom subscribers ");
    double roll, pitch;
    tf2::Quaternion quaternion(
        odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    flag_odom = true;
    RCLCPP_INFO(node_->get_logger(), "Current Yaw  [%.3f] ", yaw);
  }
  double calculate_angle_rotated(double initial_yaw, double current_yaw) {
    double angle_rotated = current_yaw - initial_yaw;
    RCLCPP_INFO(node_->get_logger(), "angle_rotated  [%.3f] ", angle_rotated);
    while (angle_rotated > M_PI) {
      RCLCPP_INFO(node_->get_logger(), "angle mayor pi antes [%.3f] ",
                  angle_rotated);
      angle_rotated -= 2.0 * M_PI;
      RCLCPP_INFO(node_->get_logger(), "angle mayor pi  despues[%.3f] ",
                  angle_rotated);
    }
    while (angle_rotated < -M_PI) {
      RCLCPP_INFO(node_->get_logger(), "angle menor a pi antes [%.3f] ",
                  angle_rotated);
      angle_rotated += 2.0 * M_PI;
      RCLCPP_INFO(node_->get_logger(), "angle menor a pi despues [%.3f] ",
                  angle_rotated);
    }
    if (angle_rotated < 0) {
      RCLCPP_INFO(node_->get_logger(), "angle menor a cero[%.3f] ",
                  angle_rotated);
      return 2.0 * M_PI + angle_rotated;
    } else {
      return angle_rotated;
    }
  }
  // Función para calcular la distancia euclidiana entre dos puntos
  double distance(const Point &p1, const Point &p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
  }

  // Función para calcular el promedio de un conjunto de puntos
  Point calculateAverage(const std::vector<Point> &points) {
    Point avg = {0, 0};
    for (const auto &point : points) {
      avg.x += point.x;
      avg.y += point.y;
    }
    avg.x /= points.size();
    avg.y /= points.size();
    RCLCPP_INFO(node_->get_logger(), "Average x [%.3f] y [%.3f]", avg.x, avg.y);
    return avg;
  }

  // Función para filtrar y promediar los puntos
  Point filterAndAveragePoints(const std::vector<Point> &points,
                               double threshold) {
    if (points.empty())
      return {0, 0};

    // Calcular el punto promedio inicial
    Point avg = calculateAverage(points);

    // Filtrar los puntos que están dentro del umbral de distancia del promedio
    std::vector<Point> filteredPoints;
    for (const auto &point : points) {
      if (distance(point, avg) <= threshold) {
        filteredPoints.push_back(point);
      }
    }

    // Calcular el nuevo punto promedio de los puntos filtrados
    if (filteredPoints.empty()) {
      RCLCPP_INFO(node_->get_logger(), "Average point ");
      return avg; // En caso de que todos los puntos se filtren, devolver el
                  // promedio inicial
    }
    return calculateAverage(filteredPoints);
  }
};

//-------------------------

BT::NodeStatus TurnRobot::onStart() {
  if (!getInput<float>("angle_rotate", angle_rotate_)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input [angle_rotate]");
  }
  // me fijo si se reuiere la posicion del shelf
  getInput<bool>("calcular_deep_shelf", calcular_deep_shelf_);

  RCLCPP_ERROR(node_->get_logger(), "Find shelf init ");
  rclcpp::Rate rate(20);
  while (flag_odom != true) {
    rclcpp::spin_some(node_);
    rate.sleep();
  }
  yaw_init = yaw;
  RCLCPP_INFO(node_->get_logger(), "Yaw init [%.3f] ", yaw_init);
  // We use this counter to simulate an action that takes a certain
  // amount of time to be completed (200 ms)
  cmd_vel_msg.angular.z = vel_turn_;
  cmd_vel_msg.linear.x = 0.0;
  pub_cmd_vel_->publish(cmd_vel_msg);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnRobot::onRunning() {
  if (calcular_deep_shelf_ == true) {
    // obbtengo la psoicion del shelf
    getInput<geometry_msgs::msg::Point>("position_deep_shelf",
                                        position_deep_shelf_);
    getInput<bool>("found_shelf", found_shelf_);
    if (found_shelf_ == true) {
      position_deep_shelf_point.x = position_deep_shelf_.x;
      position_deep_shelf_point.y = position_deep_shelf_.y;
      position_deep_shelf_points.push_back(position_deep_shelf_point);
      RCLCPP_INFO(node_->get_logger(), "Position shelf  x  [%.3f] y  [%.3f]",
                  position_deep_shelf_.x - position_deep_shelf_.y);
    }
  }

  // Pretend that we are checking if the reply has been received
  // you don't want to block inside this function too much time.
  rclcpp::spin_some(node_);
  cmd_vel_msg.angular.z = vel_turn_;
  cmd_vel_msg.linear.x = 0.0;
  pub_cmd_vel_->publish(cmd_vel_msg);

  // Pretend that, after a certain amount of time,
  // we have completed the operation
  RCLCPP_INFO(node_->get_logger(), "Yaw current [%.5f] ", yaw);
  RCLCPP_INFO(node_->get_logger(), "Yaw init [%.5f] ", yaw_init);
  float error_angle = abs(calculate_angle_rotated(yaw_init, yaw));
  // float error_angle = yaw - yaw_init;
  RCLCPP_INFO(node_->get_logger(), "angle rotated [%.8f] ", error_angle);
  RCLCPP_ERROR(node_->get_logger(), "Contador erros [%d] ", contador_error);
  contador_error++;
  if (abs(error_angle) > angle_rotate_ && contador_error > 100) {
    RCLCPP_ERROR(node_->get_logger(), "Stop robot ");
    cmd_vel_msg.angular.z = 0.0;
    cmd_vel_msg.linear.x = 0.0;
    pub_cmd_vel_->publish(cmd_vel_msg);
    if (calcular_deep_shelf_ == true) {
      // hallo la posicion del robot
      Point result = filterAndAveragePoints(position_deep_shelf_points, 1.0);
      RCLCPP_INFO(node_->get_logger(), "Shelf localizado x [%.3f] y [%.3f]",
                  result.x, result.y);
      geometry_msgs::msg::Point position_deep_shelf_end;
      position_deep_shelf_end.x = result.x;
      position_deep_shelf_end.y = result.y;
      setOutput<geometry_msgs::msg::Point>("position_deep_shelf_end",
                                           position_deep_shelf_end);
      geometry_msgs::msg::PointStamped point_stamped;
      point_stamped.header.frame_id = "map";
      point_stamped.header.stamp = node_->now();
      point_stamped.point.x = position_deep_shelf_end.x;
      point_stamped.point.y = position_deep_shelf_end.y;
      point_stamped.point.z = 0.5;
      pub_position_->publish(point_stamped);

      // pub market
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "map";
      marker.header.stamp = node_->get_clock()->now();
      marker.ns = "basic_shapes";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      // Set the pose of the marker
      marker.pose.position.x = position_deep_shelf_end.x;
      marker.pose.position.y = position_deep_shelf_end.y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker
      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;

      // Set the color of the marker
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
      pub_market_->publish(marker);
      // publico la transformacion
      geometry_msgs::msg::TransformStamped transform_;
      transform_.header.frame_id = "map"; // Marco de referencia fuente
      transform_.child_frame_id =
          "shelf_deep_frame"; // Marco de referencia objetivo
      transform_.header.stamp = node_->get_clock()->now();
      // Definir la transformación estática (traslación y rotación)
      /*transform_.transform.translation.x = 0;
      transform_.transform.translation.y = 0;*/
      transform_.transform.translation.x = position_deep_shelf_end.x;
      transform_.transform.translation.y = position_deep_shelf_end.y;

      transform_.transform.translation.z = 0.0;
      transform_.transform.rotation.x = 0.0;
      transform_.transform.rotation.y = 0.0;
      transform_.transform.rotation.z = 0.0;
      transform_.transform.rotation.w = 1.0;
      broadcaster_1->sendTransform(transform_);
    }

    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Turn robot ");
    return BT::NodeStatus::RUNNING;
  }
}

void TurnRobot::onHalted() { printf("[ MoveBase: ABORTED ]"); }