
#pragma once

#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep
#include "rclcpp/rclcpp.hpp"


#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <behaviortree_cpp/action_node.h>

using namespace std::placeholders;

class TurnRobot : public BT::StatefulActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    TurnRobot(const std::string& name)
      : StatefulActionNode(name, {})
    {
        node_ = rclcpp::Node::make_shared("bt_turn_robot_node");
        pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
        sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>("odom",10, std::bind(&TurnRobot::odomCallback,this,_1));

        node_->declare_parameter("vel_turn",0.4);

        vel_turn_ = node_->get_parameter("vel_turn").as_double();

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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    std::chrono::system_clock::time_point _completion_time;
    geometry_msgs::msg::Twist cmd_vel_msg;
    float vel_turn_;

    double yaw;
    double yaw_init;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom){
      
      double roll, pitch;
      tf2::Quaternion quaternion(odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z,odom->pose.pose.orientation.w);
      tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
      if (yaw< 0) {
        yaw = yaw + 2 * M_PI;
      }
    }

};

//-------------------------

BT::NodeStatus TurnRobot::onStart()
{

  RCLCPP_ERROR(node_->get_logger(), "Find shelf init ");
  rclcpp::spin_some(node_);
  yaw_init = yaw;

  // We use this counter to simulate an action that takes a certain
  // amount of time to be completed (200 ms)
  cmd_vel_msg.angular.z = vel_turn_;
  cmd_vel_msg.linear.x = 0.0;
  pub_cmd_vel_->publish(cmd_vel_msg);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnRobot::onRunning()
{
  // Pretend that we are checking if the reply has been received
  // you don't want to block inside this function too much time.

  cmd_vel_msg.angular.z = vel_turn_;
  cmd_vel_msg.linear.x = 0.0;
  pub_cmd_vel_->publish(cmd_vel_msg);

  // Pretend that, after a certain amount of time,
  // we have completed the operation
  if( abs(yaw_init - yaw) > M_PI )
  {
    RCLCPP_ERROR(node_->get_logger(), "Stop robot ");

    cmd_vel_msg.angular.z = 0.0;
    cmd_vel_msg.linear.x = 0.0;
    pub_cmd_vel_->publish(cmd_vel_msg);
    return BT::NodeStatus::SUCCESS;
  }
  else{
    RCLCPP_ERROR(node_->get_logger(), "Turn robot ");
    return BT::NodeStatus::RUNNING;
  }
}

void TurnRobot::onHalted()
{
  printf("[ MoveBase: ABORTED ]");
}